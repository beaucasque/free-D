#!/usr/bin/env bash
#
# install.sh — met une station Ubuntu a niveau pour free-D.
#
# Idempotent : on peut le relancer sans rien casser. Il ne demarre aucun
# service et ne touche pas aux calibrations.
#
#   ./install.sh                 tout : apt, venv, libsurvive, unite, tests
#   ./install.sh --dry-run       affiche ce qui serait fait, n'execute rien
#   ./install.sh --skip-apt      si les paquets systeme sont deja la
#   ./install.sh --skip-libsurvive   venv + numpy seulement (mode --demo)
#   ./install.sh --check         relance seulement les huit auto-tests
#
# CE QU'IL N'ACTIVE PAS
#   Le service vp-bridge est installe mais laisse a l'arret. libsurvive est
#   exclusif : un seul processus parle aux trackers, et la console doit
#   pouvoir tourner. De plus le bridge sort en erreur tant que axes.json
#   n'existe pas — l'activer avant calibration ne ferait qu'une boucle de
#   redemarrages dans le journal.

set -euo pipefail

REPO="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
VENV="$REPO/.venv"
PY="$VENV/bin/python"
LIBSURVIVE_DIR="${LIBSURVIVE_DIR:-$HOME/src/libsurvive}"
UNIT_DIR="$HOME/.config/systemd/user"

DO_APT=1; DO_VENV=1; DO_LIBSURVIVE=1; DO_CHMOD=1; DO_UNIT=1; DO_CHECK=1; DRY=0

PKGS_BASE=(python3-venv python3-dev build-essential)
# Dependances de compilation de libsurvive. La liste vient de son README ;
# lapack/openblas servent au solveur de pose, libusb au dialogue avec les
# trackers, freeglut aux outils de visualisation qu'on n'utilise pas mais
# que le CMake reclame.
PKGS_SURVIVE=(cmake git zlib1g-dev libx11-dev libusb-1.0-0-dev
              freeglut3-dev liblapacke-dev libopenblas-dev libatlas-base-dev)

# Couleur seulement sur un terminal : redirige vers un fichier ou un
# journal, la sortie reste lisible.
if [ -t 1 ]; then B=$'\033[1m'; Y=$'\033[33m'; R=$'\033[31m'; N=$'\033[0m'
else B=""; Y=""; R=""; N=""; fi

say()  { printf '\n%s== %s%s\n' "$B" "$*" "$N"; }
info() { printf '   %s\n' "$*"; }
warn() { printf '%s   ATTENTION : %s%s\n' "$Y" "$*" "$N" >&2; }
die()  { printf '%s   ERREUR : %s%s\n' "$R" "$*" "$N" >&2; exit 1; }

run() {
    if [ "$DRY" = 1 ]; then printf '   [dry-run] %s\n' "$*"; else "$@"; fi
}

# S'arrete a la premiere ligne non commentee : pas de plage de lignes a
# tenir a jour quand l'en-tete bouge.
usage() { sed -n '2,${/^#/!q; s/^# \{0,1\}//p;}' "$0"; exit 0; }

while [ $# -gt 0 ]; do
    case "$1" in
        --dry-run)         DRY=1 ;;
        --skip-apt)        DO_APT=0 ;;
        --skip-libsurvive) DO_LIBSURVIVE=0 ;;
        --skip-unit)       DO_UNIT=0 ;;
        --check)           DO_APT=0; DO_VENV=0; DO_LIBSURVIVE=0; DO_CHMOD=0; DO_UNIT=0 ;;
        -h|--help)         usage ;;
        *)                 die "option inconnue : $1  (--help)" ;;
    esac
    shift
done

# ------------------------------------------------------- 0. surtout pas root

# Lance sous sudo, $HOME devient /root : le venv serait crée root dans le
# depot (plus writable par l'utilisateur), libsurvive irait dans /root/src,
# et l'unite systemd UTILISATEUR atterrirait dans /root/.config ou la
# session de l'utilisateur ne la verrait jamais. Le script demande sudo
# lui-meme, uniquement pour apt et la regle udev.
if [ "$(id -u)" = 0 ]; then
    die "ne pas lancer ce script en root / sous sudo — le lancer comme
   l'utilisateur normal : il demandera sudo pour apt et la regle udev.
   Si vous etes vraiment root sans utilisateur, exportez SUDO_USER."
fi

# ------------------------------------------------------------------ 0. etat

say "Station"
info "depot        $REPO"
info "venv         $VENV"
[ "$DO_LIBSURVIVE" = 1 ] && info "libsurvive   $LIBSURVIVE_DIR"
if [ -r /etc/os-release ]; then
    . /etc/os-release
    info "systeme      ${PRETTY_NAME:-inconnu}"
    case "${ID:-}" in
        ubuntu|debian|linuxmint|pop) ;;
        *) warn "distribution non testee : les noms de paquets apt peuvent differer" ;;
    esac
fi
[ -f "$REPO/bridge/vp_bridge.py" ] || die "ce script doit vivre a la racine du depot free-D"

# ------------------------------------------------------------------ 1. apt

if [ "$DO_APT" = 1 ]; then
    say "Paquets systeme"
    pkgs=("${PKGS_BASE[@]}")
    [ "$DO_LIBSURVIVE" = 1 ] && pkgs+=("${PKGS_SURVIVE[@]}")

    missing=()
    for p in "${pkgs[@]}"; do
        dpkg-query -W -f='${Status}' "$p" 2>/dev/null | grep -q "^install ok installed$" \
            || missing+=("$p")
    done

    if [ ${#missing[@]} -eq 0 ]; then
        info "deja installes : ${pkgs[*]}"
    else
        info "manquants : ${missing[*]}"
        info "sudo va etre demande."
        run sudo apt-get update
        run sudo apt-get install -y "${missing[@]}"
    fi
fi

# ------------------------------------------------------------------ 2. venv

if [ "$DO_VENV" = 1 ]; then
    say "Environnement Python"
    # `python3 -m venv --help` reussit meme quand python3-venv manque : le
    # module est la, c'est l'amorcage de pip qui ne l'est pas. Seul l'import
    # d'ensurepip repond vraiment.
    python3 -c "import ensurepip" >/dev/null 2>&1 \
        || die "le paquet python3-venv manque — relancer sans --skip-apt, ou : sudo apt install python3-venv"

    if [ -x "$VENV/bin/pip" ]; then
        info "venv deja present"
    else
        # Une creation interrompue laisse un .venv sans pip, dont bin/python
        # est un lien vers le python SYSTEME : il a l'air valide, passe le
        # test -x, et ne trouve aucune dependance. On repart de zero.
        if [ -e "$VENV" ]; then
            info "venv incomplet, recreation"
            run rm -rf "$VENV"
        fi
        run python3 -m venv "$VENV"
    fi
    run "$VENV/bin/pip" install --quiet --upgrade pip
    run "$VENV/bin/pip" install --quiet -r "$REPO/bridge/requirements.txt"
    [ "$DRY" = 1 ] || info "numpy $("$PY" -c 'import numpy; print(numpy.__version__)')"
fi

# ------------------------------------------------- 3. libsurvive + pysurvive

if [ "$DO_LIBSURVIVE" = 1 ]; then
    say "libsurvive et pysurvive"
    info "hors PyPI : compile depuis les sources, comme dit requirements.txt"

    # Rien a recompiler si le venv importe deja pysurvive ET qu'il a ete
    # construit avec exactement les correctifs presents. L'empreinte evite
    # le piege inverse de la simple presence : un pysurvive installe AVANT
    # l'ajout d'un correctif serait reste non corrige, en silence.
    STAMP="$VENV/.freed-libsurvive"
    want=""
    if ls "$REPO"/patches/*.patch >/dev/null 2>&1; then
        want=$(cat "$REPO"/patches/*.patch | sha256sum | cut -d" " -f1)
    else
        want="sans-correctif"
    fi
    have=$(cat "$STAMP" 2>/dev/null || echo "")

    if [ "$DRY" = 0 ] && [ -x "$PY" ] \
       && "$PY" -c "import pysurvive" >/dev/null 2>&1 \
       && [ "$have" = "$want" ]; then
        info "pysurvive deja installe avec les correctifs courants"
        info "forcer : rm $STAMP puis relancer"
    else
        [ -n "$have" ] && [ "$have" != "$want" ] \
            && info "les correctifs ont change depuis la derniere compilation"
        if [ -d "$LIBSURVIVE_DIR/.git" ]; then
            info "depot deja clone, mise a jour"
            run git -C "$LIBSURVIVE_DIR" fetch -q origin
        else
            run mkdir -p "$(dirname "$LIBSURVIVE_DIR")"
            run git clone --recursive https://github.com/collabora/libsurvive "$LIBSURVIVE_DIR"
        fi

        # On repart TOUJOURS d'origin/master puis on rejoue nos correctifs :
        # deterministe, et relancable sans conflit. Un `git pull` sur une
        # branche portant nos commits echouerait.
        if [ -d "$REPO/patches" ] && ls "$REPO"/patches/*.patch >/dev/null 2>&1; then
            say "Correctifs libsurvive"
            run git -C "$LIBSURVIVE_DIR" checkout -q -B freed-patched origin/master
            run git -C "$LIBSURVIVE_DIR" submodule -q update --init --recursive
            for pf in "$REPO"/patches/*.patch; do
                if [ "$DRY" = 1 ]; then
                    info "[dry-run] git am $(basename "$pf")"
                    continue
                fi
                # Identite fournie ici : une station neuve n'a pas
                # forcement de git config --global, et `git am` refuse de
                # commiter sans.
                if git -C "$LIBSURVIVE_DIR" \
                       -c user.name="free-D install" \
                       -c user.email="install@free-d.local" \
                       am -q --keep-cr < "$pf"; then
                    info "applique  $(basename "$pf")"
                else
                    git -C "$LIBSURVIVE_DIR" am --abort >/dev/null 2>&1 || true
                    die "le correctif $(basename "$pf") ne s'applique plus sur
   origin/master. libsurvive a bouge : relire le patch, ou le retirer de
   patches/ si l'amont a corrige le probleme."
                fi
            done
        fi

        # Dans le venv, pas sur le systeme : c'est ce python-la que le bridge
        # et la console utiliseront.
        #
        # --no-deps est indispensable. Le setup.py de libsurvive declare
        # install_requires=['gooey'], qui tire wxPython, qui se compile depuis
        # les sources et echoue sans les en-tetes GTK3 — faisant echouer TOUTE
        # l'installation alors que pysurvive, lui, avait compile. Or free-D
        # n'en a aucun besoin : gooey n'est importe que par
        # pysurvive/__main__.py, l'entree GUI, et matplotlib/scipy seulement
        # par recorder.py. `import pysurvive` ne demande que la bibliotheque
        # standard et les bindings ctypes generes. Le §6 du handoff pose
        # d'ailleurs l'absence de dependance GUI comme une regle.
        run "$VENV/bin/pip" install --no-deps "$LIBSURVIVE_DIR"
        [ "$DRY" = 1 ] || printf '%s\n' "$want" > "$STAMP"
    fi

    # La regle udev est posee DANS TOUS LES CAS, meme quand la compilation a
    # ete sautee : pysurvive peut tres bien etre installe alors que la regle
    # ne l'est pas — c'est exactement ce qui arrive apres un echec sur
    # wxPython, qui interrompait le script avant cette etape.
    rules="$LIBSURVIVE_DIR/useful_files/81-vive.rules"
    if [ "$DRY" = 1 ] || [ -f "$rules" ]; then
        say "Regle udev"
        if [ "$DRY" = 0 ] && cmp -s "$rules" /etc/udev/rules.d/81-vive.rules 2>/dev/null; then
            info "deja posee et identique"
        else
            info "sans elle les trackers ne sont accessibles qu'a root"
            run sudo install -m 0644 "$rules" /etc/udev/rules.d/81-vive.rules
            run sudo udevadm control --reload-rules
            run sudo udevadm trigger
            info "debrancher/rebrancher les trackers pour que la regle prenne effet"
        fi
    else
        warn "81-vive.rules introuvable dans $LIBSURVIVE_DIR/useful_files/"
        warn "les trackers risquent de n'etre lisibles que par root"
    fi
fi

# ------------------------------------------------------- 4. bits executables

if [ "$DO_CHMOD" = 1 ]; then
say "Bits executables"
# Le service lance bridge/vp_bridge.py directement : sans le bit +x il sort
# en 203/EXEC. Les outils ont le meme probleme avec les commandes du README.
for f in "$REPO"/tools/*.py "$REPO/bridge/vp_bridge.py"; do
    if [ -x "$f" ]; then
        info "deja executable  $(basename "$f")"
    else
        info "a rendre executable : $(basename "$f")"
        run chmod +x "$f"
    fi
done
fi

# ------------------------------------------------------- 5. unite systemd

if [ "$DO_UNIT" = 1 ]; then
    say "Service utilisateur (installe, PAS active)"
    for unit in vp-bridge vp-console; do
    src="$REPO/system/$unit.service"
    [ -f "$src" ] || die "$src introuvable"

    # Le fichier versionne suppose le depot dans ~/free-D et compte sur le
    # shebang. Aucun des deux ne tient ici : systemd n'active pas de venv,
    # donc `env python3` trouverait le python systeme, sans numpy ni
    # pysurvive. On ecrit donc l'interpreteur du venv en clair, et le vrai
    # chemin du depot.
    run mkdir -p "$UNIT_DIR"
    if [ "$unit" = "vp-bridge" ]; then
        exec_line="ExecStart=$PY $REPO/bridge/vp_bridge.py --source survive --host 127.0.0.1 --rate 60"
    else
        exec_line="ExecStart=$PY $REPO/tools/vp-console.py --host \${VP_HOST} --port 8410"
    fi
    if [ "$DRY" = 1 ]; then
        info "[dry-run] generation de $UNIT_DIR/$unit.service"
        info "[dry-run]   $exec_line"
    else
        sed -e "s|%h/free-D|$REPO|g" \
            -e "s|^ExecStart=.*|$exec_line|" \
            "$src" > "$UNIT_DIR/$unit.service"
        info "ecrit  $UNIT_DIR/$unit.service"
    fi
    done
    run systemctl --user daemon-reload
    info "interpreteur du venv, chemin du depot resolu"
    info "les deux sont laisses A L'ARRET et s'excluent (Conflicts=) :"
    info "libsurvive n'admet qu'un processus, et le bridge sort en erreur"
    info "tant que axes.json n'existe pas."
fi

# ------------------------------------------------------------ 6. auto-tests

if [ "$DO_CHECK" = 1 ] && [ "$DRY" = 0 ]; then
    say "Auto-tests (~35 s)"
    { [ -x "$PY" ] && "$PY" -c "import numpy" >/dev/null 2>&1; } \
        || die "venv absent ou incomplet — relancer ./install.sh sans --check"

    fail=0
    for m in freed lensaxis worldframe survive_clock; do
        if "$PY" "$REPO/bridge/$m.py" >/dev/null 2>&1; then
            printf '   %-34s OK\n' "bridge/$m.py"
        else
            printf '%s   %-34s ECHEC%s\n' "$R" "bridge/$m.py" "$N"; fail=1
        fi
    done
    for t in calib-axis calib-world test-decouple vp-console; do
        if "$PY" "$REPO/tools/$t.py" --selftest >/dev/null 2>&1; then
            printf '   %-34s OK\n' "tools/$t.py --selftest"
        else
            printf '%s   %-34s ECHEC%s\n' "$R" "tools/$t.py --selftest" "$N"; fail=1
        fi
    done
    [ "$fail" = 0 ] || die "un auto-test a echoue — ne pas brancher le materiel"
    info "les huit passent"
elif [ "$DO_CHECK" = 1 ]; then
    say "Auto-tests"
    info "[dry-run] les huit auto-tests seraient lances ici"
fi

# --------------------------------------------------------------- 7. la suite

say "Ensuite"
cat <<EOS
   Activer l'environnement :
       source $VENV/bin/activate

   Trackers branches, la PREMIERE commande a lancer (§10 du handoff) —
   tout le reste depend de sa reponse :
       $PY bridge/survive_clock.py --probe

   Puis la console, etapes 3 a 6 de la sequence du §9 :
       systemctl --user stop vp-bridge
       $PY tools/vp-console.py --demo     # sans materiel
       $PY tools/vp-console.py            # http://127.0.0.1:8410

   Aucun chiffre produit sans materiel ne dit quoi que ce soit du plateau
   reel : la demo ne mesure que la coherence du code avec ses propres
   constantes (§11 du handoff).
EOS
