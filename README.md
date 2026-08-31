# free-D — tracking caméra virtuelle tout-Vive, sans SteamVR

Chaîne de tracking pour production virtuelle sous Linux : pose 6DoF d'un tracker Vive via
**libsurvive**, zoom et focus lus comme rotation relative de deux autres trackers, le tout
émis en **Free-D** vers Unreal Engine (plugin LiveLinkFreeD).

**Ni SteamVR, ni casque, ni compositeur VR.** Le retour vidéo sort en SDI vers un moniteur
caméra.

Spécification complète : [`docs/HANDOFF-free-D-v5.md`](docs/HANDOFF-free-D-v5.md).

## État — 30 août 2026

**Le code est complet et ses huit auto-tests passent**, tous en code de sortie 0. Rien n'a
encore tourné sur le matériel réel : toute la validation terrain reste à faire.

```
bridge/freed.py                    OK — encodage/décodage, checksum valide
bridge/lensaxis.py                 OK — axe, multi-tour, alignement temporel, chien de garde
bridge/worldframe.py               OK — sol, écran, ligne médiane, caméra et base stations
bridge/survive_clock.py            OK — unité détectée, offset calé sur le plancher, replis sûrs
tools/calib-axis.py    --selftest  OK — verdicts de montage, absolu/multi-tour, alerte caméra
tools/calib-world.py   --selftest  OK — trois points suffisent
tools/test-decouple.py --selftest  OK — le mouvement caméra est correctement soustrait
tools/vp-console.py    --selftest  OK — les trois onglets enchaînés, gardes actives (~30 s)
```

Le dernier est le seul à exercer la machine à états complète : trois relevés → résolution →
balayage d'axe → phases de test, refus attendus compris. Les autres ne valident que la math
de leur module.

Deux mesures au-delà des auto-tests, faites sur cet arbre :

- **Découplage caméra/objectif**, phase roulis lue en direct dans la console en `--demo` :
  crête naïve 0,726° → alignée 0,0035°, soit **1 count sur 65535**. C'est ce que le §6bis
  du handoff cherchait à démontrer, et l'auto-test de la console retrouve les mêmes chiffres.
- **Sortie Free-D** (`vp_bridge.py --source simulate`) : 267 paquets de 29 octets à 60 Hz,
  **aucun rejeté** par `freed.decode_d1()`. L'étape 2 du §9 passe avant même qu'Unreal soit
  dans la boucle.

⚠️ Aucun de ces chiffres ne vient du matériel. Ils mesurent la cohérence du code avec les
constantes que la démo injecte elle-même — le §11 du handoff met en garde contre exactement
cette lecture. Ce qu'ils ne peuvent pas dire est au §10.

⚠️ En particulier, l'auto-test de `survive_clock.py` valide sa logique de repli sur données
fabriquées, **pas** la présence réelle d'un horodatage dans ta version de pysurvive. Seule
la sonde répond, trackers branchés — et c'est la première commande à lancer après la
première mise sous tension :

```bash
python3 bridge/survive_clock.py --probe
```

## Arborescence

```
bridge/      la chaîne temps réel
  freed.py           encodage du protocole Free-D D1
  survive_clock.py   découverte de l'horloge libsurvive
  lensaxis.py        quaternions, SVD, swing-twist, multi-tour, one-euro
  worldframe.py      plan de sol, repère plateau, lecture des base stations
  vp_bridge.py       3 trackers → Free-D, applique world.json et axes.json
  requirements.txt   numpy >= 1.24 (plus pysurvive, hors PyPI)
tools/       calibration et diagnostic
  vp-console.py      serveur web, 3 onglets — Studio, Objectifs, Test
  calib-world.py     équivalent CLI de l'onglet Studio
  calib-axis.py      équivalent CLI de l'onglet Objectifs
  test-decouple.py   équivalent CLI de l'onglet Test
system/      vp-bridge.service
install.sh   mise à niveau d'une station : paquets, venv, libsurvive, udev, service
docs/        HANDOFF-free-D-v5.md — la source de vérité
archive-v2/  architecture abandonnée, conservée pour mémoire
```

## Installation

```bash
./install.sh                # paquets, venv, libsurvive, service, auto-tests
./install.sh --dry-run      # ce qu'il ferait, sans rien exécuter
```

Il est idempotent et **n'active aucun service** : libsurvive est exclusif, et le bridge
sort en erreur tant que `axes.json` n'existe pas. Options utiles : `--skip-apt`,
`--skip-libsurvive` (venv et numpy seuls, suffisant pour `--demo`), `--check` pour
relancer les huit auto-tests.

Ce qu'il installe, et pourquoi c'est plus qu'un `pip install` :

- les paquets système, dont **`python3-venv`** — une Ubuntu Studio nue n'a ni `pip` ni
  `venv`, et `python3 -m venv` échoue alors sur `ensurepip` ;
- le venv dans `.venv/`, puis numpy ;
- **libsurvive et pysurvive**, compilés depuis les sources : hors PyPI, avec leurs
  dépendances de build ;
- la règle udev **`81-vive.rules`**, sans laquelle les trackers ne sont lisibles que par
  root et libsurvive ne voit aucun périphérique ;
- le bit exécutable sur les scripts, que le service et les commandes ci-dessous supposent ;
- l'unité systemd utilisateur, réécrite pour pointer sur **l'interpréteur du venv** :
  systemd n'active pas d'environnement, donc le shebang `env python3` du bridge trouverait
  le python système, sans numpy ni pysurvive.

Les trackers doivent être en **USB filaire direct** — libsurvive ne sait pas appairer.

## Démarrage

```bash
source .venv/bin/activate
tools/vp-console.py --selftest      # ~30 s, sans rien brancher
systemctl --user stop vp-bridge     # libsurvive est exclusif : un seul processus
tools/vp-console.py --demo          # l'interface, sans matériel
tools/vp-console.py                 # puis http://127.0.0.1:8410
```

Séquence de validation complète au §9 du handoff. Ne pas sauter l'étape 2, qui isole les
problèmes Unreal des problèmes de tracking.

## archive-v2/ — ce qui a été abandonné

La version 2 lisait le zoom et le focus par des **encodeurs magnétiques AS5600** sur un
RP2040 en CircuitPython, reliés en série. Le §1 du handoff v5 retire cette voie au profit
de trois trackers Vive : une seule horloge, un seul bus, une seule chaîne de latence.

Disparaissent avec elle le firmware CircuitPython, le protocole série `E:F:...`, la règle
udev `99-vp-encoders.rules` et le conflit de port CDC unique. Le matériel correspondant
(aimants diamétraux, breakouts, RP2040) est listé dans `archive-v2/HARDWARE.md`.

Conservé pour mémoire, **pas maintenu**.

## Projet voisin

La station qui reçoit ce flux fait l'objet d'un dépôt distinct, `UnrealCAMERA`, où une
**voie concurrente** est en production : SteamVR + OpenXR, qui exige un casque pour démarrer
son compositeur. Les deux approches visent le même résultat ; celle-ci s'affranchit du
casque, celle-là est prouvée jusqu'aux poses.
