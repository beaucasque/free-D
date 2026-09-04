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

## Première mise sous tension — 31 août 2026

Les trois trackers ont tourné. Ce qui est désormais **mesuré, pas supposé** :

- **L'horodatage existe.** `Pose()[1]` est l'horloge de libsurvive, en **secondes**.
  52 700 poses en 90 s, retard de file 3,8 ms (p95). C'était la première question ouverte
  du §10 ; l'intuition du §6bis était juste et rien n'était à changer.
- **Les deux base stations sont résolues**, au plafond à 2,12 et 2,14 m, de part et
  d'autre de la médiane, écartées de 4,38 m — la topologie du §2.
- Trackers `28de:2300` en USB direct, séries `LHR-F3D3F946`, `LHR-BDBF93F3`,
  `LHR-9A85D671`, sur hub auto-alimenté. Règle udev `81-vive.rules` posée.

Cette mise sous tension a révélé **deux bugs qu'aucun test hors matériel ne pouvait
voir** — le §11 met en garde contre exactement cette confiance :

- `read_lighthouses()` n'aurait **jamais** lu une base station. Le `config.json` de
  libsurvive n'est pas du JSON valide (pas d'accolades englobantes, pas de virgule entre
  groupes), la position est le champ `pose` du groupe `lighthouseN` et non une clé plate,
  et tous les nombres sont écrits comme des chaînes. Le §4 en dépendait pour orienter la
  normale du sol et pour le diagnostic d'installation.
- Une pose de base station **toute à zéro** avec `PositionSet=0` était prise pour une
  position réelle, plaçant une station à l'origine du plateau.

## 1er septembre 2026

- **Le hub doit être multi-TT**, pas seulement alimenté. Quatre trackers full-speed
  derrière un hub *single-TT* saturent son transaction translator et faisaient planter
  libsurvive en 2,7 s, systématiquement ; trois sur le même hub tenaient des heures.
  `cat /sys/bus/usb/devices/<hub>/bDeviceProtocol` — `02` = multi-TT. Un hub « USB 3.0 »
  est deux hubs dans un boîtier : c'est la partie 2.0 qui compte. Voir §2 du handoff.
- **Les appareils s'identifient par leur numéro de série gravé.** `Name()` renvoie `T20`,
  `T21`… un rang d'énumération qui se décale dès qu'on branche un appareil de plus — vérifié.
  `survive_simple_serial_number()` donne `LHR-F3D3F946`, stable. C'était une question
  ouverte du §10, elle est réglée.
- **Quatre fonctions attribuées à la main** dans l'onglet Appareils, avant tout relevé :
  caméra, zoom, focus, relevé. `bridge/roles.json` lie la fonction au numéro de série ;
  remplacer un tracker se fait en réattribuant sa fonction, et la calibration devenue
  caduque est effacée d'elle-même. Voir §4 du handoff.
- **Deux plantages de libsurvive corrigés**, dans `patches/`, appliqués par `install.sh` :
  un transfert resoumis était ensuite libéré sous libusb, et le callback de complétion
  réentrait dans libusb. Soumis en amont —
  [collabora/libsurvive#372](https://github.com/collabora/libsurvive/pull/372). Vérifié
  sur matériel : débrancher un tracker en marche ne tue plus le processus.

Restent ouvertes, et hors de portée sans manipulation physique : la **sémantique** de
l'horodatage (résolution ou balayage — §6bis), le sens de rotation, la course des bagues,
et `--floor-offset-mm`.

## Arborescence

```
bridge/      la chaîne temps réel
  freed.py           encodage du protocole Free-D D1
  survive_clock.py   découverte de l'horloge libsurvive
  lensaxis.py        quaternions, SVD, swing-twist, multi-tour, one-euro
  worldframe.py      plan de sol, repère plateau, lecture des base stations
  vp_bridge.py       3 trackers → Free-D, applique world.json et axes.json
  requirements.txt   numpy >= 1.24 (plus pysurvive, hors PyPI)
  roles.json         quelle fonction pour quel n° de série (produit par la console)
  presets/           configurations de studio nommées (rôles + repère + axes)
tools/       calibration et diagnostic
  vp-console.py      serveur web, 5 onglets — Appareils, Studio, Objectifs,
                     Test, Sortie ; bandeau de santé permanent
  calib-world.py     équivalent CLI de l'onglet Studio
  calib-axis.py      équivalent CLI de l'onglet Objectifs
  test-decouple.py   équivalent CLI de l'onglet Test
  reset-trackers.py  réinitialise les trackers sans sudo ni redémarrage
system/      vp-bridge.service, vp-console.service — supervisés, exclusifs l'un de l'autre
patches/     correctifs appliqués à libsurvive avant compilation (voir patches/README.md)
install.sh   mise à niveau d'une station : paquets, venv, libsurvive, udev, services
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
