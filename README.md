# free-D — tracking caméra virtuelle tout-Vive, sans SteamVR

Chaîne de tracking pour production virtuelle sous Linux : pose 6DoF d'un tracker Vive via
**libsurvive**, zoom et focus lus comme rotation relative de deux autres trackers, le tout
émis en **Free-D** vers Unreal Engine (plugin LiveLinkFreeD).

**Ni SteamVR, ni casque, ni compositeur VR.** Le retour vidéo sort en SDI vers un moniteur
caméra.

Spécification complète : [`docs/HANDOFF-free-D-v4.md`](docs/HANDOFF-free-D-v4.md).

## État — 30 août 2026

**Le code est complet et ses auto-tests passent.** Rien n'a encore tourné sur le matériel
réel : toute la validation terrain reste à faire.

```
bridge/freed.py           OK — encodage/décodage cohérents, checksum valide
bridge/lensaxis.py        OK — axe, multi-tour, alignement temporel, chien de garde
bridge/worldframe.py      OK — sol, écran, ligne médiane, caméra et base stations
bridge/survive_clock.py   OK — unité détectée, offset calé sur le plancher, replis sûrs
tools/calib-world.py      OK — trois points suffisent
tools/test-decouple.py    Chaîne validée — le mouvement caméra est correctement soustrait
```

Le chiffre le plus parlant vient du dernier : **alignement temporel 106 fois meilleur que
le naïf, résidu 0,01 ms** — c'est ce que le §6bis du handoff cherchait à démontrer.

⚠️ `calib-axis.py`, `gui-decouple.py` et `vp-console.py` **n'ont pas** de `--selftest`,
contrairement à ce qu'annonce le §5 du handoff.

⚠️ `survive_clock.py` s'exécute en mode démonstration : il valide sa logique de repli, pas
la présence réelle d'un horodatage exposé par pysurvive. C'est la première question ouverte
du §10.

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
  gui-decouple.py    interface de découplage
  calib-world.py     équivalent CLI de l'onglet Studio
  calib-axis.py      équivalent CLI de l'onglet Objectifs
  test-decouple.py   équivalent CLI de l'onglet Test
system/      vp-bridge.service
docs/        HANDOFF-free-D-v4.md — la source de vérité
archive-v2/  architecture abandonnée, conservée pour mémoire
```

## Prérequis

```bash
pip install -r bridge/requirements.txt      # numpy
# puis construire libsurvive et ses bindings Python (pysurvive), hors PyPI
```

Les trackers doivent être en **USB filaire direct** — libsurvive ne sait pas appairer.

## Démarrage

```bash
systemctl --user stop vp-bridge     # libsurvive est exclusif : un seul processus
tools/vp-console.py --demo          # sans matériel
tools/vp-console.py                 # puis http://127.0.0.1:8410
```

Séquence de validation complète au §9 du handoff. Ne pas sauter l'étape 2, qui isole les
problèmes Unreal des problèmes de tracking.

## archive-v2/ — ce qui a été abandonné

La version 2 lisait le zoom et le focus par des **encodeurs magnétiques AS5600** sur un
RP2040 en CircuitPython, reliés en série. Le §1 du handoff v4 retire cette voie au profit
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
