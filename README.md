# vp-tracking

Chaîne de tracking pour production virtuelle sous Linux, **sans SteamVR et sans casque** :
pose 6DoF d'un tracker Vive via libsurvive, zoom et focus via encodeurs magnétiques,
le tout émis en Free-D vers Unreal Engine.

Conçu pour un plateau où le retour vidéo sort en SDI vers un moniteur caméra —
pas de HMD, pas de compositeur VR sur la machine de rendu.

## Architecture

```
2× base stations 2.0
    ↓ (IR)
Vive Tracker 3.0 sur la caméra ──── USB ────┐
                                            │
2× AS5600 (bagues zoom / focus)             │
    ↓ I²C                                   │
RP2040-Zero ─────────── USB CDC ────────────┤
                                            ↓
                                  Ubuntu Studio 22.04
                                  libsurvive + vp_bridge.py
                                            ↓ UDP:40000
                                  Unreal Engine 5.8
                                  LiveLinkFreeD → CineCameraActor
                                            ↓
                                  Composure → DeckLink SDI 4K
```

**Pourquoi des encodeurs plutôt que des trackers sur les bagues :** un Tracker 3.0
pèse 75 g et se déséquilibre en porte-à-faux sur une roulette de follow focus ;
il s'occulte contre le corps de l'objectif ; et son comptage multi-tour se perd
à chaque décrochage optique. Un AS5600 coûte 7 $, ne s'occulte jamais, et ne
perd jamais le compte.

## Ordre de validation

Chaque étape n'introduit qu'une seule inconnue. **Ne pas sauter d'étape** —
c'est tout l'intérêt du découpage.

| # | Étape | Commande | Ce qu'on valide |
|---|---|---|---|
| 1 | Unreal seul | `bridge/vp_bridge.py --source simulate` | Plugin LiveLinkFreeD, CineCameraActor, sens des axes |
| 2 | Aimants | `tools/test-sweep.py` | Aimantation diamétrale, entrefer, linéarité |
| 3 | Encodeurs | `bridge/vp_bridge.py --source serial` | Mapping zoom/focus dans Unreal |
| 4 | Tracker seul | `survive-cli` | Calibration lighthouses, stabilité de la pose |
| 5 | Chaîne complète | `bridge/vp_bridge.py --source survive` | Fusion, latence, alignement vidéo |

L'étape 1 ne demande **aucun matériel** : elle se lance aujourd'hui, avant
même que les aimants soient livrés.

## Démarrage

```bash
git clone <url> ~/vp-tracking
cd ~/vp-tracking
python3 -m venv .venv && source .venv/bin/activate
pip install -r bridge/requirements.txt
```

Puis, dans l'ordre :

```bash
# Identifier la carte et générer la règle udev
tools/find-device.sh

sudo cp system/99-vp-encoders.rules /etc/udev/rules.d/   # après avoir mis le n° de série
sudo udevadm control --reload-rules && sudo udevadm trigger

# Déployer le firmware
tools/deploy-firmware.sh

# Diagnostiquer les aimants
tools/test-sweep.py --sensor F --duration 15
tools/test-sweep.py --sensor Z --duration 15

# Émettre vers Unreal
bridge/vp_bridge.py --source simulate --host 127.0.0.1 --verbose
```

## Contrainte structurante : un seul port CDC

Le RP2040 n'expose **qu'un seul port série**, partagé entre le REPL MicroPython
et le flux de données. Deux clients simultanés sur `/dev/vp_encoders` reçoivent
des octets tronqués, **sans erreur explicite**.

Conséquence : le bridge doit être arrêté avant tout travail sur le firmware.
`tools/deploy-firmware.sh` s'en charge automatiquement.

```bash
systemctl --user stop vp-bridge     # avant mpremote
systemctl --user start vp-bridge    # après
```

En SSH, ne jamais lancer `mpremote repl` sans `timeout` — c'est interactif et
la session se bloque. Utiliser `tools/read-serial.py` à la place.

## Arborescence

```
firmware/vp_encoders.py     MicroPython : 2× AS5600, multi-tour, homing, diagnostic
bridge/freed.py             Encodage Free-D D1 (29 octets) + conversion de repère
bridge/vp_bridge.py         Fusion tracker + encodeurs → UDP. 3 modes de source.
tools/find-device.sh        Identification USB, génération de la règle udev
tools/deploy-firmware.sh    Déploiement mpremote avec gestion du conflit de port
tools/test-sweep.py         Verdict automatique sur l'aimant et le montage
tools/read-serial.py        Lecture série bornée (se termine toujours)
system/                     Règle udev, service systemd
docs/HARDWARE.md            Câblage, aimants, montage mécanique
```

## Notes pour Claude Code

- Toujours vérifier `systemctl --user is-active vp-bridge` avant de toucher au port série.
- `bridge/freed.py` a un auto-test intégré : `python3 bridge/freed.py`.
- Le fichier `vp_cal.json` sur la carte est propre à un montage physique : jamais versionné.
- Le zéro de homing **n'est pas persistant**, et c'est délibéré. Un encodeur multi-tour
  ne sait pas dans quel tour il démarre ; restaurer un offset depuis la flash donnerait
  une valeur fausse dès que la roulette a bougé, carte éteinte.
- Les corrections d'axes se font dans `quat_to_pan_tilt_roll()`, **jamais** côté Unreal :
  des corrections aux deux bouts se cumulent et deviennent impossibles à raisonner.
