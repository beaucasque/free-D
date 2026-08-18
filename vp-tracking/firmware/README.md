# Firmware

MicroPython sur RP2040. Voir l'en-tête de `vp_encoders.py` pour le protocole complet.

## Déploiement

```bash
../tools/deploy-firmware.sh
```

Ou manuellement :

```bash
mpremote connect /dev/vp_encoders cp vp_encoders.py :main.py
mpremote connect /dev/vp_encoders reset
```

## Commandes série

| Touche | Effet |
|---|---|
| `d` | Diagnostic : statut du champ, AGC, magnitude, balayage guidé |
| `h` | Homing — zéro des deux compteurs |
| `[` | Enregistre la butée basse |
| `]` | Enregistre la butée haute |
| `w` | Sauvegarde la calibration en flash |
| `p` | Pause / reprise de l'émission |
| `?` | État lisible |

**Bouton maintenu au boot** → entre directement en diagnostic.
**Appui court** → homing. **Appui long (>800 ms)** → butée basse.

## Séquence de calibration sur le rig

1. Les deux bagues en butée « infini / grand angle »
2. Appui court → homing
3. Appui long → butée basse
4. Les deux bagues en butée opposée
5. `]` → butée haute
6. `w` → sauvegarde

Vérifier ensuite avec `?` : `norm` doit balayer 0.000 → 1.000 sur la course.

## Passage en C

Prévu une fois le comportement figé. Le point qui le justifiera est la
régularité de la cadence d'émission : MicroPython a un jitter de quelques
millisecondes sur la boucle principale, acceptable en validation, moins en
production.
