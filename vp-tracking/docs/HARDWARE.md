# Matériel

## Nomenclature

| Élément | Qté | Note |
|---|---|---|
| Vive Tracker 3.0 | 1 | corps caméra, USB direct de préférence |
| Base station 2.0 | 2 | alimentées en permanence |
| AS5600 breakout | 2 | modules 23×23 mm à header 7 broches |
| Aimant diamétral 6×2,5 mm | 2 (+2 rechange) | **Radial Magnets 8996** chez Digi-Key.ca |
| RP2040-Zero (Waveshare) | 1 | USB-C, ~20 mm |
| Follow focus Tilta FF-T07 | 2 | un pour le zoom, un pour le focus |
| Hub USB 2.0 **alimenté** | 1 | en bout d'extension active |
| Extension USB active | 1 | vers le plateau |
| Bouton momentané | 1 | homing / butées |

## Le point critique : l'aimantation

L'AS5600 exige un aimant **diamétralement** magnétisé — pôles répartis sur le
diamètre. Un disque néodyme ordinaire est **axial** (pôles sur les faces
plates) et donne une lecture inexploitable : le champ perpendiculaire vu par
le capteur reste constant au lieu de tourner.

C'est de loin l'erreur d'achat la plus fréquente sur ce composant, et les
listings Amazon sont souvent contradictoires entre le titre et les puces.

**Test de la tranche**, à faire à réception : approche deux aimants **par le
côté**. Attraction ou répulsion franche bord à bord → diamétral. Réaction molle
sur la tranche mais violente face contre face → axial, inutilisable.

Le firmware confirme ensuite : `tools/test-sweep.py` rend un verdict explicite.

## Câblage

```
AS5600 FOCUS → i2c0          AS5600 ZOOM → i2c1
  VCC → 3V3                    VCC → 3V3
  GND → GND                    GND → GND
  SDA → GP0                    SDA → GP2
  SCL → GP1                    SCL → GP3
  DIR → GND                    DIR → GND
  OUT, GPO : NC                OUT, GPO : NC

Bouton → GP4 vers GND (pull-up interne)
```

**DIR ne doit jamais rester flottante.** Reliée à GND, le comptage croît dans
le sens horaire. Laissée en l'air, elle capte du bruit et le sens s'inverse
aléatoirement — symptôme déroutant et difficile à diagnostiquer.

L'adresse I²C de l'AS5600 est figée à **0x36** et n'est pas configurable :
d'où les deux bus matériels distincts, plutôt qu'un multiplexeur.

Pull-ups : presque toujours déjà peuplées sur ces breakouts (R1/R2 près de
SDA/SCL). Vérifier avant d'en ajouter — deux paires en parallèle donnent
~2k2, ce qui charge trop le bus.

## Montage mécanique

**Aimant** : collé à l'époxy dans un bouchon qui s'emmanche au centre de la
roulette du FF-T07 (le logement de manivelle). Excentricité à maintenir sous
**0,25 mm**, faute de quoi on récolte une erreur d'angle sinusoïdale sur le
tour — invisible à l'œil, mais visible comme une respiration de focale dans
le composite.

**Entrefer** : 0,5 à 1,5 mm. L'AGC lu par le firmware sert de règle graduée —
viser ~128, avec `ML` et `MH` tous deux inactifs.

**Support PCB** : bras rigide sur le rod 15 mm. Toute flexion sous la main de
l'assistant fait varier l'entrefer et dégrade la lecture. PETG plutôt que PLA :
le PLA ramollit vers 60 °C, et ici la géométrie *est* la calibration.

**Câblage I²C** : sous 30 cm, fils soudés et gainés, serre-câble à 3 cm de la
PCB pour que la traction porte sur le serre-câble et jamais sur les pastilles.
Pas de Dupont sur le rig — ils se déchaussent aux vibrations, et une liaison
qui coupe en plein plan, c'est le plan perdu.

**Boutons BOOT/RESET du RP2040-Zero** : exposés sur le dessus. Orienter la
carte vers l'intérieur de la cage ou coller un capot — un appui accidentel
met la carte en mode bootloader.

## Alimentation

Un Tracker 3.0 en USB fonctionne **et** recharge : ~500 mA. Trois périphériques
plus le répéteur de l'extension active dépassent ce qu'un hub passif peut
fournir en bout de 5 m de câble. Hub alimenté obligatoire — sinon,
déconnexions aléatoires quand les batteries sont basses, c'est-à-dire au pire
moment.
