# HANDOFF free-D v5 — tracking caméra virtuelle tout-Vive

**État :** code écrit et auto-testé hors matériel. Rien n'a encore tourné sur
les trackers réels. Toute la validation reste à faire.

**Remplace** v2, v3 et v4. Les §4, §8, §9 et §11.3 de v2 sont caducs.
v4 affirmait au §5 que tous les outils avaient un `--selftest` : c'était faux
pour trois d'entre eux. Corrigé ici, et les auto-tests manquants sont écrits.

---

## 1. Ce qui a changé depuis v2

Deux décisions structurantes, prises par Patrice.

**Tout Vive.** Les AS5600, le QT Py, le mux PCA9546 et les aimants
diamétraux sont abandonnés. Trois trackers en USB direct : un sur la cage
caméra, un sur chaque bague (zoom et focus). Le zoom et le focus se lisent
comme une rotation relative à la caméra.

Disparaissent avec eux : le firmware CircuitPython, le protocole série
`E:F:...`, la règle udev `99-vp-encoders.rules`, ModemManager, et le piège
du port CDC unique (§8 de v2) — plus de conflit `mpremote` / bridge.

**Plus de Windows.** Le repère du plateau est relevé au sol avec deux
contrôleurs. `lighthousedb.json` n'est plus dans la boucle. La machine
Windows ne sert plus qu'aux mises à jour de firmware des trackers.

---

## 2. Matériel et topologie

| | |
|---|---|
| Station | Ubuntu Studio 22.04.5, **RTX 4070 Super** (v2 disait 4080 — à confirmer) |
| Moteur | Unreal Engine 5.8, plugin LiveLinkFreeD, CineCameraActor + LensFile |
| Tracking | libsurvive + pysurvive, **pas de SteamVR, pas de casque** |
| Trackers | 3 × Vive Tracker 3.0, **USB direct** (pas de dongle, pas d'appairage) |
| Contrôleurs | 2, uniquement pour le relevé du plateau |
| Base stations | 2 × SteamVR 2.0, **plafond**, entre l'écran et la position caméra la plus reculée, angulées vers la médiane, regardant vers le bas |
| Hub | USB 2.0 **alimenté, 3 A minimum** (3 × ~500 mA + répéteurs des rallonges actives) |

Montage des trackers d'objectif : **un de chaque côté du bloc optique**. Avec
les base stations à gauche et à droite, monter les deux du même côté ferait
masquer une station par le corps caméra — chaque tracker n'en verrait plus
qu'une, et la redondance disparaîtrait là où les décrochages coûtent le plus
cher (perte de tour sur un axe multi-tour).

HTC recommande plus de 2 m de haut et 25–35° vers le bas ; champ 150° H,
110° V. Les stations 2.0 n'ont pas besoin de se voir.

---

## 3. Repère du plateau

```
        +Y
         ↑        ┌──────────── écran vert ────────────┐
         │        └──────────────────┬─────────────────┘
         │                        origine (0,0,0)
         │                           │
         └───────────────────────────┼──────────────→ +X
                                     │
                              ligne médiane
                                     │
                                 [caméra]
```

- **+X** normale de l'écran projetée au sol, dirigée vers la caméra — la
  ligne médiane du studio.
- **+Y** latéral, le long du bas de l'écran.
- **+Z** vertical.
- **Origine** milieu des deux coins bas de l'écran.

Le repère est **ancré sur l'écran, pas sur la caméra**. Si +X suivait la
ligne caméra→écran, la caméra serait centrée par construction et un trépied
de travers deviendrait invisible. Ici le déport latéral est mesuré et
rapporté.

---

## 4. Les deux calibrations

Elles sont **indépendantes** et ne se croisent jamais.

| | `bridge/world.json` | `bridge/axes.json` |
|---|---|---|
| Contenu | origine, orientation, sol | par axe : tracker, axe de rotation, course |
| Produit par | onglet Studio (2 contrôleurs) | onglet Objectifs (trackers) |
| Appliqué à | tracker **caméra** seul | **zoom** et **focus** seuls |
| Refaire si | une base station ou l'écran bouge | un tracker est démonté ou glisse |

### Ce qu'aucun des deux ne contient

**La soustraction du mouvement caméra.** C'est `conj(q_caméra) · q_objectif`,
du calcul pur. Le résultat est indépendant du repère monde : `world.json`
peut être refait, supprimé, changé, le focus ne bouge pas d'un compte.

**La géométrie des base stations.** libsurvive la résout seule dans
`~/.config/libsurvive/config.json`. La console la **lit** — pour orienter la
normale du sol vers le haut (les stations sont au plafond) et sortir le
diagnostic d'installation — mais ne l'écrit jamais.

---

## 5. Arborescence

```
free-D/
├── bridge/
│   ├── freed.py            encodage D1 (INCHANGÉ depuis v2)
│   ├── survive_clock.py    découverte de l'horloge libsurvive
│   ├── lensaxis.py         math d'axe : quaternions, SVD, swing-twist,
│   │                       slerp, accumulateur multi-tour, one-euro
│   ├── worldframe.py       plan de sol, repère plateau, lecture des LH
│   ├── vp_bridge.py        v3 : 3 trackers, world.json, axes.json
│   ├── axes.json           produit par la console
│   ├── world.json          produit par la console
│   └── requirements.txt    numpy + pysurvive (pyserial retiré)
├── tools/
│   ├── vp-console.py       ★ serveur web, 3 onglets
│   ├── calib-world.py      équivalent CLI de l'onglet Studio
│   ├── calib-axis.py       équivalent CLI de l'onglet Objectifs
│   └── test-decouple.py    équivalent CLI de l'onglet Test
└── system/
    └── vp-bridge.service   plus de --port série
```

`gui-decouple.py` est **supprimé** : `vp-console.py` le remplace intégralement.
Maintenir deux interfaces web pour la même mesure n'avait pas de sens.

### Auto-tests — la liste exacte

Tous finissent par une ligne `OK — ...` et sortent en code 0.
**Les lancer avant de toucher au matériel.**

| Commande | Ce qu'il valide | Durée |
|---|---|---|
| `python3 bridge/freed.py` | encodage D1, checksum | <1 s |
| `python3 bridge/lensaxis.py` | SVD d'axe, multi-tour, slerp, garde-montage | <1 s |
| `python3 bridge/worldframe.py` | plan de sol, repère, lecture LH | <1 s |
| `python3 bridge/survive_clock.py` | détection d'unité, replis | <1 s |
| `tools/calib-axis.py --selftest` | verdicts de montage, absolu/multi-tour, alerte caméra | <1 s |
| `tools/calib-world.py --selftest` | chaîne sol/écran/médiane | <1 s |
| `tools/test-decouple.py --selftest` | analyse par phase, référence au repos | <1 s |
| `tools/vp-console.py --selftest` | **machine à états du Hub**, les trois onglets enchaînés, gardes | ~30 s |

Le dernier est le seul qui exerce l'enchaînement complet : trois relevés →
résolution → balayage d'axe → phases de test, avec les refus attendus (trois
points confondus, caméra utilisée comme axe d'objectif). Les autres ne
valident que la math de leur module.

**Aucun de ces tests ne touche au matériel.** Ce qu'ils ne peuvent pas dire
est au §10.

---

## 6bis. L'horloge (nouveau en v4)

Horodater une pose à l'instant où Python la lit dans la file est faux. Entre
la résolution par libsurvive et le drain, il s'écoule un délai variable qui
**n'est pas le même pour deux trackers drainés dans la même rafale** — et cet
écart est exactement ce que le slerp de `CameraHistory` doit annuler. Le
mesurer avec `time.monotonic()` revient à mesurer une règle avec elle-même.

L'indice était dans le code depuis v1 : `u.Pose()[0]` prend l'élément `[0]`
d'un tuple. `survive_simple_object_get_latest_pose()` a pour signature
`SurvivePose f(obj, FLT *timecode)` — `Pose()[1]` est donc très probablement
l'horodatage propre de libsurvive.

Son **unité** est inconnue et dépend de la version : secondes flottantes,
tics 48 MHz, millisecondes. `bridge/survive_clock.py` ne la devine pas, il la
mesure :

1. couples (brut, monotonic au drain) collectés en continu ;
2. pente par **médiane de pentes appariées**, pas par moindres carrés — le
   bruit de drain n'est pas symétrique, il ne fait qu'ajouter du retard, et
   les moindres carrés se laisseraient tirer par la queue ;
3. si la pente tombe à 2 % près d'une unité connue, on prend la valeur
   exacte : une pente empirique légèrement fausse dérive avec le temps ;
4. offset calé sur le **percentile bas** des résidus — le drain ne peut
   qu'ajouter du retard, donc la vérité est au bord inférieur du nuage ;
5. si rien ne tient, repli sur `monotonic` **en le disant** : la console
   affiche l'état en jaune, le bridge ajoute `HORLOGE:DRAIN` à sa ligne de
   santé.

Effet en démo, où 3 ms d'écart capteur sont modélisés :

```
sans horloge  roulis  crête naïf 2.779°  aligné 0.726°   176 counts
avec horloge  roulis  crête naïf 0.726°  aligné 0.004°     1 count
```

**Ce que ça ne résout pas.** Si libsurvive horodate au moment où il a
*résolu* la pose plutôt qu'au moment du balayage des photodiodes, il reste un
biais que cette régression ne verra pas : elle compare deux horloges, elle ne
sait pas ce que l'horodatage prétend représenter. Seul un test physique le
révèle — deux trackers **rigidement solidaires**, secoués ensemble, dont la
rotation relative doit rester nulle. C'est l'onglet Test, phase roulis, avec
les deux trackers d'objectif boulonnés sur la même barre.

---

## 6. La console

```bash
systemctl --user stop vp-bridge      # libsurvive est exclusif
tools/vp-console.py --demo           # sans matériel
tools/vp-console.py                  # puis http://127.0.0.1:8410
```

Serveur HTTP de la bibliothèque standard, SSE, tracés au canvas. Aucune
dépendance GUI, rien depuis un CDN. Depuis le Mac :
`ssh -L 8410:localhost:8410 unreal`.

**Un seul processus peut parler aux trackers.** La console et le bridge
s'excluent. C'est la raison d'être du serveur unique : enchaîner les trois
étapes sans redémarrer.

### Onglet Studio

Trois relevés au sol : coin bas gauche de l'écran, coin bas droit, point sous
la caméra. Chacun 3 s, moyenné.

Trois points **non alignés** déterminent le plan entièrement. Deux ne
suffisent pas : ils imposent au plan de les contenir, il reste un degré de
liberté — le pivot autour de la droite qui les joint. Les deux contrôleurs
posés sur la médiane laisseraient donc libre le **roulis du sol**, c'est-à-dire
l'inclinaison de l'horizon virtuel.

La console propage le bruit mesuré de chaque pose et annonce l'incertitude
angulaire réelle de la normale. Bon triangle + 2 mm de gigue ≈ 0,03°. Au-delà
de 0,15°, l'horizon penche visiblement.

`--screen-mm` compare la largeur d'écran mesurée à la mesure au ruban : seul
contrôle d'échelle honnête du volume de tracking.

### Onglet Objectifs

Déclarer le tracker caméra, puis un balayage butée à butée par axe, caméra
immobile sur trépied. L'axe du pignon se déduit par SVD des log-maps ; aucune
mesure mécanique. `planarity` (2ᵉ / 1ᵉʳ singulier) juge la rigidité du
montage, `rms` mesure le swing.

Si la course relevée reste **sous 360°**, l'axe est absolu au démarrage :
aucun homing, jamais.

### Onglet Test

Bagues bloquées au ruban, caméra en mouvement. Deux chaînes tracées sur le
même graticule : « naïf » (dernier `q_caméra` connu) et « aligné » (slerp à
l'horodatage de l'échantillon objectif).

Phases : repos → panoramique → tilt → **roulis** → travelling → bagues →
retour. Le roulis est la seule qui juge vraiment : le pignon tourne autour
d'un axe parallèle à l'axe optique, donc colinéaire au roulis et orthogonal
au panoramique. Faire « repos » en premier, c'est elle qui donne la
référence.

Crête maintenue avec fenêtre de garde de 0,5 s après le démarrage d'une phase
— sinon la crête retient le geste de reprise en main de la caméra.

---

## 7. Décisions à ne pas rouvrir sans raison

**Encodeurs → trackers.** Décidé par Patrice. Les objections de v2 §9 restent
factuellement vraies (bruit non stationnaire, occlusion, latence) mais le
gain d'intégration l'emporte : une seule horloge, un seul bus, une seule
chaîne de latence.

**Trackers en USB filaire, pas sans fil.** Pas d'appairage (libsurvive ne
sait pas appairer), pas de batterie, pas de veille firmware, latence moindre.
Prévoir boucle de service et serre-câble sur les deux trackers d'objectif.

**Repère ancré sur l'écran, pas sur la caméra.** Voir §3.

**Filtrage one-euro sur θ, jamais sur le quaternion.** Filtrer un quaternion
composante par composante le dénormalise et introduit une erreur d'angle
dépendante de l'orientation.

**`full_scale = 65535`** pour zoom et focus, pas 24 bits. C'est ce que
`--source simulate` a validé côté Unreal. Le passage en millimètres réels est
le travail du LensFile, pas du bridge.

**Corrections d'axes dans le bridge, jamais dans Unreal.** Une seule place :
`quat_to_pan_tilt_roll()` dans `freed.py` et le transform `world.json`. Si le
point part à l'envers, `sweep_save&invert=1`, pas un `-1` dans un Blueprint.

---

## 8. Pièges

**Vider la file libsurvive entièrement à chaque tick.** `NextUpdated()` n'est
pas bloquant. L'appeler une seule fois par tour laissait la file grossir : la
latence dérivait, invisible avec un tracker, franche avec trois. Bug corrigé
de la v1, ne pas le réintroduire.

**Ne jamais lier un tracker par sa position dans l'énumération.** Trois
trackers identiques sur un hub ne remontent pas dans un ordre garanti au
redémarrage. `axes.json` stocke un identifiant ; le prendre dans la colonne
de gauche de la liste d'appareils.

**Ne pas extrapoler la pose caméra.** Un échantillon objectif plus récent que
tout l'historique caméra est **différé d'un tick** (purge à 200 ms).
L'accepter en extrapolant revient exactement au comportement naïf.

**L'historique caméra garde la pose BRUTE.** `world.json` s'applique à la
sortie, pas à l'historique : la soustraction pour le zoom et le focus doit se
faire dans le repère où les deux trackers sont exprimés.

**Contrôleurs posés à plat, même orientation aux trois relevés.** Le centre
suivi est quelques cm au-dessus du sol ; comme le décalage est identique
partout, il s'annule dans le plan et l'orientation. Il ne reste qu'un
scalaire, `--floor-offset-mm`, à mesurer une fois au réglet.

---

## 9. Séquence de validation

| # | Commande | Valide |
|---|---|---|
| 0 | `python3 bridge/freed.py`, `lensaxis.py`, `worldframe.py`, `survive_clock.py` | intégrité du clone |
| 1 | `tools/vp-console.py --selftest` puis `--demo` | machine à états, puis l'interface |
| 2 | `bridge/vp_bridge.py --source simulate --verbose` | plugin LiveLinkFreeD, sens des axes |
| 3 | `python3 bridge/survive_clock.py --probe` | **présence et unité de l'horodatage** |
| 3b | console → liste d'appareils | énumération, identifiants stables |
| 4 | onglet Studio | `world.json`, échelle au ruban, diagnostic LH |
| 5 | onglet Objectifs | `axes.json`, verdict de montage, course |
| 6 | onglet Test, phase roulis | découplage caméra/objectif |
| 7 | `bridge/vp_bridge.py --source survive --verbose` | chaîne complète |
| 8 | UE 5.8 + LensFile | rendu |

Ne pas sauter l'étape 2 : elle isole les problèmes Unreal des problèmes
tracking.

---

## 10. Questions ouvertes

**Ta version de pysurvive expose-t-elle un horodatage ?** L'auto-test de
`survive_clock.py` ne valide que sa logique sur données fabriquées : il ne
peut pas répondre à ça. Une sonde le fait, trackers branchés :

```bash
python3 bridge/survive_clock.py --probe
```

Elle affiche ce que `Pose()` renvoie réellement (type, longueur, contenu de
chaque élément), la liste des méthodes de l'objet, puis tente de résoudre
l'horloge sur douze secondes. Trois verdicts possibles :

- *horloge exploitable — <unité>* : rien à faire, bridge et console
  l'utilisent déjà.
- *champ présent mais inexploitable* : repli assumé sur l'instant de drain,
  affiché en jaune dans la console et `HORLOGE:DRAIN` dans le bridge.
- *aucun horodatage exploitable* : chercher dans la liste des méthodes
  affichée, ou appeler `survive_simple_object_get_latest_pose` en ctypes.

**C'est la première commande à lancer après la première mise sous tension**,
avant même la calibration : tout le reste en dépend.

**Sémantique de l'horodatage.** Résolution ou balayage ? Voir §6bis : le test
des deux trackers solidaires tranche.

**Identifiants pysurvive.** `dev_names()` essaie `Serial()`, `SerialNumber()`
puis `Name()`. On ne sait pas lequel répond ni s'il est stable au
redémarrage. À vérifier sur deux redémarrages consécutifs avant de figer
`axes.json`.

**Sens de rotation.** Inconnu tant que rien n'a tourné. Si le point part à
l'envers : bouton « Enregistrer inversé » dans l'onglet Objectifs.

**Course des bagues.** Inconnue. Si elle dépasse 360°, les axes deviennent
multi-tour et un décrochage optique long peut coûter un tour (le bridge
affiche `DROPOUT`). Un pignon plus grand ramènerait la course sous un tour et
supprimerait le problème.

**`--floor-offset-mm`.** À mesurer au réglet sur le contrôleur réel.

**RTX 4070 Super ou 4080** sur la station Ubuntu. v2 dit 4080, Patrice dit
4070 Super.

---

## 11. Ce qu'il ne faut pas faire

- Lancer la console et le bridge en même temps.
- Corriger un axe côté Unreal.
- Filtrer un quaternion.
- Relever le plateau avec deux points.
- Bouger une base station après calibration.
- Réintroduire une lecture unique de `NextUpdated()` par tick.
- Horodater au `time.monotonic()` du drain quand une horloge est disponible.
- Prendre pour argent comptant un chiffre issu de `--demo` ou d'un
  `--selftest` : ils ne mesurent que la cohérence du code avec les constantes
  que la démo injecte elle-même. Aucun ne dit quoi que ce soit du matériel.
- Écrire dans ce document qu'un outil a un `--selftest` sans l'avoir lancé.
