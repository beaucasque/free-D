"""
worldframe.py — Repere du plateau a partir de points releves au sol.

CE QUE CA REMPLACE, ET CE QUE CA NE REMPLACE PAS

Il y a DEUX calibrations distinctes qu'on confond facilement :

  1. GEOMETRIE DES BASE STATIONS — ou est LH1 par rapport a LH0. libsurvive
     la resout tout seul a partir d'un tracker qui voit les deux. L'import
     SteamVR n'apporte qu'un solveur de meilleure qualite ; il n'est PAS
     necessaire.

  2. REPERE DU PLATEAU — ou est l'origine, ou est le sol, dans quelle
     direction regarde le decor. SteamVR ne donne ca que sous une forme
     arbitraire : le "centre" de sa room setup n'a aucun rapport avec ton
     fond vert.

Ce module traite le point 2, et il le fait MIEUX que l'import Windows, parce
que l'origine est mise la ou l'alignement compte : au pied du fond vert.
L'erreur de tracking croit avec la distance a l'origine — la placer sur le
fond vert, c'est mettre la precision maximale exactement la ou le reel et le
virtuel doivent se raccorder.

Le point 1 reste a la charge de libsurvive. Ce fichier ne le remplace pas.

POURQUOI UN BALAYAGE ET PAS DEUX POINTS

Deux points au sol donnent une origine et une direction : le lacet est fixe,
mais le TANGAGE et le ROULIS du plan de sol ne le sont pas. Un sol incline
d'un degre dans le repere libsurvive fait deriver la hauteur de 17 mm par
metre — la camera flotte ou s'enfonce quand elle traverse le plateau.

On balaie donc le sol avec le controleur et on ajuste un plan par SVD. Meme
math que fit_axis() dans lensaxis.py : le plus petit vecteur singulier du
nuage centre est la normale. Le residu RMS dit du meme coup si ton sol est
plat et si le tracking tient dans tout le volume.

HAUTEUR DU CONTROLEUR

Le centre suivi d'un controleur pose au sol est quelques centimetres
au-dessus du sol reel. Comme le plan de sol ET le point d'origine sont
releves AVEC LE MEME OBJET DANS LA MEME POSE, ce decalage est identique
partout et disparait : z = 0 tombe au centre du controleur. Il ne reste
qu'un scalaire a retrancher, mesure une fois au reglet (floor_offset_mm).

CONVENTION
    Quaternions (w, x, y, z), l'ordre de libsurvive.
"""

import json
import math
import os
import re
import shutil

import numpy as np


# ------------------------------------------------------------------ ajustement


def fit_plane(points, minimum=3):
    """Ajuste un plan a un nuage de points.

    TROIS points non alignes suffisent : un plan a trois degres de liberte
    (deux pour la normale, un pour la hauteur) et chaque point en retire un.

    DEUX points ne suffisent pas. Ils imposent au plan de les contenir, soit
    deux contraintes : il reste un degre de liberte, le pivot autour de la
    droite qui les joint. Si les deux points sont sur la ligne mediane, c'est
    exactement le roulis du sol qui reste indetermine.

    Retourne (normale, centroide, rms_mm). La normale n'est pas encore
    orientee : la SVD ne sait pas ou est le haut.
    """
    p = np.asarray(points, float)
    if len(p) < minimum:
        raise ValueError("%d points, il en faut au moins %d"
                         % (len(p), minimum))
    c = p.mean(axis=0)
    _u, _s, vt = np.linalg.svd(p - c, full_matrices=False)
    n = vt[-1] / np.linalg.norm(vt[-1])
    rms = float(np.sqrt(np.mean(((p - c) @ n) ** 2)) * 1000.0)
    return n, c, rms


def coverage(points):
    """Etendue du nuage dans son propre plan, en metres. Sert a verifier que
    le balayage couvre vraiment la zone de tournage et pas un coin."""
    p = np.asarray(points, float)
    c = p.mean(axis=0)
    _u, s, _vt = np.linalg.svd(p - c, full_matrices=False)
    k = math.sqrt(12.0 / max(1, len(p)))    # ecart-type -> etendue equivalente
    return float(s[0] * k), float(s[1] * k)


def conditioning(points):
    """Etalement du triangle de points, et sa dimension la plus faible.

    C'est elle qui fixe la precision de la normale : trois points presque
    alignes determinent le plan en theorie, mais le bruit sur la troisieme
    mesure y est amplifie par le rapport des deux etalements.
    """
    p = np.asarray(points, float)
    c = p.mean(axis=0)
    _u, s, _vt = np.linalg.svd(p - c, full_matrices=False)
    return float(s[0]), float(s[1])


def normal_uncertainty(means, sems, trials=400, seed=0):
    """Incertitude angulaire de la normale, par propagation du bruit mesure.

    means  positions moyennes de chaque point de pose
    sems   erreur-type de chaque moyenne (ecart-type / racine(n))

    Repond a la seule question qui compte : avec le bruit reel de TON
    installation et l'ecartement reel de TES points, le plan est-il assez
    bien determine ? Un chiffre mesure, pas une regle du pouce.
    """
    rng = np.random.default_rng(seed)
    m = np.asarray(means, float)
    e = np.asarray(sems, float).reshape(len(m), -1)
    n0, _c, _r = fit_plane(m)
    angs = []
    for _ in range(trials):
        pert = m + rng.normal(size=m.shape) * e
        try:
            n, _c, _r = fit_plane(pert)
        except ValueError:
            continue
        angs.append(math.degrees(math.acos(
            min(1.0, abs(float(np.dot(n, n0)))))))
    return float(np.percentile(angs, 95)) if angs else float("inf")


def orient_normal(n, p_low, p_high):
    """Oriente la normale vers le haut a partir d'un point releve en hauteur."""
    d = np.asarray(p_high, float) - np.asarray(p_low, float)
    return -n if float(np.dot(n, d)) < 0.0 else n


# ---------------------------------------------------------------------- repere


def mat_to_quat(r):
    """Matrice de rotation -> quaternion (w, x, y, z)."""
    t = r[0, 0] + r[1, 1] + r[2, 2]
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        q = (0.25 * s, (r[2, 1] - r[1, 2]) / s,
             (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s)
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        q = ((r[2, 1] - r[1, 2]) / s, 0.25 * s,
             (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s)
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        q = ((r[0, 2] - r[2, 0]) / s, (r[0, 1] + r[1, 0]) / s,
             0.25 * s, (r[1, 2] + r[2, 1]) / s)
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
        q = ((r[1, 0] - r[0, 1]) / s, (r[0, 2] + r[2, 0]) / s,
             (r[1, 2] + r[2, 1]) / s, 0.25 * s)
    n = math.sqrt(sum(c * c for c in q))
    return tuple(c / n for c in q)


def build(normal, p_left, p_right, p_camera, floor_offset_mm=0.0):
    """Construit le repere plateau a partir de l'ecran et de la camera.

    p_left, p_right  les deux coins BAS de l'ecran vert. Leur milieu devient
                     l'origine, leur ecart donne la direction laterale.
    p_camera         point au sol sous la camera. Sert a orienter +X et a
                     mesurer si la camera est vraiment centree.

    +X  normale de l'ecran dans le plan du sol, dirigee vers la camera.
        C'est la ligne qui coupe le studio et l'ecran en deux.
    +Y  lateral, le long du bas de l'ecran.
    +Z  vertical.

    LE REPERE EST ANCRE SUR L'ECRAN, PAS SUR LA CAMERA. C'est l'ecran que le
    decor virtuel doit epouser ; la camera, elle, bouge. Ancrer +X sur la
    ligne camera->ecran donnerait un axe qui depend de l'endroit ou le
    trepied est pose ce jour-la, et masquerait un decentrage au lieu de le
    reveler : la camera serait a Y = 0 par construction, toujours.

    Ici, le decentrage est mesure et rapporte (camera_lateral_mm).
    """
    n = np.asarray(normal, float)
    n /= np.linalg.norm(n)

    pl = np.asarray(p_left, float)
    pr = np.asarray(p_right, float)
    pc = np.asarray(p_camera, float)

    o = (pl + pr) / 2.0 - n * (floor_offset_mm / 1000.0)

    lat = pr - pl
    lat = lat - n * float(np.dot(lat, n))        # projete dans le plan du sol
    width = float(np.linalg.norm(lat))
    if width < 0.3:
        raise ValueError("les deux coins d'ecran sont a %.0f mm l'un de "
                         "l'autre : l'orientation du plateau serait dominee "
                         "par le bruit" % (width * 1000.0))
    ey = lat / width

    ex = np.cross(ey, n)                          # normale de l'ecran, au sol
    ex /= np.linalg.norm(ex)

    d_cam = pc - (pl + pr) / 2.0
    d_cam = d_cam - n * float(np.dot(d_cam, n))
    if float(np.dot(ex, d_cam)) < 0.0:            # +X pointe vers la camera
        ex = -ex
        ey = -ey

    r = np.column_stack([ex, ey, n])

    dist = float(np.dot(d_cam, ex))
    off = float(np.dot(d_cam, ey))
    if dist < 0.3:
        raise ValueError("la camera est a %.0f mm de l'ecran : trop pres pour "
                         "orienter +X de facon fiable" % (dist * 1000.0))

    return {
        "origin": [float(x) for x in o],
        "quat": list(mat_to_quat(r)),
        "screen_width_mm": width * 1000.0,
        "camera_distance_mm": dist * 1000.0,
        "camera_lateral_mm": off * 1000.0,
        "floor_offset_mm": floor_offset_mm,
    }


def _parse_survive_config(text):
    """Le config.json de libsurvive n'est PAS du JSON valide.

    survive_config.c ecrit le groupe racine avec write_config_group(f, cg,
    NULL) : le tag etant NULL, aucune accolade englobante n'est emise. Les
    groupes lighthouse qui suivent ont un tag, donc des accolades, mais rien
    n'insere de virgule entre le groupe racine et eux, ni entre deux
    lighthouses. Le fichier ressemble a ceci :

        "v":"0",
        "poser":"MPFIT",
        "disambiguator":"StateBased"
        "lighthouse0":{
        "index":"0",
        "pose":["2.100000000000","-2.600000000000",...]
        }

    Un json.load() dessus echoue TOUJOURS. C'est ce qui faisait renvoyer un
    dictionnaire vide meme quand les base stations etaient resolues — et la
    demo ne pouvait pas le montrer, puisqu'elle injecte les lighthouses
    directement sans passer par ce fichier.

    On restaure donc les virgules manquantes devant chaque ouverture de
    groupe, puis on englobe le tout.
    """
    out = []
    for ln in text.splitlines():
        ln = ln.strip()
        if not ln:
            continue
        if (re.match(r'^"[^"]+"\s*:\s*\{$', ln) and out
                and not out[-1].endswith((",", "{"))):
            out[-1] += ","
        out.append(ln)
    return json.loads("{" + "\n".join(out) + "}")


def read_lighthouses(path=None):
    """Poses des base stations, lues dans la config de libsurvive.

    libsurvive resout la geometrie des lighthouses tout seul et l'ecrit la.
    On n'en a pas besoin pour tracker — c'est un diagnostic d'installation.

    La position est dans le groupe "lighthouseN", champ "pose" : sept
    valeurs, position puis quaternion (config_set_lighthouse dans
    survive_config.c). Ce n'est pas une cle plate "lighthouseN_position", et
    json_helpers.c ecrit tous les nombres comme des CHAINES : la conversion
    est a notre charge.
    """
    if path is None:
        path = os.path.expanduser("~/.config/libsurvive/config.json")
    if not os.path.exists(path):
        return {}
    try:
        cfg = _parse_survive_config(open(path).read())
    except (ValueError, OSError):
        return {}

    out = {}
    for i in range(4):
        grp = cfg.get("lighthouse%d" % i)
        pos = None
        if isinstance(grp, dict) and grp.get("pose"):
            # OOTXSet dit que la calibration usine a ete recue ; PositionSet
            # dit que la GEOMETRIE est resolue. Les deux sont independants :
            # au demarrage libsurvive ecrit un groupe complet avec OOTXSet=1
            # et une pose encore toute a zero. La prendre pour argent
            # comptant placerait une base station a l'origine du plateau.
            if str(grp.get("PositionSet", "1")) not in ("1", "true", "True"):
                continue
            pos = list(grp["pose"])[:3]
        else:
            # Repli sur la forme plate, si une autre version l'ecrivait ainsi.
            flat = cfg.get("lighthouse%d_position" % i)
            if flat:
                pos = list(flat)[:3]
        if not pos or len(pos) < 3:
            continue
        try:
            v = [float(x) for x in pos]
        except (TypeError, ValueError):
            continue
        # Filet : une pose exactement nulle n'est pas une position, c'est un
        # emplacement pas encore rempli.
        if all(abs(x) < 1e-12 for x in v):
            continue
        out["LH%d" % i] = v
    return out


def lighthouse_report(frame, lighthouses):
    """Base stations exprimees dans le repere plateau.

    Repond a la question posee : la camera est-elle bien au milieu des deux ?
    Le residu de symetrie est la somme des deports lateraux — nulle si les
    deux sont a egale distance de la ligne mediane, de part et d'autre.
    """
    if len(lighthouses) < 2:
        return None
    loc = {k: apply(frame, v, (1.0, 0.0, 0.0, 0.0))[:3]
           for k, v in lighthouses.items()}
    ys = [p[1] for p in loc.values()]
    keys = sorted(loc)
    a, b = loc[keys[0]], loc[keys[1]]
    return {
        "local": {k: [round(c, 3) for c in v] for k, v in loc.items()},
        "symmetry_mm": abs(sum(ys)) * 1000.0,
        "separation_mm": float(np.linalg.norm(np.array(a) - np.array(b))
                               * 1000.0),
        "opposed": (a[1] * b[1]) < 0.0,
        "height_mm": [round(v[2] * 1000.0) for v in loc.values()],
        "midpoint_x_mm": (a[0] + b[0]) / 2.0 * 1000.0,
    }


def apply(frame, pos, quat):
    """Pose libsurvive -> pose plateau.

    A appliquer au tracker CAMERA uniquement. Le zoom et le focus sont
    calcules en relatif camera : ils sont insensibles au repere monde, et y
    toucher n'aurait aucun effet.
    """
    o = frame["_o"]
    rt = frame["_rt"]
    p = rt @ (np.asarray(pos, float) - o)
    qc = frame["_qc"]
    w1, x1, y1, z1 = qc
    w2, x2, y2, z2 = quat
    q = (w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
         w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
         w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
         w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2)
    return (float(p[0]), float(p[1]), float(p[2])) + q


def prepare(frame):
    """Precalcule ce qui sert a chaque paquet. A appeler une fois."""
    q = frame["quat"]
    frame["_o"] = np.asarray(frame["origin"], float)
    frame["_qc"] = (q[0], -q[1], -q[2], -q[3])
    w, x, y, z = q
    frame["_rt"] = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y)],
        [2 * (x * y - w * z), 1 - 2 * (x * x + z * z), 2 * (y * z + w * x)],
        [2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)],
    ])
    return frame


def load(path):
    return prepare(json.load(open(path)))


def save(path, data):
    out = {k: v for k, v in data.items() if not k.startswith("_")}
    with open(path, "w") as f:
        json.dump(out, f, indent=2)
        f.write("\n")


# ------------------------------------------------------------------ auto-test

if __name__ == "__main__":
    rng = np.random.default_rng(3)

    # Plateau fabrique : sol incline de 7 degres dans le repere libsurvive,
    # ecran tourne de 40 degres en lacet, origine loin de zero.
    def rot(axis, ang):
        a = np.asarray(axis, float)
        a /= np.linalg.norm(a)
        k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
        return np.eye(3) + math.sin(ang) * k + (1 - math.cos(ang)) * (k @ k)

    r_true = rot([0.3, 0.9, 0.0], math.radians(7.0)) @ \
        rot([0, 0, 1], math.radians(40.0))
    o_true = np.array([1.7, -2.4, 0.55])
    ez = r_true[:, 2]

    def to_survive(p):
        """Coordonnees plateau -> coordonnees libsurvive."""
        return o_true + r_true @ np.asarray(p, float)

    OFF = 31.0                       # centre suivi du controleur au sol
    up = np.array([0.0, 0.0, OFF / 1000.0])

    # 1. Balayage du sol : 6 m x 4 m, bruit 2 mm, sol pas parfaitement plat.
    floor = []
    for _ in range(900):
        u, v = rng.uniform(-1, 5), rng.uniform(-2, 2)
        bosse = 0.002 * math.sin(u * 1.1) * math.cos(v * 0.9)
        floor.append(to_survive([u, v, bosse + OFF / 1000.0])
                     + rng.normal(scale=0.002, size=3))

    n, c, rms = fit_plane(floor)
    n = orient_normal(n, c, to_survive([0, 0, 1.2]))
    ext = coverage(floor)
    print("[sol]      rms %.1f mm | etendue %.1f x %.1f m | normale a %.2f deg"
          % (rms, ext[0], ext[1],
             math.degrees(math.acos(min(1.0, abs(float(np.dot(n, ez))))))))

    # 2. Coins bas de l'ecran (largeur 4 m) et point au sol sous la camera,
    #    volontairement decentree de 60 mm pour verifier qu'on le detecte.
    pl = to_survive([0.0, -2.0, 0.0] + up) + rng.normal(scale=0.002, size=3)
    pr = to_survive([0.0, +2.0, 0.0] + up) + rng.normal(scale=0.002, size=3)
    pc = to_survive([4.2, 0.060, 0.0] + up) + rng.normal(scale=0.002, size=3)

    frame = prepare(build(n, pl, pr, pc, floor_offset_mm=OFF))
    print("[ecran]    largeur %.0f mm (attendu 4000)"
          % frame["screen_width_mm"])
    print("[camera]   distance %.0f mm (attendu 4200) | deport lateral "
          "%+.0f mm (attendu +60)"
          % (frame["camera_distance_mm"], frame["camera_lateral_mm"]))

    worst = 0.0
    for target in ([0, 0, 0], [4.2, 0, 0], [0, 2, 0], [1.5, 1.0, 1.8],
                   [3.0, -1.5, 0.4]):
        got = apply(frame, to_survive(target), (1.0, 0.0, 0.0, 0.0))[:3]
        worst = max(worst, float(np.linalg.norm(np.array(got) - target)))
    print("[controle] pire ecart %.1f mm sur 5 points connus" % (worst * 1000))

    q_set = mat_to_quat(r_true)
    q_out = apply(frame, to_survive([1, 0, 0]), q_set)[3:]
    ang = math.degrees(2 * math.atan2(
        math.sqrt(sum(c * c for c in q_out[1:])), abs(q_out[0])))
    print("[rotation] residu d'orientation %.3f deg" % ang)

    # 3. Base stations au plafond, de part et d'autre de la ligne mediane.
    lh = {"LH0": to_survive([2.1, -2.6, 2.45]),
          "LH1": to_survive([2.1, +2.6, 2.45])}
    rep = lighthouse_report(frame, lh)
    print("[LH]       ecart %.0f mm | hauteurs %s mm | de part et d'autre : "
          "%s | residu de symetrie %.0f mm"
          % (rep["separation_mm"], rep["height_mm"], rep["opposed"],
             rep["symmetry_mm"]))

    assert rms < 5.0, rms
    assert worst < 0.010, worst
    assert ang < 0.5, ang
    assert abs(frame["screen_width_mm"] - 4000.0) < 15.0
    assert abs(frame["camera_distance_mm"] - 4200.0) < 15.0
    assert abs(frame["camera_lateral_mm"] - 60.0) < 15.0
    assert rep["opposed"] and rep["symmetry_mm"] < 20.0
    try:
        build(n, pl, pl + np.array([0.05, 0, 0]), pc)
    except ValueError:
        print("[garde]    coins d'ecran trop proches : refuse")
    else:
        raise AssertionError("aurait du refuser")

    # --- lecture de la config libsurvive ------------------------------
    # On reconstruit ce que survive_config.c ECRIT REELLEMENT : groupe
    # racine sans accolades, groupes lighthouse entre accolades mais sans
    # virgule avant eux, tous les nombres en chaines ("%.12f" entre
    # guillemets, json_helpers.c).
    import tempfile

    def _grp(i, pos, quat=(1.0, 0.0, 0.0, 0.0)):
        arr = ",".join('"%.12f"' % v for v in list(pos) + list(quat))
        return ('"lighthouse%d":{\n"index":"%d",\n"id":"21546783%d",\n'
                '"mode":"0",\n"pose":[%s],\n"OOTXSet":"1",\n'
                '"PositionSet":"1"\n}\n' % (i, i, i, arr))

    fake = ('"v":"0",\n"poser":"MPFIT",\n"disambiguator":"StateBased"\n'
            + _grp(0, (2.1, -2.6, 2.45)) + _grp(1, (2.1, 2.6, 2.45)))

    tmpd = tempfile.mkdtemp(prefix="wf-selftest-")
    fp = os.path.join(tmpd, "config.json")
    with open(fp, "w") as fh:
        fh.write(fake)

    try:
        json.loads(fake)
        raise AssertionError("ce format serait donc du JSON valide ?")
    except ValueError:
        pass

    got = read_lighthouses(fp)
    assert set(got) == {"LH0", "LH1"}, got
    assert abs(got["LH0"][1] + 2.6) < 1e-9, got
    assert abs(got["LH1"][2] - 2.45) < 1e-9, got
    print("[config]   libsurvive : 2 base stations lues malgre un fichier "
          "sans accolades")

    # Cas rencontre a la premiere mise sous tension : OOTX recu mais
    # geometrie pas encore resolue. libsurvive ecrit alors un groupe complet
    # avec une pose toute a zero et PositionSet=0. Ne rien renvoyer.
    half = ('"v":"0",\n"poser":"MPFIT"\n'
            '"lighthouse0":{\n"index":"0",\n'
            '"pose":["0.000000000000","0.000000000000","0.000000000000",'
            '"0.000000000000","0.000000000000","0.000000000000",'
            '"0.000000000000"],\n"OOTXSet":"1",\n"PositionSet":"0"\n}\n')
    with open(fp, "w") as fh:
        fh.write(half)
    assert read_lighthouses(fp) == {}, "PositionSet=0 : rien attendu"
    print("[config]   OOTX recu mais geometrie non resolue : ignore")

    with open(fp, "w") as fh:
        fh.write('"v":"0",\n"poser":"MPFIT"\n')
    assert read_lighthouses(fp) == {}, "config nue : rien attendu"
    assert read_lighthouses(os.path.join(tmpd, "absent.json")) == {}
    print("[config]   config nue ou absente : {} sans exception")
    shutil.rmtree(tmpd, ignore_errors=True)

    print("OK — sol, ecran, ligne mediane, camera et base stations coherents.")
