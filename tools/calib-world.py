#!/usr/bin/env python3
"""
calib-world.py — Repere du plateau releve au sol, sans SteamVR.

POURQUOI PAS DEUX POINTS

Un plan a trois degres de liberte : deux pour la normale, un pour la hauteur.
Chaque point pose sur le plan en retire un.

  2 points -> il reste UN degre de liberte : le plan peut pivoter autour de
              la droite qui les joint. Un controleur pres de la camera et un
              au pied de l'ecran sont tous deux sur la ligne mediane : ce qui
              reste indetermine est exactement le ROULIS du sol.

              Consequence : l'horizon du decor virtuel est incline d'un angle
              inconnu. Sur un travelling qui suit la mediane ca ne se voit pas
              — la camera reste a y = 0 — mais les verticales du decor ne sont
              plus verticales dans le composite.

  3 points non alignes -> le plan est entierement determine. Fin.

Donc tu as raison : sur un sol parfaitement plat, AUCUN BALAYAGE N'EST
NECESSAIRE. Il en faut juste trois au lieu de deux.

ET TU LES AS DEJA

Les trois points du releve sont les deux COINS BAS DE L'ECRAN et le point au
SOL SOUS LA CAMERA. Ils forment un grand triangle bien ouvert — chez toi,
environ 4 m de base et 4 m de hauteur. Aucune etape supplementaire : les
memes trois mesures donnent le plan du sol, l'origine, la largeur d'ecran et
la ligne mediane.

L'outil mesure la dispersion de chaque pose et en propage le bruit : il
t'annonce l'incertitude angulaire reelle de ton plan de sol. Avec 2 mm de
gigue et ce triangle, c'est de l'ordre de 0,03 degre. Trois points presque
alignes donneraient dix fois pire — d'ou le controle d'etalement.

REPERE PRODUIT
    +X  normale de l'ecran au sol, dirigee vers la camera : ta ligne mediane
    +Y  lateral, le long du bas de l'ecran
    +Z  vertical

    Ancre sur l'ECRAN, pas sur la camera. Si +X suivait la ligne
    camera->ecran, la camera serait centree par construction et un trepied de
    travers deviendrait invisible. Ici le decentrage est mesure.

CE QUE CA NE REMPLACE PAS
    La geometrie des base stations. libsurvive la resout seul des qu'un
    tracker voit les deux. L'import SteamVR n'etait qu'un solveur de
    meilleure qualite, jamais une obligation.

USAGE
    ./calib-world.py --selftest
    ./calib-world.py --list
    ./calib-world.py --device LHR-XXXX --screen-mm 4000
    ./calib-world.py --device LHR-XXXX --second-device LHR-YYYY
    ./calib-world.py --device LHR-XXXX --sweep      # diagnostic en plus
"""

import argparse
import math
import os
import sys
import threading
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "bridge"))

import worldframe  # noqa: E402

DEFAULT_OUT = os.path.join(HERE, "..", "bridge", "world.json")


def dev_names(obj):
    out = []
    for attr in ("Serial", "SerialNumber", "Name"):
        fn = getattr(obj, attr, None)
        if fn is None:
            continue
        try:
            v = fn()
        except Exception:
            continue
        if isinstance(v, bytes):
            v = v.decode("utf8", "replace")
        v = str(v).strip()
        if v and v not in out:
            out.append(v)
    return out


def open_context():
    try:
        import pysurvive
    except ImportError:
        sys.exit("pysurvive absent. Voir bridge/requirements.txt")
    return pysurvive.SimpleContext(sys.argv[:1])


def collect(ctx, devices, stop_event=None, duration=None, quiet=False):
    """Recolte les positions des peripheriques demandes.

    On vide toujours la file en entier : NextUpdated() n'est pas bloquant, et
    une seule lecture par tour la laisserait grossir.
    """
    got = {d: [] for d in devices}
    t0 = time.monotonic()
    nxt = t0 + 1.0
    while True:
        while True:
            u = ctx.NextUpdated()
            if u is None:
                break
            names = dev_names(u)
            p = u.Pose()[0]
            for d in devices:
                if d in names:
                    got[d].append((p.Pos[0], p.Pos[1], p.Pos[2]))
                    break
        now = time.monotonic()
        if not quiet and now >= nxt:
            nxt = now + 1.0
            sys.stdout.write("\r  %s   "
                             % "  ".join("%s %d" % (d[-6:], len(v))
                                         for d, v in got.items()))
            sys.stdout.flush()
        if stop_event is not None and stop_event.is_set():
            break
        if duration is not None and now - t0 > duration:
            break
        time.sleep(0.004)
    if not quiet:
        print()
    return got


def stats(samples, label):
    """Moyenne, erreur-type et gigue d'une pose immobile."""
    a = np.asarray(samples, float)
    if len(a) < 20:
        sys.exit("%s : seulement %d echantillons. L'appareil est-il vu des "
                 "base stations ?" % (label, len(a)))
    mean = a.mean(axis=0)
    sd = a.std(axis=0)
    sem = sd / math.sqrt(len(a))
    return mean, sem, float(np.linalg.norm(sd) * 1000.0)


def capture(ctx, dev, label, duration=3.0):
    got = collect(ctx, [dev], duration=duration, quiet=True)[dev]
    mean, sem, jitter = stats(got, label)
    print("   %-14s %d echantillons, gigue %.1f mm" % (label, len(got), jitter))
    return mean, sem


def cmd_list(args):
    ctx = open_context()
    print("Ecoute %.0f s — bouge chaque appareil a tour de role.\n"
          % args.duration)
    seen, last = {}, {}
    t_end = time.monotonic() + args.duration
    while time.monotonic() < t_end:
        u = ctx.NextUpdated()
        if u is None:
            time.sleep(0.01)
            continue
        names = dev_names(u)
        if not names:
            continue
        k = names[0]
        p = u.Pose()[0]
        pos = (p.Pos[0], p.Pos[1], p.Pos[2])
        d = seen.get(k, 0.0) + (math.dist(pos, last[k]) if k in last else 0.0)
        seen[k], last[k] = d, pos
    if not seen:
        sys.exit("Aucun peripherique. Reveille les controleurs (bouton "
                 "systeme) et verifie 81-vive.rules.")
    print("%-24s %s" % ("IDENTIFIANT", "DEPLACEMENT"))
    for k, d in sorted(seen.items()):
        print("%-24s %6.2f m" % (k, d))


def cmd_calibrate(args):
    ctx = open_context()
    dev, dev2 = args.device, args.second_device

    print("Appareil de releve : %s" % dev)
    if dev2:
        print("Second appareil    : %s" % dev2)
    print("\nLes appareils se posent A PLAT au sol, dans la MEME "
          "orientation.")
    print("Leur centre suivi est quelques centimetres au-dessus du sol : ce")
    print("decalage est identique aux trois poses, donc il s'annule dans le")
    print("plan et dans l'orientation. Il ne reste qu'un scalaire, "
          "--floor-offset-mm.\n")

    # -- 1. les deux coins bas de l'ecran -------------------------------
    print("=" * 70)
    if dev2:
        print("1. ECRAN — LES DEUX COINS BAS")
        input("   Pose un appareil dans chaque coin bas du fond vert, "
              "puis ENTREE.")
        got = collect(ctx, [dev, dev2], duration=3.0, quiet=True)
        p_l, s_l, j_l = stats(got[dev], "coin gauche")
        p_r, s_r, j_r = stats(got[dev2], "coin droit")
        print("   coins releves, gigue %.1f et %.1f mm" % (j_l, j_r))
    else:
        print("1. ECRAN — COIN BAS GAUCHE")
        input("   Pose l'appareil dans le coin bas gauche, puis ENTREE.")
        p_l, s_l = capture(ctx, dev, "coin gauche")
        print("\n   ECRAN — COIN BAS DROIT")
        input("   Deplace-le dans le coin bas droit, puis ENTREE.")
        p_r, s_r = capture(ctx, dev, "coin droit")

    # -- 2. le sol sous la camera ---------------------------------------
    print("\n2. CAMERA")
    print("   Pose l'appareil AU SOL SOUS LA CAMERA, au centre du trepied.")
    input("   ENTREE quand il est en place.")
    p_c, s_c = capture(ctx, dev, "sous camera")

    # -- 3. le plan du sol, a partir de ces trois points -----------------
    pts = [p_l, p_r, p_c]
    sems = [s_l, s_r, s_c]
    normal, centroid, rms = worldframe.fit_plane(pts)
    big, small = worldframe.conditioning(pts)
    unc = worldframe.normal_uncertainty(pts, sems)

    print("\n3. PLAN DU SOL — deduit des trois points, sans balayage")
    print("   etalement du triangle : %.2f m x %.2f m" % (big, small))
    print("   incertitude de la normale : %.3f deg (95 %%)" % unc)
    if small < 0.5 * big:
        print("   !! Triangle plat : les trois points sont presque alignes.")
        print("      Le bruit sur le troisieme est amplifie. Ecarte-les.")
    if unc > 0.15:
        print("   !! Au-dela de 0,15 deg, l'horizon du decor virtuel penche")
        print("      de facon visible. Refais le releve, ou ecarte les points.")

    # -- 4. orientation de la normale -----------------------------------
    lh = worldframe.read_lighthouses(args.lighthouse_config)
    if lh:
        # Les base stations sont au plafond : elles disent ou est le haut,
        # sans qu'on ait a lever quoi que ce soit.
        above = np.mean([v for v in lh.values()], axis=0)
        normal = worldframe.orient_normal(normal, centroid, above)
        print("   normale orientee par les base stations (%d vues)" % len(lh))
    else:
        print("\n   Config libsurvive introuvable : leve l'appareil a hauteur")
        input("   de poitrine et tiens-le immobile, puis ENTREE.")
        high = collect(ctx, [dev], duration=1.5, quiet=True)[dev]
        if not high:
            sys.exit("Rien recu. L'appareil s'est-il endormi ?")
        normal = worldframe.orient_normal(normal, centroid,
                                          np.mean(high, axis=0))

    # -- 5. le repere ---------------------------------------------------
    try:
        frame = worldframe.build(normal, p_l, p_r, p_c,
                                 floor_offset_mm=args.floor_offset_mm)
    except ValueError as e:
        sys.exit("Repere impossible : %s" % e)

    print("\n4. REPERE")
    print("   largeur d'ecran : %.0f mm" % frame["screen_width_mm"])
    print("   camera a        : %.0f mm de l'ecran"
          % frame["camera_distance_mm"])
    print("   deport lateral  : %+.0f mm" % frame["camera_lateral_mm"])
    if abs(frame["camera_lateral_mm"]) > 30.0:
        print("      La camera n'est pas sur la ligne mediane. Le repere")
        print("      reste juste — il est ancre sur l'ecran — mais ta")
        print("      symetrie n'y est pas : decale le trepied de %+.0f mm."
              % -frame["camera_lateral_mm"])

    if args.screen_mm:
        err = frame["screen_width_mm"] - args.screen_mm
        print("   mesure au ruban : %.0f mm  (ecart %+.0f mm, %+.2f %%)"
              % (args.screen_mm, err, 100.0 * err / args.screen_mm))
        if abs(err) > max(15.0, 0.01 * args.screen_mm):
            print("      !! Plus de 1 %% d'erreur d'echelle. Laisse libsurvive")
            print("         tourner plus longtemps avec un tracker qui voit")
            print("         les deux base stations.")

    # -- 6. balayage optionnel : distorsion du volume --------------------
    if args.sweep:
        print("\n5. BALAYAGE (diagnostic)")
        print("   Ton sol etant plat, tout ecart au plan mesure ici est une")
        print("   DISTORSION DU TRACKING, pas une irregularite du sol. C'est")
        print("   la seule facon de savoir ou ta couverture faiblit.")
        ev = threading.Event()
        input("   ENTREE pour demarrer.")
        threading.Thread(target=lambda: (input(), ev.set()),
                         daemon=True).start()
        print("   Fais glisser l'appareil partout au sol. ENTREE pour "
              "arreter.")
        floor = collect(ctx, [dev], stop_event=ev)[dev]
        if len(floor) > 200:
            f = worldframe.prepare(dict(frame))
            loc = np.array([worldframe.apply(f, p, (1, 0, 0, 0))[:3]
                            for p in floor])
            dev_mm = np.abs(loc[:, 2]) * 1000.0
            print("   %d points | ecart au plan : %.1f mm RMS, %.1f mm max"
                  % (len(floor), float(np.sqrt(np.mean(dev_mm ** 2))),
                     float(dev_mm.max())))
            # Ou ca se degrade : decoupage en bandes le long de la mediane.
            edges = np.linspace(loc[:, 0].min(), loc[:, 0].max(), 5)
            print("   par distance a l'ecran :")
            for i in range(4):
                m = (loc[:, 0] >= edges[i]) & (loc[:, 0] < edges[i + 1])
                if m.sum() > 20:
                    print("     %.1f a %.1f m : %.1f mm RMS"
                          % (edges[i], edges[i + 1],
                             float(np.sqrt(np.mean(dev_mm[m] ** 2)))))
            frame["sweep_rms_mm"] = float(np.sqrt(np.mean(dev_mm ** 2)))

    # -- 7. base stations ------------------------------------------------
    rep = worldframe.lighthouse_report(worldframe.prepare(dict(frame)), lh) \
        if len(lh) >= 2 else None
    if rep:
        print("\n6. BASE STATIONS, dans ton repere")
        for k, v in sorted(rep["local"].items()):
            print("   %-4s  x %+6.2f m (vers camera)  y %+6.2f m (lateral)  "
                  "z %+5.2f m" % (k, v[0], v[1], v[2]))
        print("   ecartement %.2f m | milieu a %.2f m de l'ecran"
              % (rep["separation_mm"] / 1000.0, rep["midpoint_x_mm"] / 1000.0))
        if rep["opposed"]:
            print("   de part et d'autre de la mediane, residu de symetrie "
                  "%.0f mm" % rep["symmetry_mm"])
        else:
            print("   !! les deux sont du MEME cote de la mediane.")
        if min(rep["height_mm"]) < 2000:
            print("   !! %d mm de haut : HTC recommande plus de 2 m."
                  % min(rep["height_mm"]))

    frame["floor_rms_mm"] = rms
    frame["floor_points"] = [[float(x) for x in p] for p in pts]
    frame["normal_uncertainty_deg"] = unc
    frame["triangle_m"] = [big, small]
    if args.screen_mm:
        frame["tape_mm"] = args.screen_mm
    if rep:
        frame["lighthouses"] = rep
    frame["calibrated"] = time.strftime("%Y-%m-%d %H:%M:%S")
    worldframe.save(args.out, frame)

    print("\n" + "=" * 70)
    print("Enregistre dans %s" % args.out)
    print("Applique au tracker CAMERA uniquement : zoom et focus sont")
    print("calcules en relatif camera, le repere monde ne les touche pas.")
    print("A refaire si une base station bouge.")


def selftest():
    """Rejoue la chaine sur un plateau fabrique, sans materiel."""
    rng = np.random.default_rng(11)

    def rot(axis, ang):
        a = np.asarray(axis, float)
        a /= np.linalg.norm(a)
        k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
        return np.eye(3) + math.sin(ang) * k + (1 - math.cos(ang)) * (k @ k)

    r = rot([0.4, 0.8, 0.0], math.radians(5.0)) @ rot([0, 0, 1],
                                                      math.radians(-25.0))
    o = np.array([-0.9, 3.1, 1.2])
    OFF = 31.0
    up = np.array([0.0, 0.0, OFF / 1000.0])

    def to_survive(p):
        return o + r @ np.asarray(p, float)

    def pose(target, n=750, sigma=0.002):
        a = np.array([to_survive(np.asarray(target) + up)
                      + rng.normal(scale=sigma, size=3) for _ in range(n)])
        return a.mean(axis=0), a.std(axis=0) / math.sqrt(n)

    # Ecran de 4 m, camera a 4,2 m, decentree de 60 mm exprès.
    p_l, s_l = pose([0.0, -2.0, 0.0])
    p_r, s_r = pose([0.0, +2.0, 0.0])
    p_c, s_c = pose([4.2, 0.060, 0.0])

    pts, sems = [p_l, p_r, p_c], [s_l, s_r, s_c]
    n, c, _rms = worldframe.fit_plane(pts)
    big, small = worldframe.conditioning(pts)
    unc = worldframe.normal_uncertainty(pts, sems)
    lh = {"LH0": to_survive([2.1, -2.6, 2.45]),
          "LH1": to_survive([2.1, +2.6, 2.45])}
    n = worldframe.orient_normal(n, c, np.mean(list(lh.values()), axis=0))

    f = worldframe.prepare(worldframe.build(n, p_l, p_r, p_c,
                                            floor_offset_mm=OFF))
    print("triangle     : %.2f x %.2f m" % (big, small))
    print("normale      : incertitude %.3f deg (95 %%), orientee par les "
          "base stations" % unc)
    print("ecran        : %.0f mm (attendu 4000)" % f["screen_width_mm"])
    print("camera       : %.0f mm de l'ecran, deport %+.0f mm (attendu +60)"
          % (f["camera_distance_mm"], f["camera_lateral_mm"]))

    worst = 0.0
    for t in ([0, 0, 0], [4.2, 0, 0], [0, 2, 0], [3.0, -1.5, 1.7]):
        got = worldframe.apply(f, to_survive(t), (1, 0, 0, 0))[:3]
        worst = max(worst, float(np.linalg.norm(np.array(got) - t)))
    print("controle     : pire ecart %.1f mm sur 4 points" % (worst * 1000))

    rep = worldframe.lighthouse_report(f, lh)
    print("base stations: ecartement %.2f m, hauteurs %s mm, de part et "
          "d'autre %s, symetrie %.0f mm"
          % (rep["separation_mm"] / 1000.0, rep["height_mm"], rep["opposed"],
             rep["symmetry_mm"]))

    # Deux points sur la mediane : le plan doit etre refuse.
    try:
        worldframe.fit_plane([p_c, (p_l + p_r) / 2.0])
    except ValueError as e:
        print("garde        : deux points -> refuse (%s)" % e)
    else:
        raise AssertionError("aurait du refuser")

    assert worst < 0.010 and unc < 0.1
    assert abs(f["screen_width_mm"] - 4000) < 15
    assert abs(f["camera_lateral_mm"] - 60) < 15
    assert rep["opposed"] and rep["symmetry_mm"] < 20
    print("OK — trois points suffisent : sol, ecran, mediane, base stations.")
    return 0


def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--list", action="store_true")
    p.add_argument("--device", metavar="ID",
                   help="controleur ou tracker de releve")
    p.add_argument("--second-device", metavar="ID",
                   help="second appareil, pour les deux coins d'ecran en une "
                        "fois")
    p.add_argument("--out", default=DEFAULT_OUT)
    p.add_argument("--floor-offset-mm", type=float, default=0.0,
                   help="hauteur du centre suivi au-dessus du sol reel, "
                        "mesuree au reglet")
    p.add_argument("--screen-mm", type=float,
                   help="largeur d'ecran mesuree au ruban, pour verifier "
                        "l'echelle")
    p.add_argument("--sweep", action="store_true",
                   help="balayage du sol en plus : mesure la distorsion du "
                        "tracking dans le volume")
    p.add_argument("--lighthouse-config", default=None,
                   help="config.json de libsurvive (defaut : "
                        "~/.config/libsurvive/config.json)")
    p.add_argument("--duration", type=float, default=20.0)
    p.add_argument("--selftest", action="store_true")
    args = p.parse_args()

    if args.selftest:
        return selftest()
    if args.list:
        return cmd_list(args)
    if args.device:
        return cmd_calibrate(args)
    p.print_help()
    return 1


if __name__ == "__main__":
    sys.exit(main() or 0)
