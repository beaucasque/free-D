#!/usr/bin/env python3
"""
calib-axis.py — Calibration d'un axe d'objectif porte par un tracker Vive.

Remplace les mesures mecaniques du paragraphe 4 du handoff. Plus de rapport
roulette/pignon, plus de comptage de dents : on balaie la bague butee a butee
et l'axe reel se deduit des donnees.

USAGE

  1. Identifier les trackers (une fois, noter les identifiants) :

        ./calib-axis.py --list

  2. Declarer le tracker camera :

        ./calib-axis.py --set-camera LHR-XXXXXXXX

  3. Calibrer chaque axe. Camera IMMOBILE sur trepied, les deux base
     stations visibles :

        ./calib-axis.py --axis focus --device LHR-YYYYYYYY
        ./calib-axis.py --axis zoom  --device LHR-ZZZZZZZZ

Ecrit bridge/axes.json. La calibration est PERSISTANTE : q_rel ne depend ni
du repere monde ni de la calibration des lighthouses. Elle ne casse que si un
tracker est demonte ou glisse sur son support.

Meme pendant le balayage, q_camera est interpole (slerp) a l'horodatage de
chaque echantillon objectif : si la camera bouge un peu, l'axe reste juste.
L'outil verifie quand meme l'immobilite et le dit.

VERDICT

  planarity  rapport du 2e au 1er singulier de la SVD. Petit = la rotation
             est planaire, le support est rigide.
  rms        ecart RMS a l'axe. C'est le swing : flexion, jeu de roulement.
  span       course totale. SI ELLE RESTE SOUS 360 DEGRES, l'axe est absolu
             au demarrage — aucun homing, jamais. Meme gain que le
             paragraphe 4 du handoff, sans mesurer un seul engrenage.
"""

import argparse
import math
import os
import sys
import threading
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "bridge"))

import lensaxis  # noqa: E402

DEFAULT_CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              "..", "bridge", "axes.json")


def dev_names(obj):
    """Identifiants disponibles pour un objet libsurvive.

    Selon la version de pysurvive, l'objet expose Name() (nom de code type
    T20, WM0) et parfois Serial(). On recolte tout ce qui repond ; l'important
    est que ce soit STABLE au redemarrage. Trois trackers identiques sur un
    hub ne sont pas enumeres dans un ordre garanti : ne jamais se fier a la
    position dans la liste.
    """
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


def cmd_list(args):
    ctx = open_context()
    print("Ecoute %.0f s — bouge chaque tracker a tour de role pour "
          "l'identifier.\n" % args.duration)
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
        key = names[0]
        p = u.Pose()[0]
        pos = (p.Pos[0], p.Pos[1], p.Pos[2])
        moved = seen.get(key, (names, 0.0))[1]
        if key in last:
            moved += math.dist(pos, last[key])
        seen[key] = (names, moved)
        last[key] = pos

    if not seen:
        sys.exit("Aucun peripherique. Verifie 81-vive.rules, le hub alimente, "
                 "et qu'aucun SteamVR ne tourne ailleurs.")
    print("%-24s %-28s %s" % ("IDENTIFIANT", "ALIAS", "DEPLACEMENT"))
    for key, (names, moved) in sorted(seen.items()):
        print("%-24s %-28s %6.2f m"
              % (key, ", ".join(names[1:]) or "-", moved))
    print("\nUtilise la colonne IDENTIFIANT dans --set-camera et --device.")


def cmd_set_camera(args):
    cfg = lensaxis.load(args.config) if os.path.exists(args.config) else {}
    cfg["camera"] = args.set_camera
    cfg.setdefault("axes", {})
    lensaxis.save(args.config, cfg)
    print("Tracker camera : %s  ->  %s" % (args.set_camera, args.config))


def cmd_calibrate(args):
    cfg = lensaxis.load(args.config) if os.path.exists(args.config) else {}
    cam = cfg.get("camera")
    if not cam:
        sys.exit("Tracker camera non declare. Lance d'abord --set-camera.")
    if args.device == cam:
        sys.exit("Le tracker objectif ne peut pas etre le tracker camera.")

    ctx = open_context()
    hist = lensaxis.CameraHistory(span=2.0)
    pending = []
    samples = []
    cam_quats = []

    def drain():
        """Vide la file. Camera -> historique, objectif -> file d'attente."""
        now = time.monotonic()
        got_cam = got_lens = False
        while True:
            u = ctx.NextUpdated()
            if u is None:
                break
            names = dev_names(u)
            p = u.Pose()[0]
            quat = (p.Rot[0], p.Rot[1], p.Rot[2], p.Rot[3])
            if cam in names:
                hist.push(now, quat, (p.Pos[0], p.Pos[1], p.Pos[2]))
                cam_quats.append(quat)
                got_cam = True
            elif args.device in names:
                pending.append((now, quat))
                got_lens = True
        return got_cam, got_lens

    def resolve():
        """Ne retenir que les echantillons objectif interpolables."""
        keep = []
        for t, quat in pending:
            got = hist.at(t)
            if got is None or got[2] == "extrap":
                keep.append((t, quat))
                continue
            if got[2] == "stale":
                continue
            samples.append(lensaxis.relative(got[0], quat))
        pending[:] = keep

    print("Camera : %s" % cam)
    print("Axe    : %s sur %s\n" % (args.axis, args.device))

    print("Attente des deux trackers...")
    seen_cam = seen_lens = False
    t_end = time.monotonic() + 15.0
    while time.monotonic() < t_end and not (seen_cam and seen_lens):
        c, l = drain()
        seen_cam = seen_cam or c
        seen_lens = seen_lens or l
        time.sleep(0.02)
    if not (seen_cam and seen_lens):
        missing = ([] if seen_cam else [cam]) + ([] if seen_lens else [args.device])
        sys.exit("Pas de donnees pour : %s" % ", ".join(missing))
    print("Les deux repondent.\n")

    print("LA CAMERA NE DOIT PAS BOUGER pendant tout le balayage.")
    print("Amene la bague en butee BASSE, puis ENTREE.")
    input()

    pending.clear()
    samples.clear()
    cam_quats.clear()

    print("Balaie LENTEMENT jusqu'en butee haute, puis ENTREE.")
    print("Lentement : le solveur lighthouse a besoin de voir les "
          "photodiodes.\n")

    done = threading.Event()
    threading.Thread(target=lambda: (input(), done.set()), daemon=True).start()

    t0 = time.monotonic()
    next_dot = t0 + 1.0
    while not done.is_set():
        drain()
        resolve()
        now = time.monotonic()
        if now >= next_dot:
            next_dot = now + 1.0
            sys.stdout.write("\r%d echantillons, %.0f s   "
                             % (len(samples), now - t0))
            sys.stdout.flush()
        if now - t0 > args.max_duration:
            print("\nDuree maximale atteinte.")
            break
        time.sleep(0.005)
    print()

    if args.decimate > 1:
        samples = samples[::args.decimate]

    # Immobilite de la camera : ecart angulaire max sur tout le balayage.
    cam_move = 0.0
    if len(cam_quats) > 1:
        q0 = cam_quats[0]
        cam_move = max(
            math.degrees(math.sqrt(sum(
                c * c for c in lensaxis.q_log(lensaxis.q_mul(
                    lensaxis.q_conj(q0), q)))))
            for q in cam_quats[::max(1, len(cam_quats) // 400)])

    try:
        cal = lensaxis.fit_axis(samples)
    except ValueError as e:
        sys.exit("Calibration impossible : %s" % e)

    v, why = lensaxis.verdict(cal)
    print("\n--- %s ---" % args.axis)
    print("echantillons   : %d" % cal["samples"])
    print("axe            : [%.4f, %.4f, %.4f]" % tuple(cal["axis"]))
    print("course         : %.1f deg" % cal["span_deg"])
    print("planarity      : %.4f" % cal["planarity"])
    print("rms swing      : %.2f deg" % cal["rms_deg"])
    print("bouge camera   : %.2f deg" % cam_move)
    print("verdict        : %s — %s" % (v, why))

    if cam_move > 2.0:
        print("\n!! La camera a bouge de %.1f deg pendant le balayage."
              % cam_move)
        print("   L'interpolation slerp compense le decalage temporel, pas un")
        print("   deplacement franc : ca degrade l'axe. Trepied, et refais.")

    if cal["span_deg"] < 355.0:
        print("\n>> Course sous 360 deg : cet axe est ABSOLU au demarrage.")
        print("   Pas de homing, ni maintenant ni jamais.")
    else:
        print("\n>> Course de %.0f deg : multi-tour." % cal["span_deg"])
        print("   L'accumulateur deroule, mais un decrochage optique long")
        print("   peut couter un tour. Le bridge affichera DROPOUT.")
        print("   Un pignon plus grand ramenerait ca sous un tour.")

    if v == "REFAIRE" and not args.force:
        sys.exit("\nNon enregistre. Reprends le montage, ou --force.")

    cal["device"] = args.device
    cal["invert"] = args.invert
    cal["camera_motion_deg"] = cam_move
    cal["calibrated"] = time.strftime("%Y-%m-%d %H:%M:%S")
    cfg.setdefault("axes", {})[args.axis] = cal
    lensaxis.save(args.config, cfg)
    print("\nEnregistre dans %s" % args.config)


def selftest():
    """Valide la logique de decision de cet outil, sans materiel.

    lensaxis.py teste deja la math d'ajustement. Ce qui appartient en propre
    a calib-axis.py, c'est ce qu'il DECIDE : accepter ou refuser un montage,
    detecter une camera qui a bouge, annoncer un axe absolu ou multi-tour.
    Ce sont ces verdicts qui sont verifies ici.
    """
    import numpy as np
    rng = np.random.default_rng(5)

    def q_from(axis, ang):
        a = np.asarray(axis, float)
        a /= np.linalg.norm(a)
        s = math.sin(ang / 2.0)
        return lensaxis.q_norm((math.cos(ang / 2.0),
                                a[0] * s, a[1] * s, a[2] * s))

    def sweep(span_deg, swing_deg, cam_deg=0.0, n=400):
        """Fabrique un balayage : bague sur son axe, swing parasite, et une
        camera qui derive eventuellement pendant la mesure."""
        ax = np.array([0.31, -0.84, 0.44])
        ax /= np.linalg.norm(ax)
        mount = q_from([1.0, 0.2, -0.4], 0.7)
        out, cams = [], []
        for i in range(n):
            u = i / (n - 1)
            q_cam = q_from([0, 0, 1], math.radians(cam_deg) * u)
            cams.append(q_cam)
            swing = q_from(rng.normal(size=3),
                           math.radians(swing_deg) * rng.normal())
            q_lens = lensaxis.q_mul(q_cam, lensaxis.q_mul(
                mount, lensaxis.q_mul(q_from(ax, math.radians(span_deg) * u),
                                      swing)))
            # Le releve se fait toujours en relatif camera.
            out.append(lensaxis.relative(q_cam, q_lens))
        return out, cams

    def cam_motion(cams):
        q0 = cams[0]
        return max(math.degrees(math.sqrt(sum(
            c * c for c in lensaxis.q_log(
                lensaxis.q_mul(lensaxis.q_conj(q0), q)))))
            for q in cams[::max(1, len(cams) // 300)])

    print("%-26s %-9s %8s %9s %s"
          % ("CAS", "VERDICT", "COURSE", "PLANEITE", "DECISION"))
    print("-" * 74)
    results = {}
    for label, span, swing, cam in (
            ("montage rigide, 270 deg", 270.0, 0.2, 0.0),
            ("montage qui flotte", 270.0, 9.0, 0.0),
            ("multi-tour, 520 deg", 520.0, 0.2, 0.0),
            ("camera qui a bouge", 270.0, 0.2, 6.0)):
        samples, cams = sweep(span, swing, cam)
        cal = lensaxis.fit_axis(samples)
        v, _why = lensaxis.verdict(cal)
        move = cam_motion(cams)
        decision = ("ABSOLU" if cal["span_deg"] < 355.0 else "MULTI-TOUR")
        if v == "REFAIRE":
            decision = "REFUSE (sans --force)"
        if move > 2.0:
            decision += " + ALERTE CAMERA %.1f deg" % move
        print("%-26s %-9s %7.0f deg %9.4f %s"
              % (label, v, cal["span_deg"], cal["planarity"], decision))
        results[label] = (v, cal, move, decision)

    assert results["montage rigide, 270 deg"][0] == "OK"
    assert results["montage qui flotte"][0] == "REFAIRE"
    assert results["multi-tour, 520 deg"][1]["span_deg"] > 355.0
    assert results["camera qui a bouge"][2] > 2.0
    assert abs(results["montage rigide, 270 deg"][1]["span_deg"] - 270.0) < 3.0

    print("\nOK — verdicts de montage, absolu/multi-tour et alerte camera.")
    return 0


def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--list", action="store_true",
                   help="enumerer les peripheriques lighthouse")
    p.add_argument("--set-camera", metavar="ID",
                   help="declarer le tracker porte par la camera")
    p.add_argument("--axis", choices=("focus", "zoom"))
    p.add_argument("--device", metavar="ID",
                   help="tracker porte par la roulette")
    p.add_argument("--config", default=DEFAULT_CONFIG)
    p.add_argument("--duration", type=float, default=20.0,
                   help="duree d'ecoute pour --list")
    p.add_argument("--max-duration", type=float, default=180.0)
    p.add_argument("--decimate", type=int, default=1,
                   help="ne garder qu'un echantillon sur N")
    p.add_argument("--invert", action="store_true",
                   help="inverser le sens de la valeur Free-D")
    p.add_argument("--force", action="store_true",
                   help="enregistrer meme si le verdict est REFAIRE")
    p.add_argument("--selftest", action="store_true",
                   help="valider la logique de decision, sans materiel")
    args = p.parse_args()

    if args.selftest:
        return selftest()
    if args.list:
        return cmd_list(args)
    if args.set_camera:
        return cmd_set_camera(args)
    if args.axis and args.device:
        return cmd_calibrate(args)
    p.print_help()
    return 1


if __name__ == "__main__":
    sys.exit(main() or 0)
