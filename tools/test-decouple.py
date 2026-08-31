#!/usr/bin/env python3
"""
test-decouple.py — Le mouvement camera fuit-il dans le zoom et le focus ?

CE QUE CA MESURE

Bagues BLOQUEES, camera qui bouge. Le zoom et le focus ne doivent pas broncher.
Tout ce qui bouge quand meme est une fuite du tracker camera dans les axes
d'objectif : c'est le mode de panne propre au montage tout-Vive, et il ne se
voit qu'au compositing si on ne le mesure pas avant.

L'outil calcule DEUX chaines en parallele sur les memes echantillons :

    naif    q_camera = le dernier connu au moment ou l'echantillon objectif
            arrive. C'est ce que fait une implementation ordinaire.
    aligne  q_camera interpole (slerp) a l'horodatage exact de l'echantillon
            objectif. C'est ce que fait vp_bridge.py.

Si "aligne" n'est pas nettement meilleur que "naif", l'alignement temporel ne
sert a rien sur ta machine et il faut chercher ailleurs. S'il l'est, tu as la
preuve chiffree sur TON materiel, pas sur une simulation.

LE CHIFFRE QUI COMPTE

    skew = pente de |erreur d'angle| en fonction de la vitesse angulaire
           camera.

Cette pente a la dimension d'un TEMPS : c'est le decalage residuel entre les
deux trackers, en millisecondes. Un residu sous 1 ms est excellent, au-dela
de 5 ms il reste quelque chose a corriger.

MOUVEMENTS DEMANDES, ET POURQUOI

  repos       plancher de bruit. Sans lui, aucun autre chiffre n'a de sens.
  panoramique axe vertical, quasi orthogonal a l'axe du pignon : fuite faible
              attendue. C'est le cas facile.
  tilt        idem.
  ROULIS      axe optique, COLINEAIRE a l'axe du pignon. Fuite maximale.
              C'est le seul mouvement qui compte vraiment. Dutch, epaule,
              Steadicam, grue qui vrille.
  travelling  translation pure. Un quaternion ignore la translation : le
              resultat doit etre exactement zero. S'il ne l'est pas, la tete
              tourne aussi, ou un support a glisse.
  bagues      controle de vivacite : on bouge les bagues, camera immobile.
              Sans cette phase, un resultat parfait pourrait simplement
              vouloir dire que rien ne remonte.
  retour      retour au repos. theta doit revenir a sa valeur de depart :
              sinon un tour a ete perdu, ou le montage a glisse.

USAGE

    ./test-decouple.py                       # test complet guide
    ./test-decouple.py --phases roulis       # une seule phase
    ./test-decouple.py --record run.csv      # garde les brutes
    ./test-decouple.py --selftest            # valide l'outil, sans materiel

BLOQUER LES BAGUES : ruban gaffer sur la roulette de chaque unite encodeur.
Pas la main dessus — la main bouge.
"""

import argparse
import csv
import math
import os
import sys
import threading
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "bridge"))

import lensaxis  # noqa: E402

DEFAULT_CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              "..", "bridge", "axes.json")

PHASES = [
    ("repos", 20.0,
     "Ne touche a RIEN. Camera immobile, bagues bloquees."),
    ("panoramique", 20.0,
     "Panoramiques gauche-droite, du lent au franc. Pas de tilt."),
    ("tilt", 20.0,
     "Haut-bas, du lent au franc. Pas de pano."),
    ("roulis", 25.0,
     "ROULIS autour de l'axe optique (dutch). LA phase critique — "
     "va jusqu'a un mouvement franc."),
    ("travelling", 20.0,
     "Translation pure : avance, recule, lateral. Garde le cadre parallele."),
    ("bagues", 20.0,
     "Camera IMMOBILE. Debloque les bagues et balaie zoom puis focus."),
    ("retour", 15.0,
     "Rebloque les bagues a leur position de depart. Camera immobile."),
]


# ------------------------------------------------------------------ collecte


class Tap:
    """Lit libsurvive et calcule theta par les deux chaines, naive et alignee.

    Aucun filtrage : on veut le couplage brut. Le one-euro de vp_bridge.py
    masquerait justement ce qu'on cherche a mesurer.
    """

    def __init__(self, cfg):
        self.camera = cfg["camera"]
        self.axes = cfg["axes"]
        self.by_device = {c["device"]: n for n, c in self.axes.items()}
        self.hist = lensaxis.CameraHistory(span=2.0)
        self.pending = []
        self.rows = []

        self.st = {}
        for name, cal in self.axes.items():
            self.st[name] = {
                "cal": cal,
                "inv_ref": lensaxis.q_conj(tuple(cal["ref"])),
                "acc_n": lensaxis.Accumulator(),
                "acc_a": lensaxis.Accumulator(),
                "last_t": 0.0,
                "max_gap": 0.0,
            }
        self.n_exact = self.n_defer = self.n_stale = 0
        self.cam_gap = 0.0
        self._cam_last = 0.0

    def drain(self, ctx):
        now = time.monotonic()
        while True:
            u = ctx.NextUpdated()
            if u is None:
                break
            names = _dev_names(u)
            p = u.Pose()[0]
            pos = (p.Pos[0], p.Pos[1], p.Pos[2])
            quat = (p.Rot[0], p.Rot[1], p.Rot[2], p.Rot[3])
            if self.camera in names:
                if self._cam_last:
                    self.cam_gap = max(self.cam_gap, now - self._cam_last)
                self._cam_last = now
                self.hist.push(now, quat, pos)
                continue
            for n in names:
                name = self.by_device.get(n)
                if name is None:
                    continue
                st = self.st[name]
                if st["last_t"]:
                    st["max_gap"] = max(st["max_gap"], now - st["last_t"])
                st["last_t"] = now
                # q_camera "dernier connu" : fige tout de suite, c'est ce que
                # verrait une implementation naive.
                latest = self.hist.latest()
                self.pending.append((now, name, quat, pos,
                                     latest[0] if latest else None))
                break

    def resolve(self, phase):
        keep = []
        for t, name, quat, pos, q_naive in self.pending:
            got = self.hist.at(t)
            if got is None or got[2] == "extrap":
                if got is not None and time.monotonic() - t > 0.20:
                    self.n_stale += 1
                else:
                    self.n_defer += 1
                    keep.append((t, name, quat, pos, q_naive))
                continue
            if got[2] == "stale":
                self.n_stale += 1
                continue
            self.n_exact += 1
            q_cam, p_cam, _ = got
            st = self.st[name]
            axis = st["cal"]["axis"]

            th_a = st["acc_a"].push(lensaxis.twist_angle(
                lensaxis.q_mul(st["inv_ref"],
                               lensaxis.relative(q_cam, quat)), axis))
            th_n = th_a
            if q_naive is not None:
                th_n = st["acc_n"].push(lensaxis.twist_angle(
                    lensaxis.q_mul(st["inv_ref"],
                                   lensaxis.relative(q_naive, quat)), axis))

            d = lensaxis.relative_position(q_cam, p_cam, pos)
            self.rows.append({
                "phase": phase, "t": t, "axis": name,
                "theta_naif": th_n, "theta_aligne": th_a,
                "cam_rate": self.hist.rate(),
                "dist": math.sqrt(d[0] ** 2 + d[1] ** 2 + d[2] ** 2),
            })
        self.pending = keep


def _dev_names(obj):
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


# ------------------------------------------------------------------- analyse


def reference(rows, axis):
    """Valeur de theta au repos, bagues bloquees. C'est LA verite terrain :
    la bague n'ayant pas bouge de tout le test, tout ecart a cette valeur est
    une fuite, quelle que soit la phase.

    Prendre a la place le premier echantillon de chaque phase masquerait un
    biais constant — et un mouvement camera a vitesse constante produit
    justement une erreur constante.
    """
    r = [x for x in rows if x["phase"] == "repos" and x["axis"] == axis]
    if not r:
        return None
    return {"naif": float(np.degrees(np.mean([x["theta_naif"] for x in r]))),
            "aligne": float(np.degrees(np.mean([x["theta_aligne"] for x in r])))}


def analyse(rows, phase, axis, span_deg, ref=None):
    """Statistiques d'une phase pour un axe, par rapport au repos."""
    r = [x for x in rows if x["phase"] == phase and x["axis"] == axis]
    if len(r) < 10:
        return None

    rate = np.degrees([x["cam_rate"] for x in r])
    out = {"n": len(r), "cam_rate_max": float(rate.max()),
           "dist_drift_mm": float((max(x["dist"] for x in r)
                                   - min(x["dist"] for x in r)) * 1000.0)}

    for tag, key in (("naif", "theta_naif"), ("aligne", "theta_aligne")):
        th = np.degrees([x[key] for x in r])
        dev = th - (ref[tag] if ref else th[0])
        out[tag] = {
            "max": float(np.abs(dev).max()),
            "rms": float(np.sqrt(np.mean(dev ** 2))),
            "pct": float(np.abs(dev).max() / span_deg * 100.0),
            "counts": int(abs(dev).max() / span_deg * 65535),
        }
        # Pente |erreur| vs vitesse angulaire, forcee par l'origine.
        # Elle a la dimension d'un temps : c'est le decalage residuel.
        num = float(np.sum(rate * np.abs(dev)))
        den = float(np.sum(rate ** 2))
        out[tag]["skew_ms"] = (num / den * 1000.0) if den > 1e-9 else 0.0
    return out


def report(rows, cfg, tap=None):
    print("\n" + "=" * 74)
    print("RESULTATS — bagues bloquees, sauf phase 'bagues'")
    print("=" * 74)

    floor = {}
    worst = {}
    refs = {}
    for axis, cal in cfg["axes"].items():
        span = cal["span_deg"]
        ref = refs[axis] = reference(rows, axis)
        if ref is None:
            print("\n!! Phase 'repos' absente pour %s : les ecarts sont "
                  "mesures par rapport au debut de chaque phase, ce qui "
                  "masque un biais constant." % axis)
        print("\n### %s  (course %.0f deg, %.0f counts Free-D par degre)"
              % (axis, span, 65535 / span))
        print("%-12s %7s | %8s %8s | %8s %8s %7s %8s"
              % ("phase", "cam", "naif", "naif", "aligne", "aligne",
                 "aligne", "residu"))
        print("%-12s %7s | %8s %8s | %8s %8s %7s %8s"
              % ("", "deg/s", "max deg", "skew ms", "max deg", "% course",
                 "counts", "skew ms"))
        print("-" * 74)

        for name, _d, _h in PHASES:
            a = analyse(rows, name, axis, span, ref)
            if a is None:
                continue
            print("%-12s %7.0f | %8.3f %8.2f | %8.3f %8.2f %7d %8.2f"
                  % (name, a["cam_rate_max"],
                     a["naif"]["max"], a["naif"]["skew_ms"],
                     a["aligne"]["max"], a["aligne"]["pct"],
                     a["aligne"]["counts"], a["aligne"]["skew_ms"]))
            if name == "repos":
                floor[axis] = a
            elif name not in ("bagues", "retour"):
                if axis not in worst or a["aligne"]["pct"] > worst[axis][1]["aligne"]["pct"]:
                    worst[axis] = (name, a)

        drift = max((analyse(rows, n, axis, span, ref) or {})
                    .get("dist_drift_mm", 0.0) for n, _d, _h in PHASES)
        print("-" * 74)
        print("derive de la distance camera<->objectif : %.1f mm" % drift)

    if tap:
        tot = tap.n_exact + tap.n_defer + tap.n_stale
        print("\nEchantillons objectif : %d interpolables, %d differes, "
              "%d perdus (%.1f %% exploitables)"
              % (tap.n_exact, tap.n_defer, tap.n_stale,
                 100.0 * tap.n_exact / max(1, tot)))
        print("Trou max camera : %.0f ms" % (tap.cam_gap * 1000))
        for name, st in tap.st.items():
            print("Trou max %-6s : %.0f ms" % (name, st["max_gap"] * 1000))

    # ---- verdict -----------------------------------------------------
    print("\n" + "=" * 74)
    print("VERDICT")
    print("=" * 74)
    ok = True
    for axis in cfg["axes"]:
        if axis not in worst:
            print("%-6s : pas assez de donnees" % axis)
            ok = False
            continue
        phase, a = worst[axis]
        f = floor.get(axis)
        pct = a["aligne"]["pct"]
        gain = (a["naif"]["max"] / a["aligne"]["max"]
                if a["aligne"]["max"] > 1e-6 else float("inf"))

        if pct < 0.3:
            v = "OK"
        elif pct < 1.0:
            v = "PASSABLE"
        else:
            v = "PROBLEME"
            ok = False
        print("%-6s : %-9s pire phase '%s' — %.2f %% de la course "
              "(%d counts)" % (axis, v, phase, pct, a["aligne"]["counts"]))
        if f:
            print("         plancher de bruit au repos : %.2f %% "
                  "(%d counts)" % (f["aligne"]["pct"], f["aligne"]["counts"]))
            if pct < f["aligne"]["pct"] * 1.5:
                print("         la fuite est sous le bruit : rien a corriger.")
        print("         alignement temporel : %.1fx meilleur que le naif, "
              "residu %.2f ms" % (gain, a["aligne"]["skew_ms"]))
        if gain < 1.5:
            print("         !! l'alignement n'apporte presque rien — verifie "
                  "que les deux trackers remontent bien.")
        if a["aligne"]["skew_ms"] > 5.0:
            print("         !! residu > 5 ms : un tracker est mal vu des "
                  "base stations, ou le hub sature.")
        if a["dist_drift_mm"] > 8.0:
            print("         !! la distance camera<->objectif a derive de "
                  "%.0f mm : un support glisse." % a["dist_drift_mm"])

    a0 = list(cfg["axes"])[0]
    tr = analyse(rows, "travelling", a0,
                 cfg["axes"][a0]["span_deg"], refs.get(a0))
    if tr and tr["aligne"]["max"] > 0.3:
        print("\n!! Le travelling fait bouger l'angle de %.2f deg. Une "
              "translation pure ne peut pas : la tete tourne, ou un support "
              "glisse." % tr["aligne"]["max"])
        ok = False

    for axis, cal in cfg["axes"].items():
        liv = analyse(rows, "bagues", axis, cal["span_deg"], refs.get(axis))
        if liv and liv["aligne"]["max"] < cal["span_deg"] * 0.1:
            print("\n!! Phase 'bagues' : %s n'a bouge que de %.1f deg alors "
                  "que tu balayais. Le tracker ne remonte pas, ou l'axe "
                  "calibre est faux — un resultat 'OK' ailleurs ne voudrait "
                  "alors rien dire." % (axis, liv["aligne"]["max"]))
            ok = False

    ret = {a: analyse(rows, "retour", a, c["span_deg"], refs.get(a))
           for a, c in cfg["axes"].items()}
    for axis, a in ret.items():
        if a and a["aligne"]["max"] > 2.0:
            print("\n!! %s n'est pas revenu a sa valeur de depart (%.1f deg). "
                  "Tour perdu pendant le balayage, ou montage qui a bouge."
                  % (axis, a["aligne"]["max"]))
            ok = False

    print("\n%s" % ("Chaine validee — le mouvement camera est correctement "
                    "soustrait." if ok else
                    "A corriger avant de tourner."))
    return 0 if ok else 1


# ------------------------------------------------------------------- capture


def run(args, cfg):
    try:
        import pysurvive
    except ImportError:
        sys.exit("pysurvive absent. Voir bridge/requirements.txt")
    ctx = pysurvive.SimpleContext(sys.argv[:1])
    tap = Tap(cfg)

    wanted = [name for name, _d, _h in PHASES]
    if args.phases:
        wanted = [p for p in wanted if p in args.phases]

    print("Camera : %s" % cfg["camera"])
    for a, c in cfg["axes"].items():
        print("  %-6s %s  course %.0f deg" % (a, c["device"], c["span_deg"]))
    print("\nBAGUES BLOQUEES AU RUBAN avant de commencer (sauf phase "
          "'bagues').\n")

    for name, dur, howto in PHASES:
        if name not in wanted:
            continue
        print("-" * 74)
        print("PHASE '%s' — %.0f s" % (name, dur))
        print("  %s" % howto)
        print("  ENTREE pour demarrer, ENTREE pour arreter avant la fin.")
        input()

        done = threading.Event()
        threading.Thread(target=lambda: (input(), done.set()),
                         daemon=True).start()
        t0 = time.monotonic()
        nxt = t0 + 1.0
        while not done.is_set() and time.monotonic() - t0 < dur:
            tap.drain(ctx)
            tap.resolve(name)
            now = time.monotonic()
            if now >= nxt:
                nxt = now + 1.0
                sys.stdout.write("\r  %2.0f s restantes, %d echantillons   "
                                 % (dur - (now - t0), len(tap.rows)))
                sys.stdout.flush()
            time.sleep(0.004)
        print("\r  termine.%s" % (" " * 30))

    if args.record:
        with open(args.record, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(tap.rows[0].keys()))
            w.writeheader()
            w.writerows(tap.rows)
        print("\nBrutes : %s (%d lignes)" % (args.record, len(tap.rows)))

    return report(tap.rows, cfg, tap)


# ------------------------------------------------------------------ selftest


def selftest():
    """Valide l'analyse sur des donnees fabriquees, sans materiel.

    Bagues immobiles, camera qui roule a 200 deg/s, tracker objectif
    horodate 3 ms avant le dernier echantillon camera. La chaine naive doit
    voir une fuite, l'alignee non.
    """
    def q_from(axis, ang):
        a = np.array(axis, float)
        a /= np.linalg.norm(a)
        s = math.sin(ang / 2.0)
        return lensaxis.q_norm((math.cos(ang / 2.0),
                                a[0] * s, a[1] * s, a[2] * s))

    axis = [0.0, 0.0, 1.0]
    q_bague = q_from(axis, math.radians(90.0))
    cfg = {"camera": "CAM",
           "axes": {"focus": {"device": "FOC", "axis": axis,
                              "ref": list(q_bague), "lo": 0.0,
                              "hi": math.radians(300.0), "span_deg": 300.0}}}

    tap = Tap(cfg)
    st = tap.st["focus"]

    def cam_angle(t, moving):
        # Roulis oscillant : la vitesse doit VARIER, sinon l'erreur naive est
        # constante et la soustraction de reference l'effacerait.
        return math.radians(60.0) * math.sin(2.0 * math.pi * t) if moving else 0.0

    for phase, moving in (("repos", False), ("roulis", True)):
        hist = lensaxis.CameraHistory(span=2.0)
        tap.hist = hist
        st["acc_n"] = lensaxis.Accumulator()
        st["acc_a"] = lensaxis.Accumulator()
        for i in range(500):
            tc = i * 0.004
            ang = cam_angle(tc, moving)
            q_cam = q_from(axis, ang)
            q_naive = hist.latest()[0] if hist.t else None
            hist.push(tc, q_cam, (0.0, 0.0, 1.4))
            if i < 3:
                continue
            tl = tc - 0.003
            q_lens = lensaxis.q_mul(q_from(axis, cam_angle(tl, moving)),
                                    q_bague)
            tap.pending.append((tl, "focus", q_lens, (0.1, 0.0, 1.4), q_naive))
        tap.resolve(phase)

    return report(tap.rows, cfg, tap)


def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--config", default=DEFAULT_CONFIG)
    p.add_argument("--phases", nargs="+",
                   choices=[n for n, _d, _h in PHASES],
                   help="ne faire que ces phases")
    p.add_argument("--record", metavar="CSV",
                   help="ecrire les echantillons bruts")
    p.add_argument("--selftest", action="store_true",
                   help="valider l'outil sur des donnees fabriquees")
    args = p.parse_args()

    if args.selftest:
        return selftest()
    if not os.path.exists(args.config):
        sys.exit("axes.json introuvable (%s). Lance calib-axis.py d'abord."
                 % args.config)
    return run(args, lensaxis.load(args.config))


if __name__ == "__main__":
    sys.exit(main())
