#!/usr/bin/env python3
"""
vp_bridge.py — Emet du Free-D vers Unreal a partir de trois trackers Vive.

v3 : tout Vive. Les AS5600, le QT Py et le mux ont disparu, et avec eux le
piege du port CDC unique, la regle udev vp_encoders et ModemManager. Les
trois poses sortent du meme event loop libsurvive : position, zoom et focus
partagent la meme horloge, il n'y a plus deux chaines de latence a realigner.

  tracker CAMERA  -> pan/tilt/roll + X/Y/Z
  tracker FOCUS   -> twist autour de son axe calibre -> champ focus
  tracker ZOOM    -> idem -> champ zoom

SOUSTRACTION DU MOUVEMENT CAMERA
    Le zoom et le focus sont calcules sur q_rel = conj(q_camera) * q_objectif :
    tout mouvement commun aux deux trackers s'annule. Mais les trackers ne
    sont pas echantillonnes au meme instant, et confronter un q_camera perime
    a un q_objectif recent fabrique une rotation parasite proportionnelle a
    la vitesse angulaire de la camera.

    Le bridge maintient donc un historique camera et interpole (slerp) a
    l'horodatage exact de chaque echantillon objectif. Un echantillon
    objectif plus recent que tout l'historique camera est DIFFERE d'un tick
    au lieu d'etre traite avec une pose extrapolee — c'est la seule facon que
    la soustraction soit exacte plutot qu'approximative.

CONTROLE D'INTEGRITE
    La distance camera <-> objectif est une constante mecanique. Si elle
    derive, un support a glisse et l'axe calibre est faux. Affiche SLIP.

DEUX MODES
  --source simulate   aucun materiel, valide la chaine Unreal
  --source survive    trois trackers USB + bridge/axes.json

EXEMPLES
    ./vp_bridge.py --source simulate --verbose
    ./vp_bridge.py --source survive --host 127.0.0.1 --rate 60 --verbose
    ./vp_bridge.py --list-devices
"""

import argparse
import math
import os
import signal
import sys
import time

import lensaxis
import survive_clock
import worldframe
from freed import FreeDSender, decode_d1, survive_to_freed

running = True

DEFAULT_AXES = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "axes.json")
DEFAULT_WORLD = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "world.json")

PENDING_MAX_AGE = 0.20      # au-dela, l'echantillon objectif est perime


def _stop(signum, frame):
    global running
    running = False


# ------------------------------------------------------------------ libsurvive


def dev_names(obj):
    """Identifiants d'un objet, le plus stable en tete.

    Delegue a survive_clock.object_names : le numero de serie GRAVE
    (LHR-F3D3F946) passe avant le nom de code (T20). Ce dernier n'est qu'un
    rang d'enumeration — verifie le 31 aout 2026 : brancher un quatrieme
    appareil a decale tous les noms. axes.json doit porter la serie, sans
    quoi la calibration du zoom finirait appliquee au focus.
    """
    return survive_clock.object_names(obj)


class SurviveSource:
    """Trois trackers, une file, un vidage complet par tick.

    NextUpdated() n'est pas bloquant. L'appeler une seule fois par tick
    laissait la file libsurvive grossir sans limite : la latence croissait
    avec le nombre de peripheriques. Invisible avec un tracker, franche avec
    trois. On vide toujours, entierement.
    """

    def __init__(self, axes_path, world_path=None, cam_span=1.0, slip_mm=8.0):
        try:
            import pysurvive
        except ImportError:
            sys.exit("pysurvive absent : voir bridge/requirements.txt")

        if not os.path.exists(axes_path):
            sys.exit("axes.json introuvable (%s).\n"
                     "Lance tools/calib-axis.py --list, puis --set-camera "
                     "et --axis." % axes_path)
        cfg = lensaxis.load(axes_path)

        self.camera = cfg.get("camera")
        if not self.camera:
            sys.exit("Tracker camera non declare dans axes.json.")
        self.axes = cfg.get("axes", {})
        if not self.axes:
            sys.exit("Aucun axe calibre dans axes.json.")

        # Repere plateau. Applique au tracker CAMERA uniquement : le zoom et
        # le focus sont calcules en relatif camera, le repere monde ne les
        # touche pas.
        self.world = None
        if world_path and os.path.exists(world_path):
            self.world = worldframe.load(world_path)
            print("Repere plateau : %s" % world_path)
        else:
            print("Repere plateau : aucun — poses brutes libsurvive. "
                  "Lance tools/calib-world.py.")

        self.ctx = pysurvive.SimpleContext(sys.argv[:1])
        # Horodater au drain est faux : le retard de file n'est pas le meme
        # pour deux trackers draines dans la meme rafale, et c'est justement
        # cet ecart que le slerp doit annuler. On decouvre l'horloge de
        # libsurvive plutot que de la supposer.
        self.clock = survive_clock.SurviveClock()
        self.hist = lensaxis.CameraHistory(span=cam_span)
        self.last_pose = (0.0, 0.0, 1.5, 1.0, 0.0, 0.0, 0.0)
        self.cam_seen = 0.0
        self.dropped_stale = 0

        self.state = {}
        for name, cal in self.axes.items():
            self.state[name] = {
                "cal": cal,
                "acc": lensaxis.Accumulator(),
                "filt": lensaxis.OneEuro(),
                "watch": lensaxis.MountWatch(tolerance_mm=slip_mm),
                "inv_ref": lensaxis.q_conj(tuple(cal["ref"])),
                "pending": [],
                "value": 0,
                "seen": 0.0,
                "last_t": 0.0,
                "slip": False,
            }

        print("Camera : %s" % self.camera)
        for name, cal in self.axes.items():
            print("  %-6s %s  course %.0f deg%s"
                  % (name, cal["device"], cal["span_deg"],
                     "  [MULTI-TOUR]" if cal["span_deg"] >= 355.0 else ""))

    # -- lecture -----------------------------------------------------------

    def poll(self):
        now = time.monotonic()
        by_device = {c["device"]: n for n, c in self.axes.items()}

        # 1. Vider la file. La camera va dans l'historique, les objectifs
        #    dans leur file d'attente respective.
        while True:
            u = self.ctx.NextUpdated()
            if u is None:
                break
            p = u.Pose()[0]
            pos = (p.Pos[0], p.Pos[1], p.Pos[2])
            quat = (p.Rot[0], p.Rot[1], p.Rot[2], p.Rot[3])
            names = dev_names(u)

            raw = survive_clock.read_timecode(u)
            self.clock.feed(raw, now)
            if self.clock.state == "apprentissage" and self.clock.ready():
                if self.clock.solve():
                    print(self.clock.describe())
            t = self.clock.to_mono(raw, now)

            if self.camera in names:
                # L'historique garde la pose BRUTE : la soustraction du
                # mouvement camera pour le zoom et le focus doit se faire
                # dans le repere ou les deux trackers sont exprimes, pas
                # dans le repere plateau.
                self.hist.push(t, quat, pos)
                self.last_pose = (worldframe.apply(self.world, pos, quat)
                                  if self.world else pos + quat)
                self.cam_seen = now
                continue
            for n in names:
                name = by_device.get(n)
                if name is not None:
                    st = self.state[name]
                    st["pending"].append((t, quat, pos))
                    st["seen"] = now
                    break

        # 2. Resoudre ce qui est interpolable. Ce qui est plus recent que
        #    l'historique camera attend le tick suivant.
        for name, st in self.state.items():
            keep = []
            for t, quat, pos in st["pending"]:
                got = self.hist.at(t)
                if got is None:
                    keep.append((t, quat, pos))
                    continue
                q_cam, p_cam, kind = got
                if kind == "extrap":
                    if now - t < PENDING_MAX_AGE:
                        keep.append((t, quat, pos))
                    else:
                        self.dropped_stale += 1
                    continue
                if kind == "stale":
                    self.dropped_stale += 1
                    continue
                self._update(st, t, quat, pos, q_cam, p_cam)
            st["pending"] = keep

    def _update(self, st, t, q_lens, p_lens, q_cam, p_cam):
        gap = t - st["last_t"] if st["last_t"] else 0.0
        st["last_t"] = t

        q_rel = lensaxis.relative(q_cam, q_lens)
        dq = lensaxis.q_mul(st["inv_ref"], q_rel)
        theta = st["acc"].push(lensaxis.twist_angle(dq, st["cal"]["axis"]),
                               gap=gap)
        theta = st["filt"](theta, t)
        st["value"] = lensaxis.to_freed(theta,
                                        st["cal"]["lo"], st["cal"]["hi"],
                                        invert=st["cal"].get("invert", False))

        if st["watch"].push(lensaxis.relative_position(q_cam, p_cam, p_lens)):
            st["slip"] = True

    # -- sortie ------------------------------------------------------------

    def pose(self):
        return self.last_pose

    def lens(self):
        return (self.state.get("zoom", {}).get("value", 0),
                self.state.get("focus", {}).get("value", 0))

    def health(self, timeout=0.5):
        now = time.monotonic()
        bits = []
        if not self.cam_seen:
            bits.append("CAM:ABSENT")
        elif now - self.cam_seen > timeout:
            bits.append("CAM:MUET")
        for name, st in self.state.items():
            tag = name.upper()[:3]
            if not st["seen"]:
                bits.append("%s:ABSENT" % tag)
            elif now - st["seen"] > timeout:
                bits.append("%s:MUET" % tag)
            if st["acc"].dropout():
                bits.append("%s:DROPOUT" % tag)
            if st["slip"]:
                bits.append("%s:SLIP%+.0fmm" % (tag, st["watch"].drift * 1000))
        if self.dropped_stale:
            bits.append("skip=%d" % self.dropped_stale)
        rate = math.degrees(self.hist.rate())
        if self.clock.state == "monotonic":
            bits.append("HORLOGE:DRAIN")
        return ("%s  cam=%3.0f deg/s" % (" ".join(bits) if bits else "OK", rate))


def cmd_list_devices(duration):
    try:
        import pysurvive
    except ImportError:
        sys.exit("pysurvive absent.")
    ctx = pysurvive.SimpleContext(sys.argv[:1])
    seen = {}
    t_end = time.monotonic() + duration
    while time.monotonic() < t_end:
        u = ctx.NextUpdated()
        if u is None:
            time.sleep(0.01)
            continue
        n = dev_names(u)
        if n:
            seen[n[0]] = n
    if not seen:
        sys.exit("Aucun peripherique lighthouse. Verifie 81-vive.rules, le "
                 "hub alimente, et qu'aucun SteamVR ne tourne ailleurs.")
    for k, names in sorted(seen.items()):
        print("%-24s %s" % (k, ", ".join(names[1:]) or "-"))
    return 0


# -------------------------------------------------------------------- simulate


def synthetic_pose(t):
    radius = 2.0
    angle = 0.15 * t
    x = radius * math.cos(angle)
    y = radius * math.sin(angle)
    z = 1.5 + 0.1 * math.sin(0.4 * t)
    yaw = angle + math.pi
    pitch = -0.05 * math.sin(0.3 * t)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    return (x, y, z, cy * cp, sy * sp, cy * sp, sy * cp)


def synthetic_lens(t):
    zoom = int(65535 * (0.5 + 0.5 * math.sin(0.25 * t)))
    focus = int(65535 * (0.5 + 0.5 * math.sin(0.17 * t + 1.0)))
    return zoom, focus


# ------------------------------------------------------------------------ main


def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--source", choices=("simulate", "survive"),
                   default="simulate")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--udp-port", type=int, default=40000)
    p.add_argument("--axes", default=DEFAULT_AXES)
    p.add_argument("--world", default=DEFAULT_WORLD,
                   help="repere plateau produit par tools/calib-world.py")
    p.add_argument("--rate", type=float, default=60.0,
                   help="cadence d'emission (aligner sur la cadence video)")
    p.add_argument("--camera-id", type=int, default=1)
    p.add_argument("--slip-mm", type=float, default=8.0,
                   help="derive de montage toleree avant alerte SLIP")
    p.add_argument("--list-devices", action="store_true")
    p.add_argument("--verbose", "-v", action="store_true")
    args = p.parse_args()

    if args.list_devices:
        return cmd_list_devices(10.0)

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    sender = FreeDSender(args.host, args.udp_port)
    print("Free-D -> %s:%d a %.1f Hz (source: %s)"
          % (args.host, args.udp_port, args.rate, args.source))

    src = (SurviveSource(args.axes, world_path=args.world,
                         slip_mm=args.slip_mm)
           if args.source == "survive" else None)

    period = 1.0 / args.rate
    t0 = time.monotonic()
    next_tick = t0
    next_report = t0 + 1.0
    frames = 0

    while running:
        now = time.monotonic()
        if now < next_tick:
            time.sleep(min(period / 4, next_tick - now))
            continue
        next_tick += period
        if next_tick < now:
            next_tick = now + period

        if src:
            src.poll()
            pose = src.pose()
            zoom, focus = src.lens()
        else:
            t = now - t0
            pose = synthetic_pose(t)
            zoom, focus = synthetic_lens(t)

        sender.send(survive_to_freed(pose, zoom=zoom, focus=focus,
                                     camera_id=args.camera_id))
        frames += 1

        if args.verbose and now >= next_report:
            next_report = now + 1.0
            d = decode_d1(survive_to_freed(pose, zoom=zoom, focus=focus,
                                           camera_id=args.camera_id))
            extra = ("  %s" % src.health()) if src else ""
            print("%5d pkt/s  pan=%7.2f tilt=%6.2f  x=%7.1f y=%7.1f z=%6.1f mm  "
                  "zoom=%5d focus=%5d%s"
                  % (frames, d["pan"], d["tilt"], d["x"], d["y"], d["z"],
                     d["zoom"], d["focus"], extra))
            frames = 0

    print("\nArret. %d paquets emis." % sender.sent)
    sender.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
