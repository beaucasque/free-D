#!/usr/bin/env python3
"""
vp_bridge.py — Emet du Free-D vers Unreal a partir du tracker et des encodeurs.

TROIS MODES, a valider dans cet ordre :

  --source simulate
      N'a besoin d'aucun materiel. Genere un mouvement de camera synthetique
      et un zoom/focus qui balaient leur course. Sert a valider toute la
      chaine Unreal (plugin LiveLinkFreeD, CineCameraActor, LensFile) avant
      d'avoir le moindre capteur branche.

  --source serial
      Encodeurs reels, pose synthetique. Valide les AS5600 et le mapping
      zoom/focus dans Unreal, sans dependre de libsurvive.

  --source survive
      Tout reel. Tracker via pysurvive, encodeurs via serie.

Ce decoupage est deliberé : chaque etape n'introduit qu'une seule inconnue.
Quand ca casse, on sait ou.

EXEMPLES
    ./vp_bridge.py --source simulate --host 127.0.0.1
    ./vp_bridge.py --source serial --port /dev/vp_encoders
    ./vp_bridge.py --source survive --port /dev/vp_encoders --host 192.168.1.73
"""

import argparse
import math
import signal
import sys
import time

from freed import FreeDSender, decode_d1, encode_d1, survive_to_freed

running = True


def _stop(signum, frame):
    global running
    running = False


class EncoderReader:
    """Lit les trames 'E:F:<n> Z:<n> S:<xx>' du RP2040.

    Non bloquant : on garde toujours la derniere valeur connue. Si la carte
    se tait, on continue d'emettre l'ancienne valeur plutot que de figer tout
    le bridge — un zoom qui ne bouge plus est moins grave qu'un tracking mort.
    """

    def __init__(self, port, baud=115200):
        import serial
        self.ser = serial.Serial(port, baud, timeout=0)
        self.focus = 0
        self.zoom = 0
        self.status = "??"
        self.last_update = 0.0
        self._buf = b""

    def poll(self):
        try:
            data = self.ser.read(4096)
        except Exception:
            return
        if not data:
            return
        self._buf += data
        while b"\n" in self._buf:
            line, self._buf = self._buf.split(b"\n", 1)
            self._parse(line.decode("utf8", "replace").strip())

    def _parse(self, line):
        # Tout ce qui ne commence pas par E: est du bruit REPL, on jette.
        if not line.startswith("E:"):
            return
        try:
            for token in line[2:].split():
                key, _, value = token.partition(":")
                if key == "F":
                    self.focus = int(value)
                elif key == "Z":
                    self.zoom = int(value)
                elif key == "S":
                    self.status = value
            self.last_update = time.monotonic()
        except ValueError:
            pass

    def stale(self, timeout=1.0):
        return (time.monotonic() - self.last_update) > timeout

    def close(self):
        self.ser.close()


def synthetic_pose(t):
    """Camera qui decrit lentement un arc, a hauteur d'epaule.

    Amplitudes volontairement modestes : on veut verifier que ca bouge dans
    le bon sens, pas impressionner. Un mouvement ample masque les erreurs de
    signe d'axe, un mouvement lent les revele.
    """
    radius = 2.0
    angle = 0.15 * t
    x = radius * math.cos(angle)
    y = radius * math.sin(angle)
    z = 1.5 + 0.1 * math.sin(0.4 * t)

    # La camera regarde vers l'origine
    yaw = angle + math.pi
    pitch = -0.05 * math.sin(0.3 * t)

    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    qw = cy * cp
    qx = cy * sp * 0.0 + sy * sp
    qy = cy * sp
    qz = sy * cp
    return (x, y, z, qw, qx, qy, qz)


def synthetic_lens(t):
    """Zoom et focus qui balaient leur course a des vitesses differentes."""
    zoom = int(32768 * (0.5 + 0.5 * math.sin(0.25 * t)))
    focus = int(32768 * (0.5 + 0.5 * math.sin(0.17 * t + 1.0)))
    return zoom, focus


def map_encoder(raw, lo, hi):
    """Position accumulee -> valeur Free-D 0..65535 sur la course calibree."""
    if hi == lo:
        return 0
    v = (raw - lo) / (hi - lo)
    return int(max(0, min(65535, v * 65535)))


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--source", choices=("simulate", "serial", "survive"),
                   default="simulate")
    p.add_argument("--host", default="127.0.0.1",
                   help="adresse de la machine Unreal")
    p.add_argument("--udp-port", type=int, default=40000)
    p.add_argument("--port", default="/dev/vp_encoders",
                   help="port serie du RP2040")
    p.add_argument("--rate", type=float, default=60.0,
                   help="cadence d'emission en Hz (aligner sur la cadence video)")
    p.add_argument("--camera-id", type=int, default=1)
    p.add_argument("--tracker", default=None,
                   help="nom du tracker libsurvive (defaut : le premier trouve)")
    p.add_argument("--focus-range", nargs=2, type=int, metavar=("LO", "HI"),
                   default=(0, 4096))
    p.add_argument("--zoom-range", nargs=2, type=int, metavar=("LO", "HI"),
                   default=(0, 4096))
    p.add_argument("--verbose", "-v", action="store_true")
    args = p.parse_args()

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    sender = FreeDSender(args.host, args.udp_port)
    print("Free-D -> %s:%d a %.1f Hz (source: %s)"
          % (args.host, args.udp_port, args.rate, args.source))

    encoders = None
    if args.source in ("serial", "survive"):
        try:
            encoders = EncoderReader(args.port)
            print("Encodeurs : %s" % args.port)
        except Exception as e:
            print("Impossible d'ouvrir %s : %s" % (args.port, e), file=sys.stderr)
            print("Verifie le groupe dialout et la regle udev.", file=sys.stderr)
            return 1

    survive_ctx = None
    if args.source == "survive":
        try:
            import pysurvive
            survive_ctx = pysurvive.SimpleContext(sys.argv[:1])
            print("libsurvive initialise")
        except ImportError:
            print("pysurvive absent : pip install pysurvive", file=sys.stderr)
            return 1

    period = 1.0 / args.rate
    t0 = time.monotonic()
    next_tick = t0
    next_report = t0 + 1.0
    frames = 0
    last_pose = (0.0, 0.0, 1.5, 1.0, 0.0, 0.0, 0.0)

    while running:
        now = time.monotonic()
        if now < next_tick:
            time.sleep(min(period / 4, next_tick - now))
            continue
        next_tick += period
        if next_tick < now:              # on a pris du retard : on resynchronise
            next_tick = now + period

        t = now - t0

        if encoders:
            encoders.poll()

        # --- pose ---
        if args.source == "survive":
            updated = survive_ctx.NextUpdated()
            if updated is not None:
                if args.tracker is None or updated.Name().decode() == args.tracker:
                    pos = updated.Pose()[0].Pos
                    rot = updated.Pose()[0].Rot
                    last_pose = (pos[0], pos[1], pos[2],
                                 rot[0], rot[1], rot[2], rot[3])
            pose = last_pose
        else:
            pose = synthetic_pose(t)

        # --- objectif ---
        if encoders and not encoders.stale():
            zoom = map_encoder(encoders.zoom, *args.zoom_range)
            focus = map_encoder(encoders.focus, *args.focus_range)
        else:
            zoom, focus = synthetic_lens(t)

        packet = survive_to_freed(pose, zoom=zoom, focus=focus,
                                  camera_id=args.camera_id)
        sender.send(packet)
        frames += 1

        if args.verbose and now >= next_report:
            next_report = now + 1.0
            d = decode_d1(packet)
            extra = ""
            if encoders:
                extra = "  enc=%s%s" % (encoders.status,
                                        " [MUET]" if encoders.stale() else "")
            print("%5d pkt/s  pan=%7.2f tilt=%6.2f  x=%7.1f y=%7.1f z=%6.1f mm  "
                  "zoom=%5d focus=%5d%s"
                  % (frames, d["pan"], d["tilt"], d["x"], d["y"], d["z"],
                     d["zoom"], d["focus"], extra))
            frames = 0

    print("\nArret. %d paquets emis." % sender.sent)
    sender.close()
    if encoders:
        encoders.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
