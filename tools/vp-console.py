#!/usr/bin/env python3
"""
vp-console.py — Console unique du pipeline Free-D. Trois onglets.

    STUDIO      releve du repere plateau     -> bridge/world.json
    OBJECTIFS   calibration des axes         -> bridge/axes.json
    TEST        decouplage camera/objectif   -> verdict

POURQUOI UN SEUL PROCESSUS

libsurvive ouvre les peripheriques USB en exclusif : deux outils ne peuvent
pas parler aux trackers en meme temps. Tant que la console tourne, ARRETER
LE BRIDGE (systemctl --user stop vp-bridge), et inversement. Un serveur
unique qui possede le contexte et le partage entre les trois onglets est la
seule facon d'enchainer les trois etapes sans redemarrer quoi que ce soit.

CE QUE CHAQUE FICHIER CONTIENT, ET CE QU'IL NE CONTIENT PAS

    world.json   origine, sol, ligne mediane. Applique au tracker CAMERA
                 uniquement.
    axes.json    pour chaque axe : quel tracker, autour de quel axe lire la
                 rotation, sur quelle course la mapper.

La soustraction du mouvement camera dans le zoom et le focus ne vient
d'AUCUN des deux. C'est conj(q_camera) * q_objectif, du calcul pur, et le
resultat est independant du repere monde. world.json peut etre refait,
supprime, change : le focus ne bouge pas d'un compte.

La geometrie des base stations n'est ecrite nulle part non plus : libsurvive
la resout seule dans ~/.config/libsurvive/config.json. La console la lit,
pour orienter la normale du sol et sortir le diagnostic d'installation.

USAGE
    ./vp-console.py --demo          # sans materiel
    ./vp-console.py                 # puis http://127.0.0.1:8410

Depuis le Mac :  ssh -L 8410:localhost:8410 unreal
"""

import argparse
import json
import math
import collections
import os
import re
import shutil
import sys
import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import unquote

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
BRIDGE = os.path.join(HERE, "..", "bridge")
sys.path.insert(0, BRIDGE)

import freed           # noqa: E402
import lensaxis        # noqa: E402
import survive_clock   # noqa: E402
import worldframe      # noqa: E402

AXES_PATH = os.path.join(BRIDGE, "axes.json")
WORLD_PATH = os.path.join(BRIDGE, "world.json")
ROLES_PATH = os.path.join(BRIDGE, "roles.json")

ROLES = [
    ("camera", "Tracker CAMÉRA — sur la cage"),
    ("zoom", "Tracker ZOOM — bague de zoom"),
    ("focus", "Tracker FOCUS — bague de mise au point"),
    ("survey", "Appareil de RELEVÉ — posé au sol, 3 points"),
]

PHASES = [
    ("repos", "Ne touche a rien. Camera immobile, bagues bloquees. "
              "Etablit la reference."),
    ("panoramique", "Panoramiques gauche-droite, du lent au franc."),
    ("tilt", "Haut-bas, du lent au franc."),
    ("roulis", "Roulis autour de l'axe optique. LA phase qui juge : "
               "colineaire a l'axe des pignons."),
    ("travelling", "Translation pure. Doit donner zero."),
    ("bagues", "Camera immobile, on balaie les bagues. Controle de vivacite."),
    ("retour", "Bagues rebloquees a leur position de depart."),
]

SLOTS = [("left", "Coin bas GAUCHE de l'ecran"),
         ("right", "Coin bas DROIT de l'ecran"),
         ("camera", "Au sol SOUS la camera")]


def dev_names(obj):
    """Identifiants d'un objet, le plus stable en tete.

    Delegue a survive_clock : le numero de serie grave (LHR-F3D3F946) plutot
    que le nom de code (T20), qui n'est qu'un rang d'enumeration et se
    decale des qu'on branche un appareil de plus.
    """
    return survive_clock.object_names(obj)


def is_lighthouse(ident):
    return survive_clock.is_lighthouse(ident)


# ------------------------------------------------------------------- moteur


class Hub:
    """Possede le contexte libsurvive et l'etat des trois onglets."""

    def __init__(self, demo=False):
        self.demo = demo
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.ctx = None
        if not demo:
            try:
                import pysurvive
            except ImportError:
                sys.exit("pysurvive absent. Voir bridge/requirements.txt")
            self.ctx = pysurvive.SimpleContext(sys.argv[:1])

        self.dev = {}                 # id -> {pos, quat, t, travel}
        # Horodater a l'instant du drain est faux : le retard de file differe
        # d'un tracker a l'autre dans une meme rafale. On decouvre l'horloge.
        self.clock = survive_clock.SurviveClock()
        self.hist = lensaxis.CameraHistory(span=2.0)
        self.camera = None

        # -- studio
        self.slots = {k: [] for k, _ in SLOTS}
        self.capture = None           # (slot, [devices], t_end)
        self.world = None
        self.world_report = None
        self.lighthouses = worldframe.read_lighthouses()

        # -- objectifs
        self.axes = {}
        if os.path.exists(AXES_PATH):
            try:
                cfg = lensaxis.load(AXES_PATH)
                self.camera = cfg.get("camera")
                self.axes = cfg.get("axes", {})
            except (ValueError, OSError):
                pass
        self.sweep = None             # {name, device, samples, pending}
        self.sweep_result = None

        # -- test
        self.tap = None
        self.phase = "libre"
        self.phase_t0 = None
        self._wf = None          # repere plateau precalcule pour la vue de
        self._wf_key = None      # dessus ; recalcule quand world change

        # -- onglet Sortie : la trame Free-D telle qu'Unreal la recevrait.
        # Meme math que vp_bridge.py, pas une approximation : pose camera
        # passee dans world.json, angles d'objectif en relatif camera,
        # conversion en counts par lensaxis.to_freed.
        self.outs = {}           # par axe : cal, inv_ref, accumulateur
        self.outp = []           # echantillons objectif en attente de slerp
        self.freed = None        # derniere trame decodee
        self.sender = None       # FreeDSender si l'emission est active
        self.emit_to = None      # (host, port)
        self.emit_n = 0          # paquets emis
        self.emit_hz = 0.0
        self._emit_t = 0.0
        self._emit_c = 0
        self.results = {}        # onglet Test : verdicts figes par phase

        # Roles attribues a la main, une fois pour toutes, avant toute
        # calibration. Ils portent des NUMEROS DE SERIE : le §8 interdit de
        # lier un appareil a son rang d'enumeration, et on a la preuve que
        # ce rang se decale.
        self.roles = {}
        self._load_roles()
        self.done = []
        self.ref = None
        self.msg = ""

    # -- ingestion --------------------------------------------------------

    def loop(self):
        n = 0
        while not self.stop.is_set():
            now = time.monotonic()
            if self.demo:
                # La demo fournit un vrai instant d'echantillonnage par
                # appareil : elle exerce donc le meme chemin d'horloge que le
                # materiel, au lieu de le court-circuiter.
                for d, (p, q, ts) in demo_poses(
                        now, self.phase, self.sweep is not None).items():
                    self.clock.feed(ts, now)
                    self._ingest(d, p, q, self.clock.to_mono(ts, now))
                if self.clock.state == "apprentissage" and self.clock.ready():
                    self.clock.solve()
                n += 1
            else:
                while True:
                    u = self.ctx.NextUpdated()
                    if u is None:
                        break
                    names = dev_names(u)
                    if not names or is_lighthouse(names[0]):
                        continue
                    ps = u.Pose()[0]
                    raw = survive_clock.read_timecode(u)
                    self.clock.feed(raw, now)
                    if (self.clock.state == "apprentissage"
                            and self.clock.ready()):
                        self.clock.solve()
                    self._ingest(names[0],
                                 (ps.Pos[0], ps.Pos[1], ps.Pos[2]),
                                 (ps.Rot[0], ps.Rot[1], ps.Rot[2], ps.Rot[3]),
                                 self.clock.to_mono(raw, now))
            with self.lock:
                self._resolve(now)
            time.sleep(0.004)

    def _ingest(self, dev, pos, quat, t):
        with self.lock:
            d = self.dev.get(dev)
            if d is None:
                d = self.dev[dev] = {"travel": 0.0, "pos": pos, "n": 0,
                                     # instants d'echantillonnage : debit et
                                     # detection de decrochage
                                     "ts": collections.deque(maxlen=400)}
            else:
                d["travel"] += math.dist(pos, d["pos"])
            d["ts"].append(t)
            d.update(pos=pos, quat=quat, t=t)
            d["n"] += 1

            if dev == self.camera:
                self.hist.push(t, quat, pos)

            if self.capture and dev in self.capture[1]:
                self.slots[self.capture[0]].append((dev, pos))

            if self.sweep and dev in (self.camera, self.sweep["device"]):
                if dev == self.sweep["device"]:
                    self.sweep["pending"].append((t, quat))

            if self.tap and dev in self.tap["devices"]:
                self.tap["pending"].append((t, dev, quat, pos))

            # La sortie Free-D doit vivre des que les axes sont calibres,
            # sans qu'on ait a armer un test.
            if any(c.get("device") == dev for c in self.axes.values()):
                self.outp.append((t, dev, quat))

    def _prepare_frame(self):
        """Repere plateau precalcule, refait quand world change d'identite.

        Appele depuis _resolve et non depuis snapshot : snapshot ne tourne
        que si un navigateur est connecte, alors que la sortie Free-D doit
        etre exprimee dans le repere plateau qu'il y ait un client ou non.
        """
        if self.world is None or self._wf_key is self.world:
            return
        try:
            self._wf = worldframe.prepare(dict(self.world))
        except Exception:                                # noqa: BLE001
            self._wf = None
        self._wf_key = self.world

    def _resolve(self, now):
        self._prepare_frame()
        if self.capture and now > self.capture[2]:
            self.capture = None

        if self.sweep:
            keep = []
            for t, quat in self.sweep["pending"]:
                got = self.hist.at(t)
                if got is None or got[2] == "extrap":
                    if now - t < 0.2:
                        keep.append((t, quat))
                    continue
                if got[2] == "stale":
                    continue
                self.sweep["samples"].append(lensaxis.relative(got[0], quat))
                self.sweep["cam"].append(got[0])
            self.sweep["pending"] = keep

        # Sortie Free-D : meme regle que partout, on n'extrapole jamais la
        # pose camera. Un echantillon plus recent que l'historique attend le
        # tick suivant, et est abandonne au-dela de 200 ms.
        keep_o = []
        for t, dev, quat in self.outp:
            got = self.hist.at(t)
            if got is None or got[2] == "extrap":
                if now - t < 0.2:
                    keep_o.append((t, dev, quat))
                continue
            if got[2] == "stale":
                continue
            self._out_update(t, dev, quat, got)
        self.outp = keep_o[-600:]
        self._out_frame(now)

        if self.tap:
            keep = []
            for t, dev, quat, pos in self.tap["pending"]:
                got = self.hist.at(t)
                if got is None or got[2] == "extrap":
                    if now - t < 0.2:
                        keep.append((t, dev, quat, pos))
                    else:
                        self.tap["lost"] += 1
                    continue
                if got[2] == "stale":
                    self.tap["lost"] += 1
                    continue
                self.tap["exact"] += 1
                self._tap_update(t, dev, quat, pos, got)
            self.tap["pending"] = keep

    # -- sortie Free-D ----------------------------------------------------

    def _out_state(self, name):
        """Accumulateur par axe, cree a la volee et refait si la calibration
        change (nouveau balayage enregistre)."""
        cal = self.axes.get(name)
        if not cal:
            self.outs.pop(name, None)
            return None
        st = self.outs.get(name)
        if st is None or st["cal"] is not cal:
            st = self.outs[name] = {
                "cal": cal,
                "inv_ref": lensaxis.q_conj(tuple(cal["ref"])),
                "acc": lensaxis.Accumulator()}
        return st

    def _out_update(self, t, dev, quat, got):
        """Angle deroule d'un axe, aligne sur l'horodatage de l'echantillon.

        C'est la chaine 'alignee' de l'onglet Test, mais permanente : elle
        n'attend pas qu'on arme un test, puisque la sortie Free-D doit
        exister des que les axes sont calibres.
        """
        name = next((n for n, c in self.axes.items()
                     if c.get("device") == dev), None)
        if name is None:
            return
        st = self._out_state(name)
        if st is None:
            return
        q_cam = got[0]
        st["theta"] = st["acc"].push(lensaxis.twist_angle(
            lensaxis.q_mul(st["inv_ref"], lensaxis.relative(q_cam, quat)),
            st["cal"]["axis"]))

    def _out_frame(self, now):
        """Assemble la trame, l'encode, la decode, et l'emet si demande.

        On decode ce qu'on vient d'encoder plutot que d'afficher les valeurs
        d'entree : ce qui s'affiche est donc ce qui part reellement sur le
        cable, quantification comprise. Une valeur qui saturerait ou se
        tronquerait se verrait ici, pas dans Unreal.
        """
        d = self.dev.get(self.camera)
        if not d or d.get("quat") is None:
            self.freed = None
            return
        pose = (worldframe.apply(self._wf, d["pos"], d["quat"])
                if self._wf is not None else tuple(d["pos"]) + tuple(d["quat"]))

        vals = {}
        for name in ("zoom", "focus"):
            st = self.outs.get(name)
            cal = self.axes.get(name)
            if st is None or cal is None or "theta" not in st:
                vals[name] = None
                continue
            vals[name] = lensaxis.to_freed(st["theta"], cal["lo"], cal["hi"],
                                           invert=cal.get("invert", False))

        pkt = freed.survive_to_freed(pose,
                                     zoom=vals["zoom"] or 0,
                                     focus=vals["focus"] or 0,
                                     camera_id=1)
        out = freed.decode_d1(pkt)
        out["zoom_ok"] = vals["zoom"] is not None
        out["focus_ok"] = vals["focus"] is not None
        out["framed"] = self._wf is not None
        self.freed = out

        if self.sender is not None:
            try:
                self.sender.send(pkt)
                self.emit_n += 1
                self._emit_c += 1
            except OSError as e:                          # noqa: BLE001
                self.msg = "Emission interrompue : %s" % e
                self.emit_stop()
        if now - self._emit_t >= 1.0:
            self.emit_hz = self._emit_c / max(1e-6, now - self._emit_t)
            self._emit_t, self._emit_c = now, 0

    def report(self):
        """Rapport texte des phases terminees, exportable et archivable.

        En texte plutot qu'en JSON : il finira colle dans un carnet de bord
        ou compare a la main d'une session a l'autre, pas relu par un
        programme.
        """
        with self.lock:
            L = ["Rapport de decouplage camera / objectif — free-D",
                 time.strftime("%Y-%m-%d %H:%M:%S"),
                 "",
                 "horloge      : %s" % self.clock.describe(),
                 "camera       : %s" % (self.camera or "non declaree"),
                 "repere       : %s" % ("world.json applique" if self.world
                                        else "AUCUN — coordonnees brutes"),
                 ""]
            for n, c in sorted(self.axes.items()):
                L.append("axe %-6s : %s, course %.1f deg%s"
                         % (n, c.get("device"), c.get("span_deg", 0.0),
                            "  [MULTI-TOUR]" if c.get("span_deg", 0) >= 355.0
                            else ""))
            L += ["",
                  "%-13s %-7s %9s %9s %8s %8s  %s"
                  % ("PHASE", "AXE", "NAIF", "ALIGNE", "COUNTS", "% COURSE",
                     "VERDICT"),
                  "-" * 74]
            if not self.results:
                L.append("(aucune phase terminee)")
            for ph, r in self.results.items():
                for ax, v in r["axes"].items():
                    pct = v["pct"]
                    verdict = ("OK" if pct < 0.3 else
                               "PASSABLE" if pct < 1.0 else "REFAIRE")
                    L.append("%-13s %-7s %8.3f° %8.3f° %8d %7.3f%%  %s"
                             % (ph, ax, v["peak_n"], v["peak_a"], v["counts"],
                                pct, verdict))
            L += ["",
                  "Seuils : moins de 0,3 % de la course = OK, moins de 1 % =",
                  "passable. La phase 'roulis' est la seule qui juge vraiment :",
                  "elle est colineaire a l'axe des pignons.",
                  "",
                  "Un chiffre issu de --demo ne mesure que la coherence du code",
                  "avec ses propres constantes (§11 du handoff)."]
            return "\n".join(L) + "\n"

    def emit_start(self, host, port):
        if self.sender is not None:
            self.emit_stop()
        try:
            port = int(port)
        except (TypeError, ValueError):
            self.msg = "Port invalide."
            return
        self.sender = freed.FreeDSender(host, port)
        self.emit_to = (host, port)
        self.emit_n = 0
        self._emit_t, self._emit_c = time.monotonic(), 0
        self.msg = "Emission Free-D vers %s:%d." % (host, port)

    def emit_stop(self):
        if self.sender is not None:
            try:
                self.sender.close()
            except OSError:
                pass
        self.sender = None
        self.emit_to = None
        self.emit_hz = 0.0

    def _tap_update(self, t, dev, quat, pos, got):
        q_cam, p_cam, _ = got
        name = self.tap["devices"][dev]
        st = self.tap["st"][name]
        q_naive = self.hist.latest()[0]
        axis = st["cal"]["axis"]
        th_a = st["acc_a"].push(lensaxis.twist_angle(
            lensaxis.q_mul(st["inv_ref"], lensaxis.relative(q_cam, quat)),
            axis))
        th_n = st["acc_n"].push(lensaxis.twist_angle(
            lensaxis.q_mul(st["inv_ref"], lensaxis.relative(q_naive, quat)),
            axis))
        d = lensaxis.relative_position(q_cam, p_cam, pos)
        self.tap["rows"].append({
            "phase": self.phase, "t": t, "axis": name,
            "n": th_n, "a": th_a,
            "rate": self.hist.rate(),
            "dist": math.sqrt(d[0] ** 2 + d[1] ** 2 + d[2] ** 2)})

    # -- studio -----------------------------------------------------------

    def studio_capture(self, slot, devices=None, seconds=3.0):
        with self.lock:
            # Un seul appareil, celui qui porte le role « releve ».
            if not devices:
                d = self.roles.get("survey")
                if not d:
                    self.msg = ("Aucun appareil n'a le role releve. "
                                "Onglet Appareils.")
                    return
                devices = [d]
            self.slots[slot] = []
            self.capture = (slot, list(devices),
                            time.monotonic() + seconds)
            self.msg = "Releve de %s..." % slot

    def studio_solve(self, floor_offset_mm=0.0, screen_mm=None):
        with self.lock:
            means, sems = {}, {}
            for k, _lab in SLOTS:
                s = self.slots[k]
                if len(s) < 20:
                    self.msg = "%s : %d echantillons, il en faut 20." \
                        % (k, len(s))
                    return
                # Un point au sol est releve avec UN appareil. Si deux
                # appareils poses a deux endroits ont alimente le meme
                # point, la moyenne donnerait leur milieu — un point qui
                # n'existe nulle part, et rien ne le signalerait. On
                # refuse plutot que de resoudre sur du faux.
                srcs = sorted({d for d, _p in s})
                if len(srcs) > 1:
                    self.msg = ("%s : releve avec %d appareils (%s). Un seul "
                                "par point." % (k, len(srcs), ", ".join(srcs)))
                    return
                a = np.array([p for _d, p in s], float)
                means[k] = a.mean(axis=0)
                sems[k] = a.std(axis=0) / math.sqrt(len(a))

            pts = [means["left"], means["right"], means["camera"]]
            try:
                normal, centroid, rms = worldframe.fit_plane(pts)
            except ValueError as e:
                self.msg = str(e)
                return
            big, small = worldframe.conditioning(pts)
            unc = worldframe.normal_uncertainty(
                pts, [sems["left"], sems["right"], sems["camera"]])

            if self.lighthouses:
                above = np.mean(list(self.lighthouses.values()), axis=0)
                normal = worldframe.orient_normal(normal, centroid, above)
                oriented = "base stations"
            else:
                normal = normal if normal[2] > 0 else -normal
                oriented = "suppose (+Z libsurvive) — config introuvable"

            try:
                frame = worldframe.build(normal, means["left"], means["right"],
                                         means["camera"],
                                         floor_offset_mm=floor_offset_mm)
            except ValueError as e:
                self.msg = str(e)
                return

            f = worldframe.prepare(dict(frame))
            rep = (worldframe.lighthouse_report(f, self.lighthouses)
                   if len(self.lighthouses) >= 2 else None)

            frame.update(floor_rms_mm=rms, normal_uncertainty_deg=unc,
                         triangle_m=[big, small],
                         floor_points=[[float(x) for x in p] for p in pts])
            if screen_mm:
                frame["tape_mm"] = screen_mm
            if rep:
                frame["lighthouses"] = rep

            self.world = frame
            self.world_report = {
                "triangle": [big, small], "unc": unc, "rms": rms,
                "oriented": oriented, "lh": rep,
                "scale_err_mm": (frame["screen_width_mm"] - screen_mm)
                if screen_mm else None}
            self.msg = "Repere resolu."

    def studio_save(self):
        with self.lock:
            if not self.world:
                self.msg = "Rien a enregistrer."
                return
            self.world["calibrated"] = time.strftime("%Y-%m-%d %H:%M:%S")
            worldframe.save(WORLD_PATH, self.world)
            self.msg = "world.json enregistre."

    # -- objectifs ---------------------------------------------------------

    # -- roles -------------------------------------------------------------

    def _load_roles(self):
        try:
            with open(ROLES_PATH) as f:
                d = json.load(f)
        except (OSError, ValueError):
            return
        if isinstance(d, dict):
            self.roles = {k: v for k, v in d.items()
                          if k in dict(ROLES) and v}
            if self.roles.get("camera"):
                self.camera = self.roles["camera"]

    def _save_roles(self):
        try:
            with open(ROLES_PATH, "w") as f:
                json.dump(self.roles, f, indent=2)
                f.write("\n")
        except OSError as e:                              # noqa: BLE001
            self.msg = "roles.json non ecrit : %s" % e

    def set_role(self, role, dev):
        """Attribue un role. Un appareil ne peut en tenir qu'un.

        Assigner deux roles au meme appareil produirait des mesures qui ont
        l'air justes : le zoom suivrait le focus, et rien ne le dirait.
        """
        with self.lock:
            if role not in dict(ROLES):
                self.msg = "Role inconnu : %s" % role
                return
            if not dev:
                self.roles.pop(role, None)
                if role == "camera":
                    self.camera = None
                self._save_roles()
                self.msg = "Role %s libere." % role
                return
            held = next((r for r, v in self.roles.items()
                         if v == dev and r != role), None)
            if held:
                self.msg = ("%s tient deja le role %s. Un appareil, un role."
                            % (dev, held))
                return
            before = self.roles.get(role)
            self.roles[role] = dev
            if role == "camera":
                self.camera = dev
                self.hist = lensaxis.CameraHistory(span=2.0)
            self._save_roles()

            dropped = self._invalidate(role, before, dev)
            self.msg = "%s : %s" % (role, dev)
            if dropped:
                self.msg += (" — calibration de %s effacee, a refaire."
                             % ", ".join(dropped))

    def _invalidate(self, role, before, dev):
        """Efface ce qu'un changement d'appareil rend caduc.

        Remplacer un tracker physique et garder sa calibration donnerait des
        valeurs plausibles et fausses, sans rien signaler. Deux cas :

        - un axe change d'appareil : sa course et son axe de rotation ont ete
          releves sur l'ancien montage, ils ne valent rien pour le nouveau ;
        - la CAMERA change : les deux axes sont calibres en relatif camera
          (cal["ref"] est un conj(q_cam)*q_objectif), donc les deux tombent.

        world.json n'est pas touche : il vient des trois points au sol, pas
        de la camera.
        """
        if before == dev:
            return []
        dropped = []
        if role in ("zoom", "focus"):
            if self.axes.pop(role, None) is not None:
                dropped.append(role)
        elif role == "camera":
            for n in list(self.axes):
                self.axes.pop(n)
                dropped.append(n)
        if dropped:
            self.outs.clear()
            lensaxis.save(AXES_PATH,
                          {"camera": self.camera, "axes": self.axes})
        return dropped

    def set_camera(self, dev):
        self.set_role("camera", dev)

    def sweep_start(self, name, dev=None):
        with self.lock:
            # L'appareil vient du role attribue a l'onglet Appareils. Le
            # menu du balayage a disparu : choisir la un tracker different
            # de celui qui portera l'axe en production n'aurait aucun sens.
            dev = dev or self.roles.get(name)
            if not self.camera:
                self.msg = "Attribue d'abord le role camera."
                return
            if not dev:
                self.msg = ("Aucun tracker n'a le role %s. Onglet Appareils."
                            % name)
                return
            if dev == self.camera:
                self.msg = "Le tracker objectif ne peut pas etre la camera."
                return
            self.sweep = {"name": name, "device": dev, "samples": [],
                          "cam": [], "pending": [], "t0": time.monotonic()}
            self.sweep_result = None
            self.msg = "Balaie %s lentement, butee a butee." % name

    def sweep_stop(self):
        with self.lock:
            if not self.sweep:
                return
            s = self.sweep
            self.sweep = None
            try:
                cal = lensaxis.fit_axis(s["samples"])
            except ValueError as e:
                self.msg = "Calibration impossible : %s" % e
                return
            move = 0.0
            if len(s["cam"]) > 1:
                q0 = s["cam"][0]
                step = max(1, len(s["cam"]) // 300)
                move = max(math.degrees(math.sqrt(sum(
                    c * c for c in lensaxis.q_log(
                        lensaxis.q_mul(lensaxis.q_conj(q0), q)))))
                    for q in s["cam"][::step])
            v, why = lensaxis.verdict(cal)
            cal["device"] = s["device"]
            cal["invert"] = False
            cal["camera_motion_deg"] = move
            self.sweep_result = {"name": s["name"], "cal": cal,
                                 "verdict": v, "why": why,
                                 "camera_motion_deg": move}
            self.msg = "%s : %s" % (s["name"], v)

    def sweep_save(self, invert=False):
        with self.lock:
            r = self.sweep_result
            if not r:
                self.msg = "Aucun balayage a enregistrer."
                return
            r["cal"]["invert"] = invert
            r["cal"]["calibrated"] = time.strftime("%Y-%m-%d %H:%M:%S")
            self.axes[r["name"]] = r["cal"]
            lensaxis.save(AXES_PATH,
                          {"camera": self.camera, "axes": self.axes})
            self.sweep_result = None
            self.msg = "axes.json enregistre (%s)." % r["name"]

    # -- test --------------------------------------------------------------

    def test_arm(self):
        with self.lock:
            if not self.camera or not self.axes:
                self.msg = "Il faut un tracker camera et au moins un axe."
                return
            self.tap = {
                "devices": {c["device"]: n for n, c in self.axes.items()},
                "st": {n: {"cal": c,
                           "inv_ref": lensaxis.q_conj(tuple(c["ref"])),
                           "acc_n": lensaxis.Accumulator(),
                           "acc_a": lensaxis.Accumulator()}
                       for n, c in self.axes.items()},
                "rows": [], "pending": [], "exact": 0, "lost": 0}
            self.done = []
            self.results = {}
            self.ref = None
            self.phase = "libre"
            self.msg = "Test arme. Bloque les bagues au ruban."

    def phase_start(self, name):
        with self.lock:
            if not self.tap:
                self.msg = "Arme d'abord le test."
                return
            self.phase = name
            self.phase_t0 = time.monotonic()

    def phase_stop(self):
        with self.lock:
            if self.phase == "libre" or not self.tap:
                return
            if self.phase not in self.done:
                self.done.append(self.phase)
            # La crete n'existait que pendant la phase : au phase_stop elle
            # disparaissait, et rien ne restait a comparer entre phases ni
            # entre sessions. On la fige ici.
            snap = self._test_snapshot()
            self.results[self.phase] = {
                "t": time.time(),
                "dur": (time.monotonic() - self.phase_t0)
                       if self.phase_t0 else 0.0,
                "axes": {n: dict(v) for n, v in snap["axes"].items()},
                "exact_pct": snap["exact_pct"]}
            if self.phase == "repos":
                self.ref = {}
                for name in self.axes:
                    r = [x for x in self.tap["rows"]
                         if x["phase"] == "repos" and x["axis"] == name]
                    if r:
                        self.ref[name] = {
                            "n": float(np.degrees(np.mean([x["n"] for x in r]))),
                            "a": float(np.degrees(np.mean([x["a"] for x in r])))}
            self.phase = "libre"
            self.phase_t0 = None

    # -- instantane --------------------------------------------------------

    def snapshot(self):
        with self.lock:
            now = time.monotonic()
            devs = []
            for k, d in sorted(self.dev.items()):
                age = now - d["t"]
                # Un appareil qui ne remonte plus rien garde une file
                # d'horodatages figee : calculer un debit dessus afficherait
                # 240 Hz pour un tracker debranche depuis neuf minutes. Vu en
                # vrai le 1er septembre 2026. Passe ce delai, il est absent,
                # et on le dit plutot que de le laisser paraitre sain.
                gone = age > 1.0
                ts = d.get("ts")
                rate = gap = 0.0
                drops = 0
                if not gone and ts and len(ts) > 4:
                    span = ts[-1] - ts[0]
                    if span > 1e-3:
                        rate = (len(ts) - 1) / span
                    # Un trou dans les instants d'echantillonnage est un
                    # decrochage optique : le tracker n'a rien vu. C'est ce
                    # qui coute un tour sur un axe multi-tour (§10).
                    prev = None
                    for x in ts:
                        if prev is not None:
                            dt = x - prev
                            if dt > gap:
                                gap = dt
                            if dt > 0.1:
                                drops += 1
                        prev = x

                xy = None
                if self._wf is not None:
                    try:
                        f = worldframe.apply(self._wf, d["pos"],
                                             d.get("quat") or (1.0, 0, 0, 0))
                        xy = [round(f[0], 3), round(f[1], 3), round(f[2], 3)]
                    except Exception:                    # noqa: BLE001
                        xy = None

                devs.append({"id": k, "travel": round(d["travel"], 2),
                             "age_ms": age * 1000.0,
                             "gone": gone,
                             "rate": round(rate, 1),
                             "gap_ms": round(gap * 1000.0, 1),
                             "drops": drops,
                             "xy": xy,
                             "role": ("camera" if k == self.camera else
                                      next((n for n, c in self.axes.items()
                                            if c.get("device") == k), ""))})
            out = {"devices": devs, "camera": self.camera, "msg": self.msg,
                   "clock": self.clock.describe(),
                   "clock_ok": self.clock.scale is not None,
                   "tab_ready": {"studio": bool(self.world),
                                 "axes": sorted(self.axes),
                                 "test": bool(self.tap)}}

            out["capture"] = ({"slot": self.capture[0],
                               "left": max(0.0, self.capture[2] - now)}
                              if self.capture else None)
            out["slots"] = {k: len(v) for k, v in self.slots.items()}
            # Quel appareil a servi a chaque point : verifiable d'un coup
            # d'oeil avant de resoudre.
            out["slot_src"] = {
                k: (sorted({d for d, _p in v})[0] if len({d for d, _p in v}) == 1
                    else (", ".join(sorted({d for d, _p in v})) if v else None))
                for k, v in self.slots.items()}
            out["world"] = self.world
            out["world_report"] = self.world_report

            if self.sweep:
                out["sweep"] = {"name": self.sweep["name"],
                                "n": len(self.sweep["samples"]),
                                "s": now - self.sweep["t0"]}
            else:
                out["sweep"] = None
            out["sweep_result"] = self.sweep_result
            out["axes"] = {n: {"device": c.get("device"),
                               "span": c.get("span_deg"),
                               "multiturn": c.get("span_deg", 0) >= 355.0}
                           for n, c in self.axes.items()}

            out["phase"] = self.phase
            out["done"] = list(self.done)
            out["elapsed"] = (now - self.phase_t0) if self.phase_t0 else 0.0
            out["ref_ok"] = self.ref is not None
            out["cam_rate"] = math.degrees(self.hist.rate())
            out["test"] = self._test_snapshot() if self.tap else None
            out["results"] = self.results
            out["roles"] = dict(self.roles)
            out["role_defs"] = [[k, lab] for k, lab in ROLES]
            out["freed"] = self.freed
            out["emit"] = {"on": self.sender is not None,
                           "to": ("%s:%d" % self.emit_to) if self.emit_to else "",
                           "n": self.emit_n,
                           "hz": round(self.emit_hz, 1)}

            # Verdict de sante, pour le bandeau permanent. Le pire des
            # trackers commande : c'est lui qui gatera la mesure.
            assigned = [d for d in devs if d["role"]]
            watch = assigned or devs
            gone = [d for d in devs if d["gone"]]
            out["health"] = {
                "clock_ok": self.clock.scale is not None,
                "n_dev": len(devs) - len(gone),
                "n_gone": len(gone),
                "n_assigned": len(assigned),
                "worst_age_ms": round(max([d["age_ms"] for d in watch],
                                          default=0.0), 1),
                "worst_gap_ms": round(max([d["gap_ms"] for d in watch],
                                          default=0.0), 1),
                "min_rate": round(min([d["rate"] for d in watch],
                                      default=0.0), 1),
                "drops": sum(d["drops"] for d in watch),
                "lh_seen": len(self.lighthouses or {}),
                # Un axes.json produit en --demo reste sur le disque et sera
                # relu au demarrage suivant, bridge compris. Le dire fort
                # plutot que de laisser croire a une calibration reelle.
                "demo_cal": any(
                    str(v).startswith("DEMO-")
                    for v in [self.camera] + [c.get("device")
                                              for c in self.axes.values()]),
            }
            return out

    def _test_snapshot(self):
        t0 = (self.phase_t0 or 0.0) + 0.5
        tot = self.tap["exact"] + self.tap["lost"]
        res = {"exact_pct": 100.0 * self.tap["exact"] / max(1, tot), "axes": {}}
        for name, cal in self.axes.items():
            rows = [x for x in self.tap["rows"] if x["axis"] == name]
            r = (self.ref or {}).get(name)
            last = rows[-1] if rows else None
            span = cal["span_deg"]
            ph = [x for x in rows
                  if x["phase"] == self.phase and x["t"] >= t0] \
                if self.phase != "libre" else rows[-1500:]
            pn = pa = 0.0
            if ph and r:
                pn = float(np.max(np.abs(
                    np.degrees([x["n"] for x in ph]) - r["n"])))
                pa = float(np.max(np.abs(
                    np.degrees([x["a"] for x in ph]) - r["a"])))
            drift = 0.0
            if len(rows) > 30:
                d = [x["dist"] for x in rows[-1500:]]
                drift = (max(d) - min(d)) * 1000.0
            res["axes"][name] = {
                "n": (math.degrees(last["n"]) - r["n"]) if (last and r) else 0.0,
                "a": (math.degrees(last["a"]) - r["a"]) if (last and r) else 0.0,
                "peak_n": pn, "peak_a": pa, "span": span,
                "counts": int(pa / span * 65535),
                "pct": pa / span * 100.0, "drift_mm": drift}
        return res


# ------------------------------------------------------------------ demo


def _q(axis, ang):
    a = np.asarray(axis, float)
    a /= np.linalg.norm(a)
    s = math.sin(ang / 2.0)
    return lensaxis.q_norm((math.cos(ang / 2.0), a[0] * s, a[1] * s, a[2] * s))


_DEMO_R = None


def demo_poses(t, phase, sweeping=False):
    """Plateau fabrique : ecran de 4 m, camera a 4,2 m, base stations au
    plafond. Repere libsurvive volontairement de travers."""
    global _DEMO_R
    if _DEMO_R is None:
        def rot(ax, an):
            a = np.asarray(ax, float)
            a /= np.linalg.norm(a)
            k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
            return np.eye(3) + math.sin(an) * k + (1 - math.cos(an)) * (k @ k)
        _DEMO_R = (rot([0.4, 0.8, 0], math.radians(5.0))
                   @ rot([0, 0, 1], math.radians(-25.0)),
                   np.array([-0.9, 3.1, 1.2]))
    r, o = _DEMO_R

    def to_s(p):
        return tuple(o + r @ np.asarray(p, float))

    rng = np.random.default_rng(int(t * 1000) % 100000)
    n = lambda s=0.002: rng.normal(scale=s, size=3)   # noqa: E731

    amp = {"panoramique": 40.0, "tilt": 25.0, "roulis": 55.0}.get(phase, 0.0)
    ax = {"panoramique": [0, 0, 1], "tilt": [0, 1, 0],
          "roulis": [1, 0, 0]}.get(phase, [0, 0, 1])
    ang = math.radians(amp) * math.sin(2 * math.pi * 0.7 * t)
    q_move = _q(r @ np.asarray(ax, float), ang)

    out = {}
    # Trois positions au sol. En vrai tu n'as qu'UN appareil de releve — le
    # quatrieme tracker, §2bis — que tu deplaces d'un point a l'autre ; la
    # demo en montre trois a la fois pour que les releves s'enchainent sans
    # manipulation. La console, elle, exige bien un seul appareil par point.
    # Chaque appareil porte son propre instant d'echantillonnage. Les
    # trackers d'objectif sont echantillonnes 3 ms avant la camera : c'est le
    # decalage que l'horloge doit rendre visible et que le slerp doit annuler.
    out["DEMO-SURV1"] = (to_s([0.0, -2.0, 0.031] + n()), (1.0, 0, 0, 0), t)
    out["DEMO-SURV2"] = (to_s([0.0, 2.0, 0.031] + n()), (1.0, 0, 0, 0), t)
    out["DEMO-SURV3"] = (to_s([4.2, 0.06, 0.031] + n()), (1.0, 0, 0, 0), t)
    out["DEMO-CAM"] = (to_s([4.2, 0.06, 1.35] + n(0.001)), q_move, t)

    lens_axis = tuple(r @ np.array([1.0, 0.0, 0.0]))
    for dev, side, sp in (("DEMO-FOC", -0.09, 300.0), ("DEMO-ZOO", 0.09, 180.0)):
        b = math.radians(120.0)
        if phase == "bagues" or sweeping:
            # Pendant un balayage de calibration, la demo fait tourner les
            # bagues : sinon l'onglet Objectifs n'aurait rien a ajuster.
            b += math.radians(sp * 0.45) * math.sin(2 * math.pi * 0.2 * t)
        q = lensaxis.q_mul(_q(r @ np.asarray(ax, float),
                              math.radians(amp) * math.sin(
                                  2 * math.pi * 0.7 * (t - 0.003))),
                           _q(lens_axis, b))
        out[dev] = (to_s([4.2, 0.06 + side, 1.35] + n(0.001)), q, t - 0.003)
    return out


DEMO_LH = None


# ------------------------------------------------------------------ page

PAGE = r"""<!DOCTYPE html>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Console Free-D</title>
<style>
:root{--panel:#1b1e24;--panel2:#22262e;--rule:#31363f;--ink:#e8e6e1;
 --dim:#8b929e;--naif:#e2a03f;--aligne:#4fc3d9;--cam:#c77dbb;
 --ok:#7fbf6a;--warn:#e2a03f;--bad:#e05c5c}
*{box-sizing:border-box}
html,body{margin:0;height:100%}
body{background:var(--panel);color:var(--ink);font-size:14px;
 font-family:"Ubuntu","DejaVu Sans",system-ui,sans-serif;
 display:flex;flex-direction:column}
.mono{font-family:"Ubuntu Mono","DejaVu Sans Mono",monospace;
 font-variant-numeric:tabular-nums}
.eyebrow{font-family:"Ubuntu Condensed","Ubuntu",sans-serif;
 text-transform:uppercase;letter-spacing:.14em;font-size:11px;color:var(--dim)}
header{display:flex;align-items:center;gap:6px;padding:0 16px;
 border-bottom:1px solid var(--rule);background:var(--panel2)}
header h1{margin:0 22px 0 0;font-family:"Ubuntu Condensed",sans-serif;
 font-size:17px;font-weight:500;letter-spacing:.04em}
.tab{padding:14px 18px;cursor:pointer;border-bottom:2px solid transparent;
 font-family:"Ubuntu Condensed",sans-serif;text-transform:uppercase;
 letter-spacing:.1em;font-size:12.5px;color:var(--dim)}
.tab.on{color:var(--ink);border-bottom-color:var(--aligne)}
.tab .chk{color:var(--ok);margin-left:6px}
main{flex:1;overflow:auto;padding:18px}
.pane{display:none}.pane.on{display:block}
.grid{display:grid;grid-template-columns:300px 1fr;gap:20px}
@media(max-width:860px){.grid{grid-template-columns:1fr}}
.card{background:var(--panel2);border:1px solid var(--rule);border-radius:3px;
 padding:14px;margin-bottom:14px}
.card h3{margin:0 0 10px;font-family:"Ubuntu Condensed",sans-serif;
 font-size:13px;text-transform:uppercase;letter-spacing:.11em;font-weight:500}
button{background:#2d3a42;color:var(--ink);border:1px solid var(--rule);
 border-radius:3px;padding:8px 12px;font-family:"Ubuntu Condensed",sans-serif;
 text-transform:uppercase;letter-spacing:.09em;font-size:12px;cursor:pointer}
button:hover{background:#37454e}
button:focus-visible{outline:2px solid var(--aligne);outline-offset:2px}
button.go{background:#2e4436;border-color:#3d5a45}
button.stop{background:#432d2d;border-color:#5a3a3a}
button[disabled]{opacity:.4;cursor:default}
select,input{background:#171a1f;color:var(--ink);border:1px solid var(--rule);
 border-radius:3px;padding:6px;font-family:inherit;width:100%}
table{width:100%;border-collapse:collapse;font-size:13px}
td,th{padding:5px 6px;text-align:left;border-bottom:1px solid var(--rule)}
th{color:var(--dim);font-weight:400;font-size:11px;text-transform:uppercase;
 letter-spacing:.1em}
.dot{display:inline-block;width:8px;height:8px;border-radius:50%;
 background:var(--bad);margin-right:6px}
.dot.ok{background:var(--ok)}.dot.warn{background:var(--warn)}
canvas{width:100%;display:block;background:#171a1f;border:1px solid var(--rule);
 border-radius:2px}
.kv{display:flex;justify-content:space-between;padding:4px 0;
 border-bottom:1px solid var(--rule)}
/* Lignes de releve : le libelle s'allongeait jusqu'a toucher le compteur.
   Grille explicite — libelle souple, compteur et bouton a largeur fixe. */
#slots .kv{display:grid;grid-template-columns:1fr auto auto;gap:10px;
 align-items:center;font-size:12.5px;line-height:1.3}
#slots .kv b{font-size:12.5px;min-width:2ch;text-align:right}
#slots .kv button{padding:5px 10px;font-size:11px}
.kv b{font-weight:400;font-family:"Ubuntu Mono",monospace}
.big{font-size:21px}
.v-OK{color:var(--ok)}.v-PASSABLE{color:var(--warn)}.v-REFAIRE{color:var(--bad)}
ol.ph{list-style:none;margin:0;padding:0;counter-reset:p}
ol.ph li{counter-increment:p;display:grid;grid-template-columns:20px 1fr;
 gap:8px;padding:8px 6px;border-radius:3px;cursor:pointer}
ol.ph li:hover{background:#2a2f38}
ol.ph li::before{content:counter(p);color:var(--dim);font-size:12px;
 font-family:"Ubuntu Mono",monospace}
ol.ph li.on{background:#2d3a42;outline:1px solid var(--aligne)}
ol.ph li.done::before{content:"✓";color:var(--ok)}
ol.ph li.key b{color:var(--naif)}
ol.ph b{font-weight:500;display:block}
/* Le li est une grille 20px/1fr : ::before prend (col1,ligne1), le <b>
   (col2,ligne1), et le <small> retombait en (col1,ligne2) — 20 px de
   large, soit un mot par ligne. On le force en colonne 2. */
ol.ph small{grid-column:2;color:var(--dim);font-size:11.5px;
 line-height:1.35}
.hb{display:flex;align-items:center;gap:14px;flex-wrap:wrap;
  padding:7px 16px;border-bottom:1px solid var(--rule);
  background:var(--panel2);font-size:12px;
  font-family:"Ubuntu Mono",monospace}
.hb .lamp{width:8px;height:8px;border-radius:50%;display:inline-block;
  margin-right:6px;vertical-align:1px}
.hb .chip{padding:2px 8px;border:1px solid var(--rule);border-radius:10px;
  color:var(--dim);white-space:nowrap}
.hb .chip b{color:var(--ink);font-weight:600}
.hb .alarm{color:var(--bad);border-color:var(--bad)}
.hb .warn2{color:var(--warn);border-color:var(--warn)}
.bar{padding:9px 16px;border-top:1px solid var(--rule);background:var(--panel2);
 color:var(--dim);font-size:12.5px}
.note{color:var(--dim);font-size:12.5px;line-height:1.5}
</style>

<header>
  <h1>Console Free-D</h1>
  <div class="tab on" data-t="dev">Appareils<span class="chk" id="c-dev"></span></div>
  <div class="tab" data-t="studio">Studio<span class="chk" id="c-studio"></span></div>
  <div class="tab" data-t="axes">Objectifs<span class="chk" id="c-axes"></span></div>
  <div class="tab" data-t="test">Test<span class="chk" id="c-test"></span></div>
  <div class="tab" data-t="out">Sortie<span class="chk" id="c-out"></span></div>
  <div style="margin-left:auto" class="eyebrow" id="hdr"></div>
</header>

<!-- Bandeau de sante : hors des onglets, donc toujours visible. Il dit si
     le TRACKING va mal, avant que ca ne se voie dans une mesure. -->
<div class="hb" id="hb"><span class="chip">en attente…</span></div>

<main>
<!-- ------------------------------------------------- APPAREILS -->
<div class="pane on" id="p-dev"><div class="grid">
  <div>
    <div class="card"><h3>Appareils vus</h3>
      <table><tbody id="devs"></tbody></table>
      <p class="note" style="margin:9px 0 0">Bouge un appareil pour
      l'identifier : la colonne de droite compte les mètres parcourus.
      L'identifiant est le <b>numéro de série gravé</b> — <code>LHR-</code>
      pour un tracker ou un joystick. Il ne change jamais, contrairement aux
      noms <code>T20</code>, <code>T21</code>… de libsurvive, qui se
      décalent dès qu'on branche un appareil de plus.</p>
    </div>
  </div>
  <div>
    <div class="card"><h3>Rôles</h3>
      <div id="roles"></div>
      <p class="note">À faire <b>en premier</b>, avant tout relevé et toute
      calibration. Un appareil ne peut tenir qu'un rôle.</p>
      <p class="note">Si tu remplaces un tracker, réattribue simplement son
      rôle ici : la calibration qui en dépendait est effacée et redemandée.
      Changer le tracker <b>caméra</b> efface les deux axes — ils sont
      calibrés en relatif caméra.</p>
    </div>
  </div>
</div></div>

<!-- ------------------------------------------------- STUDIO -->
<div class="pane" id="p-studio"><div class="grid">
  <div>
    <div class="card"><h3>Relevé — 3 points</h3>
      <p class="note" style="margin:0 0 10px">Relevé avec
        <b id="survey-who">—</b></p>
      <div id="slots"></div>
      <p class="note">Trois points non alignés déterminent le plan. Deux
      laisseraient libre le roulis du sol, donc l'inclinaison de l'horizon
      virtuel.</p>
    </div>
    <div class="card"><h3>Résoudre</h3>
      <label class="eyebrow">Largeur d'écran au ruban (mm)</label>
      <input id="screen-mm" class="mono" placeholder="4000">
      <label class="eyebrow" style="display:block;margin-top:9px">Hauteur du
      centre suivi au-dessus du sol (mm)</label>
      <input id="floor-off" class="mono" placeholder="0">
      <div style="display:flex;gap:8px;margin-top:11px">
        <button class="go" id="b-solve">Résoudre</button>
        <button id="b-wsave">Enregistrer</button>
      </div>
    </div>
  </div>
  <div>
    <div class="card"><h3>Vue de dessus</h3><canvas id="top" height="260"></canvas>
      <p class="note" style="margin:9px 0 0">+X va de l'écran vers la caméra —
      ta ligne médiane. Le repère est ancré sur l'écran : le déport de la
      caméra est mesuré, pas annulé.</p></div>
    <div class="card"><h3>Résultat</h3><div id="wrep" class="note">Relève les
      trois points, puis résous.</div></div>
  </div>
</div></div>

<!-- ------------------------------------------------- OBJECTIFS -->
<div class="pane" id="p-axes"><div class="grid">
  <div>
    <div class="card"><h3>Balayage d'axe</h3>
      <label class="eyebrow">Axe</label>
      <select id="sel-axis"><option>focus</option><option>zoom</option></select>
      <p class="note" style="margin:9px 0 0">Tracker :
        <b id="lens-who">—</b> — attribué dans l'onglet Appareils.</p>
      <div style="display:flex;gap:8px;margin-top:11px">
        <button class="go" id="b-sw-start">Démarrer</button>
        <button class="stop" id="b-sw-stop">Arrêter</button>
      </div>
      <p class="note"><b>Deux passes.</b> Un axe à la fois : choisis
      « focus », balaie, enregistre — puis reviens ici, choisis « zoom » et
      recommence avec l'autre tracker. Chaque bague a le sien.</p>
      <p class="note">Caméra immobile sur trépied. Butée à butée, lentement.</p>
    </div>
    <div class="card"><h3>Axes enregistrés</h3>
      <table><tbody id="axlist"></tbody></table>
      <p class="note">Les deux sont nécessaires. Le §2 impose de monter les
      trackers d'objectif <b>un de chaque côté</b> du bloc optique : du même
      côté, le corps caméra masquerait une base station à chacun, et la
      redondance disparaîtrait là où un décrochage coûte le plus cher.</p>
      </div>
  </div>
  <div>
    <div class="card"><h3>Angle relevé</h3><canvas id="sweep" height="220"></canvas></div>
    <div class="card"><h3>Verdict</h3><div id="swrep" class="note">Aucun
      balayage.</div></div>
  </div>
</div></div>

<!-- ------------------------------------------------- TEST -->
<div class="pane" id="p-test"><div class="grid">
  <div>
    <div class="card"><h3>Phases</h3>
      <ol class="ph" id="phases"></ol>
      <div style="display:flex;gap:8px;margin-top:11px">
        <button id="b-arm">Armer</button>
        <button class="stop" id="b-ph-stop">Arrêter</button>
      </div>
      <p class="note">Bagues bloquées au ruban, sauf phase « bagues ».
      Fais « repos » en premier : c'est elle qui donne la référence.</p>
    </div>
    <div class="card"><h3>Bilan</h3><div id="tstat"></div></div>
    <div class="card"><h3>Phases terminées</h3>
      <table><tbody id="tres"></tbody></table>
      <div style="margin-top:10px">
        <a id="b-report" href="report" target="_blank"><button>Exporter le rapport</button></a>
      </div>
      <p class="note">La crête d'une phase n'existait que pendant qu'elle
      tournait. Elle est maintenant conservée : les phases se comparent entre
      elles, et le rapport se garde d'une session à l'autre.</p></div>
  </div>
  <div id="scopes"></div>
</div></div>
<!-- ------------------------------------------------- SORTIE -->
<div class="pane" id="p-out"><div class="grid">
  <div>
    <div class="card"><h3>Émission</h3>
      <div class="row"><label>Hôte</label>
        <input id="ehost" value="127.0.0.1"></div>
      <div class="row"><label>Port</label>
        <input id="eport" value="40000"></div>
      <div style="display:flex;gap:8px;margin-top:11px">
        <button class="go" id="b-emit">Émettre</button>
        <button class="stop" id="b-emit-stop">Arrêter</button>
      </div>
      <div id="estat" class="note" style="margin-top:10px"></div>
      <p class="note">Unreal écoute par défaut sur le port 40000. La console
      et le bridge s'excluent — libsurvive n'admet qu'un seul processus —
      donc c'est ici OU <code>vp_bridge.py</code>, jamais les deux.</p>
    </div>
    <div class="card"><h3>Ce qui manque</h3><div id="oreq" class="note"></div></div>
  </div>
  <div>
    <div class="card"><h3>Trame Free-D D1</h3><div id="oframe"></div>
      <p class="note">Ces valeurs sont décodées depuis les 29 octets
      réellement encodés, pas depuis les valeurs d'entrée : ce que tu lis est
      ce qui part sur le câble, quantification comprise.</p></div>
  </div>
</div></div>
</main>

<div class="bar" id="msg">Prêt.</div>

<script>
const N=900, S={};let cfg=null,camBuf=ring(N),sweepBuf=ring(N),last=null;
function ring(n){return{a:new Float32Array(n),i:0,n:0}}
function push(r,v){r.a[r.i]=v;r.i=(r.i+1)%r.a.length;if(r.n<r.a.length)r.n++}
function at(r,k){return r.a[(r.i-r.n+k+r.a.length)%r.a.length]}
function el(t,c,x){const e=document.createElement(t);if(c)e.className=c;
 if(x!==undefined)e.textContent=x;return e}
function go(q){return fetch("cmd?"+q)}
function $(i){return document.getElementById(i)}

document.querySelectorAll(".tab").forEach(t=>t.onclick=()=>{
  document.querySelectorAll(".tab").forEach(x=>x.classList.remove("on"));
  document.querySelectorAll(".pane").forEach(x=>x.classList.remove("on"));
  t.classList.add("on");$("p-"+t.dataset.t).classList.add("on")});

fetch("config").then(r=>r.json()).then(c=>{cfg=c;build();tick()});

function build(){
  const sl=$("slots");
  cfg.slots.forEach(([k,lab])=>{
    const row=el("div","kv");
    row.appendChild(el("span",null,lab));
    const b=el("button",null,"Relever");b.onclick=()=>{
      // UN seul appareil par point. Le releve moyenne les echantillons
      // recus ; en accepter deux, poses a deux endroits, donnerait
      // silencieusement leur milieu.
      // L'appareil vient du role « relevé » (onglet Appareils) : le
      // serveur le resout seul, et refuse si aucun n'est attribue.
      go("studio_capture="+k)};
    const n=el("b","mono","0");n.id="slot-"+k;
    row.appendChild(n);row.appendChild(b);sl.appendChild(row)});

  const ol=$("phases");
  cfg.phases.forEach(([n,h])=>{
    const li=el("li");li.dataset.name=n;if(n==="roulis")li.className="key";
    li.appendChild(el("b",null,n));li.appendChild(el("small",null,h));
    li.onclick=()=>go("phase="+encodeURIComponent(n));ol.appendChild(li)});

  $("b-solve").onclick=()=>go("studio_solve=1&screen_mm="
    +($("screen-mm").value||0)+"&floor_off="+($("floor-off").value||0));
  $("b-wsave").onclick=()=>go("studio_save=1");
  $("b-sw-start").onclick=()=>{sweepBuf=ring(N);
    go("sweep_start="+$("sel-axis").value)};
  $("b-sw-stop").onclick=()=>go("sweep_stop=1");
  $("b-arm").onclick=()=>go("test_arm=1");
  $("b-emit").onclick=()=>go("emit_start=1&ehost="
    +encodeURIComponent($("ehost").value)+"&eport="
    +encodeURIComponent($("eport").value));
  $("b-emit-stop").onclick=()=>go("emit_stop=1");
  $("b-ph-stop").onclick=()=>go("phase_stop=1");
}

const ev=new EventSource("stream");
ev.onmessage=e=>{const s=JSON.parse(e.data);last=s;
  $("msg").textContent=s.msg||"Prêt.";
  $("c-studio").textContent=s.tab_ready.studio?"✓":"";
  $("c-axes").textContent=s.tab_ready.axes.length?"✓":"";
  $("c-test").textContent=s.tab_ready.test?"✓":"";
  $("hdr").textContent=(s.camera?("caméra "+s.camera+" · "):"")+(s.clock||"");
  $("hdr").style.color=s.clock_ok?"var(--dim)":"var(--warn)";
  $("c-out").textContent=(s.emit&&s.emit.on)?"✓":"";
  $("c-dev").textContent=(s.role_defs||[]).every(([k])=>s.roles&&s.roles[k])?"✓":"";
  health(s);roles(s);devices(s);studio(s);axes(s);test(s);outp(s)};

// Attribution des roles. Un menu par role, alimente par les appareils vus.
// Un appareil deja pris est signale, et le serveur refuse : un appareil, un
// role. Un appareil attribue mais absent de la liste (debranche) reste dans
// le menu, en rouge, plutot que de disparaitre en silence.
function roles(s){
  const defs=s.role_defs||[],cur=s.roles||{},host=$("roles");
  if(host.dataset.built!=="1"){
    host.innerHTML="";host.dataset.built="1";
    for(const [k,lab] of defs){
      const row=el("div","row");row.style.cssText="align-items:center;gap:10px";
      const l=el("label",null,lab);l.style.cssText="flex:1;font-size:12px";
      const sel=el("select");sel.id="role-"+k;sel.style.minWidth="190px";
      sel.onchange=()=>go("set_role="+k+"&device="+encodeURIComponent(sel.value));
      row.appendChild(l);row.appendChild(sel);host.appendChild(row)}
  }
  for(const [k] of defs){
    const sel=$("role-"+k);if(!sel)continue;
    const want=cur[k]||"";
    const opts=[""].concat(s.devices.map(d=>d.id));
    if(want&&!opts.includes(want))opts.push(want);
    const sig=opts.join("|")+"#"+want+"#"+JSON.stringify(cur);
    if(sel.dataset.sig!==sig){
      sel.dataset.sig=sig;sel.innerHTML="";
      for(const id of opts){
        const o=el("option");o.value=id;
        const held=Object.entries(cur).find(([r,v])=>v===id&&r!==k);
        o.textContent=id?(held?id+"  (déjà "+held[0]+")":id):"— aucun —";
        if(id===want)o.selected=true;
        sel.appendChild(o)}
    }
    const vu=s.devices.some(d=>d.id===want);
    sel.style.color=want?(vu?"var(--ok)":"var(--bad)"):"var(--dim)";
    sel.title=want&&!vu?"attribué mais pas vu — débranché ?":"";
  }
  const w=$("survey-who");
  if(w){w.textContent=cur.survey||"aucun appareil de relevé";
        w.style.color=cur.survey?"var(--ok)":"var(--warn)"}
}

// Verdicts figes au phase_stop. Le seuil est celui de l'onglet : 0,3 % de
// la course est bon, 1 % passable, au-dela il faut refaire.
function results(s){
  const R=s.results||{}, names=Object.keys(R);
  const tb=$("tres");
  if(!names.length){
    tb.innerHTML='<tr><td class="note">Aucune phase terminée.</td></tr>';return}
  let h='<tr class="eyebrow"><td>phase</td><td>axe</td>'
       +'<td style="text-align:right">naïf</td>'
       +'<td style="text-align:right">aligné</td>'
       +'<td style="text-align:right">counts</td><td></td></tr>';
  for(const ph of names){
    const r=R[ph];
    for(const ax in r.axes){
      const v=r.axes[ax];
      const cls=v.pct<0.3?"v-OK":(v.pct<1.0?"v-PASSABLE":"v-REFAIRE");
      const lab=v.pct<0.3?"OK":(v.pct<1.0?"PASSABLE":"REFAIRE");
      h+='<tr><td>'+ph+'</td><td class="dim">'+ax+'</td>'
        +'<td class="mono" style="text-align:right;color:var(--naif)">'
          +v.peak_n.toFixed(3)+'°</td>'
        +'<td class="mono" style="text-align:right;color:var(--aligne)">'
          +v.peak_a.toFixed(3)+'°</td>'
        +'<td class="mono" style="text-align:right">'+v.counts+'</td>'
        +'<td class="'+cls+'">'+lab+'</td></tr>';
    }
  }
  tb.innerHTML=h;
}

// --- onglet Sortie -----------------------------------------------------
function outp(s){
  const f=s.freed, box=$("oframe");
  if(!f){box.innerHTML='<p class="note">Pas de pose caméra. '
    +'Déclare le tracker caméra dans l\'onglet Objectifs.</p>'}
  else{
    const L=(k,v,u)=>'<tr><td>'+k+'</td><td class="mono big" '
      +'style="text-align:right">'+v+'</td><td class="dim">'+u+'</td></tr>';
    box.innerHTML='<table><tbody>'
      +L("pan",  f.pan.toFixed(3),  "°")
      +L("tilt", f.tilt.toFixed(3), "°")
      +L("roll", f.roll.toFixed(3), "°")
      +L("X", f.x.toFixed(1), "mm")
      +L("Y", f.y.toFixed(1), "mm")
      +L("Z", f.z.toFixed(1), "mm")
      +L("zoom",  f.zoom_ok ? f.zoom  : "—", f.zoom_ok ? "/65535" : "non calibré")
      +L("focus", f.focus_ok? f.focus : "—", f.focus_ok? "/65535" : "non calibré")
      +'</tbody></table>';
  }
  // Ce qui manque pour que la trame ait un sens.
  const miss=[];
  if(!s.camera) miss.push("le tracker caméra n'est pas déclaré (onglet Objectifs)");
  if(!f||!f.framed) miss.push("world.json absent : X/Y/Z sont en coordonnées "
    +"libsurvive brutes, pas dans le repère plateau (onglet Studio)");
  if(!s.axes||!s.axes.zoom)  miss.push("axe zoom non calibré : le champ reste à 0");
  if(!s.axes||!s.axes.focus) miss.push("axe focus non calibré : le champ reste à 0");
  $("oreq").innerHTML = miss.length
    ? "<ul style='margin:0;padding-left:18px'><li>"+miss.join("</li><li>")+"</li></ul>"
    : "<span style='color:var(--ok)'>Rien. La trame est complète.</span>";

  const e=s.emit||{};
  $("estat").innerHTML = e.on
    ? '<span style="color:var(--ok)">● émission vers '+e.to+'</span> — '
      +'<b class="mono">'+e.n+'</b> paquets, <b class="mono">'+e.hz.toFixed(0)
      +'</b> Hz'
    : '<span class="dim">● arrêtée</span>';
}

// Bandeau de sante. Le pire tracker commande : c'est lui qui gatera la
// mesure, et une moyenne le cacherait. Quand des roles sont attribues, on
// ne juge que les trackers en service — les controleurs poses au sol n'ont
// pas a declencher d'alarme.
function health(s){
  const H=s.health;if(!H)return;
  const box=$("hb");const c=[];
  const lamp=k=>'<span class="lamp" style="background:'+k+'"></span>';
  const ok="var(--ok)",wr="var(--warn)",bd="var(--bad)";

  // horloge — le §6bis en fait la condition de tout le reste
  c.push('<span class="chip'+(H.clock_ok?'':' warn2')+'">'
    +lamp(H.clock_ok?ok:wr)+(s.clock||"horloge inconnue")+'</span>');

  // trackers vus / en service
  const gone=H.n_gone||0;
  c.push('<span class="chip'+(H.n_dev?(gone?' alarm':''):' alarm')+'">'
    +lamp(H.n_dev?(gone?bd:ok):bd)
    +'<b>'+H.n_dev+'</b> appareils'
    +(gone?(' · <b>'+gone+'</b> DISPARU'+(gone>1?'S':'')):'')
    +(H.n_assigned?(' · <b>'+H.n_assigned+'</b> en service'):'')+'</span>');

  // debit du plus lent
  const rl=H.min_rate>=100?ok:(H.min_rate>=40?wr:bd);
  c.push('<span class="chip'+(rl===ok?'':(rl===wr?' warn2':' alarm'))+'">'
    +lamp(rl)+'débit min <b>'+H.min_rate.toFixed(0)+'</b> Hz</span>');

  // fraicheur de la pose la plus vieille
  const al=H.worst_age_ms<50?ok:(H.worst_age_ms<200?wr:bd);
  c.push('<span class="chip'+(al===ok?'':(al===wr?' warn2':' alarm'))+'">'
    +lamp(al)+'pose la + vieille <b>'+H.worst_age_ms.toFixed(0)+'</b> ms</span>');

  // decrochages : ce qui coute un tour sur un axe multi-tour
  const dl=H.drops===0?ok:bd;
  c.push('<span class="chip'+(dl===ok?'':' alarm')+'">'+lamp(dl)
    +'décrochages <b>'+H.drops+'</b>'
    +(H.worst_gap_ms>20?(' · trou '+H.worst_gap_ms.toFixed(0)+' ms'):'')
    +'</span>');

  // base stations lues dans la config libsurvive
  c.push('<span class="chip'+(H.lh_seen>=2?'':' warn2')+'">'
    +lamp(H.lh_seen>=2?ok:wr)+'<b>'+H.lh_seen+'</b> base stations</span>');

  // une calibration de demo sur le disque serait relue par le bridge
  if(H.demo_cal)
    c.push('<span class="chip alarm">'+lamp(bd)
      +'CALIBRATION DE DÉMO chargée — ne pas s\'y fier</span>');

  box.innerHTML=c.join("");
}

function fmtAge(ms){const s=ms/1000;
  if(s<90)return s.toFixed(0)+" s";
  if(s<5400)return (s/60).toFixed(0)+" min";
  return (s/3600).toFixed(1)+" h"}

function devices(s){
  const tb=$("devs");const sel=new Set([...tb.querySelectorAll("tr.sel")]
    .map(r=>r.dataset.id));
  tb.innerHTML="";
  for(const d of s.devices){
    const tr=el("tr");tr.dataset.id=d.id;
    if(sel.has(d.id))tr.className="sel";
    tr.style.cursor="pointer";
    tr.onclick=()=>tr.classList.toggle("sel");
    const c1=el("td");
    const dot=el("span","dot"+(d.gone?"":(d.age_ms<200?" ok":" warn")));
    if(d.gone)dot.style.background="var(--bad)";
    c1.appendChild(dot);c1.appendChild(document.createTextNode(d.id));
    tr.appendChild(c1);
    tr.appendChild(el("td","mono",d.role||"—"));
    // Un appareil qui a disparu reste LISTE : le voir s'effacer en silence
    // laisserait croire qu'il n'a jamais existe.
    if(d.gone){
      const g=el("td","mono","absent "+fmtAge(d.age_ms));
      g.style.color="var(--bad)";tr.appendChild(g);
      tr.style.opacity="0.65";
    }else{
      tr.appendChild(el("td","mono",d.travel.toFixed(1)+" m"));
    }
    tb.appendChild(tr)}
}

function studio(s){
  for(const k in s.slots)
    {const e=$("slot-"+k);if(e){
       const n=s.slots[k],src=(s.slot_src||{})[k];
       e.textContent=n;
       e.title=src?("relevé avec "+src):"";
       e.style.color=n?"var(--ok)":"var(--dim)"}}
  if(s.capture)$("msg").textContent="Relevé "+s.capture.slot+" — "
    +s.capture.left.toFixed(1)+" s";
  const r=s.world_report,w=s.world;
  if(!r||!w)return;
  const rows=[["Triangle",r.triangle[0].toFixed(2)+" × "+r.triangle[1].toFixed(2)+" m"],
   ["Incertitude normale",r.unc.toFixed(3)+"° (95 %)"],
   ["Normale orientée par",r.oriented],
   ["Largeur d'écran",w.screen_width_mm.toFixed(0)+" mm"],
   ["Caméra ← écran",w.camera_distance_mm.toFixed(0)+" mm"],
   ["Déport latéral",(w.camera_lateral_mm>=0?"+":"")+w.camera_lateral_mm.toFixed(0)+" mm"]];
  if(r.scale_err_mm!==null)rows.push(["Écart au ruban",
    (r.scale_err_mm>=0?"+":"")+r.scale_err_mm.toFixed(0)+" mm"]);
  let h=rows.map(([a,b])=>`<div class="kv"><span>${a}</span><b>${b}</b></div>`).join("");
  if(r.unc>0.15)h+=`<p class="note" style="color:var(--bad)">Au-delà de
   0,15°, l'horizon du décor penche visiblement. Écarte les points.</p>`;
  if(Math.abs(w.camera_lateral_mm)>30)h+=`<p class="note">La caméra n'est pas
   sur la médiane. Le repère reste juste — il est ancré sur l'écran — mais
   décale le trépied de ${(-w.camera_lateral_mm).toFixed(0)} mm.</p>`;
  if(r.lh){h+=`<h3 style="margin-top:14px">Base stations</h3>`;
   for(const k in r.lh.local){const v=r.lh.local[k];
    h+=`<div class="kv"><span>${k}</span><b>x ${v[0].toFixed(2)}  y ${v[1].toFixed(2)}  z ${v[2].toFixed(2)} m</b></div>`}
   h+=`<div class="kv"><span>Symétrie</span><b>${r.lh.symmetry_mm.toFixed(0)} mm`
     +(r.lh.opposed?"":" — MÊME CÔTÉ")+`</b></div>`}
  $("wrep").innerHTML=h;
}

function axes(s){
  // On liste les DEUX axes attendus, calibres ou non. Une table vide ne
  // disait pas qu'il faut repasser une seconde fois : le balayage se fait un
  // axe a la fois, focus puis zoom, en changeant le menu « Axe ».
  const tb=$("axlist");tb.innerHTML="";
  for(const k of ["focus","zoom"]){
    const a=s.axes[k],tr=el("tr");
    tr.appendChild(el("td",null,k));
    if(a){
      tr.appendChild(el("td","mono",a.device||"—"));
      tr.appendChild(el("td","mono",(a.span||0).toFixed(0)+"°"
        +(a.multiturn?" ⟳":"")));
      const ok=el("td");ok.style.color="var(--ok)";ok.textContent="✓";
      tr.appendChild(ok);
    }else{
      const td=el("td","dim","à calibrer");td.colSpan=3;
      tr.appendChild(td);
    }
    tb.appendChild(tr)}
  if(s.sweep){$("msg").textContent=s.sweep.name+" — "+s.sweep.n
    +" échantillons, "+s.sweep.s.toFixed(0)+" s"}
  const r=s.sweep_result;
  if(!r){return}
  const c=r.cal;
  let h=`<div class="big v-${r.verdict}">${r.verdict}</div>
   <p class="note">${r.why}</p>`
   +[["Course",c.span_deg.toFixed(0)+"°"],
     ["Planéité",c.planarity.toFixed(4)],
     ["RMS swing",c.rms_deg.toFixed(2)+"°"],
     ["Échantillons",c.samples],
     ["Bougé caméra",r.camera_motion_deg.toFixed(2)+"°"]]
    .map(([a,b])=>`<div class="kv"><span>${a}</span><b>${b}</b></div>`).join("");
  h+=c.span_deg<355
   ?`<p class="note" style="color:var(--ok)">Course sous 360° : cet axe est
     absolu au démarrage. Pas de homing, jamais.</p>`
   :`<p class="note" style="color:var(--warn)">Multi-tour. L'accumulateur
     déroule, mais un décrochage long peut coûter un tour.</p>`;
  if(r.camera_motion_deg>2)h+=`<p class="note" style="color:var(--bad)">La
   caméra a bougé de ${r.camera_motion_deg.toFixed(1)}°. Le slerp compense un
   décalage temporel, pas un déplacement. Trépied, et refais.</p>`;
  h+=`<div style="display:flex;gap:8px;margin-top:12px">
   <button class="go" onclick="go('sweep_save=1')">Enregistrer</button>
   <button onclick="go('sweep_save=1&invert=1')">Enregistrer inversé</button></div>`;
  $("swrep").innerHTML=h;
}

function test(s){
  document.querySelectorAll("#phases li").forEach(li=>{
    li.classList.toggle("on",li.dataset.name===s.phase);
    li.classList.toggle("done",s.done.includes(li.dataset.name))});
  push(camBuf,s.cam_rate);
  results(s);
  if(!s.test){return}
  const host=$("scopes");
  for(const axis in s.test.axes){
    if(!S[axis]){
      const d=el("div","card");
      const hd=el("div");hd.style.cssText="display:flex;align-items:baseline;gap:14px;margin-bottom:6px";
      hd.appendChild(el("span","eyebrow",axis+" — écart / repos"));
      const bn=el("b","mono"),ba=el("b","mono"),pk=el("span","eyebrow mono");
      bn.style.cssText="color:var(--naif);font-size:19px;font-weight:400;margin-left:auto";
      ba.style.cssText="color:var(--aligne);font-size:19px;font-weight:400";
      hd.appendChild(bn);hd.appendChild(ba);hd.appendChild(pk);
      d.appendChild(hd);
      const c=el("canvas");c.height=140;d.appendChild(c);
      host.appendChild(d);
      S[axis]={c:c,bn:bn,ba:ba,pk:pk,
        buf:{n:ring(N),a:ring(N)},span:s.test.axes[axis].span}}
    const v=s.test.axes[axis],sc=S[axis];
    push(sc.buf.n,v.n);push(sc.buf.a,v.a);
    sc.bn.textContent=(v.n>=0?"+":"")+v.n.toFixed(3)+"°";
    sc.ba.textContent=(v.a>=0?"+":"")+v.a.toFixed(3)+"°";
    sc.pk.textContent="crête "+v.peak_n.toFixed(2)+"° / "+v.peak_a.toFixed(2)+"°"}
  if(!$("scopes").querySelector(".camcard")){
    const d=el("div","card");d.className="card camcard";
    d.appendChild(el("div","eyebrow","vitesse angulaire caméra"));
    const c=el("canvas");c.height=90;c.id="camcv";d.appendChild(c);
    $("scopes").appendChild(d)}
  let leak=0,drift=0;
  for(const k in s.test.axes){leak=Math.max(leak,s.test.axes[k].counts);
    drift=Math.max(drift,s.test.axes[k].drift_mm)}
  $("tstat").innerHTML=[["Fuite",leak+" counts"],
    ["Exploitables",s.test.exact_pct.toFixed(1)+" %"],
    ["Dérive montage",drift.toFixed(1)+" mm"],
    ["Référence",s.ref_ok?"établie":"phase repos manquante"]]
   .map(([a,b])=>`<div class="kv"><span>${a}</span><b>${b}</b></div>`).join("");
}

// cv.height est la taille du TAMPON de rendu, pas la hauteur affichee. La
// version precedente faisait h=cv.height puis cv.height=h*r : sur un ecran
// retina (devicePixelRatio 2) le canvas se retrouvait avec un tampon de 660
// px et, faute de hauteur CSS, s'affichait a 660 px de haut au lieu de 330.
// D'ou une vue de dessus deux fois trop grande sur Mac, normale ailleurs.
// On memorise donc la hauteur voulue une fois, on la pose en CSS, et le
// tampon en est deduit.
function prep(cv){
 const r=devicePixelRatio||1;
 if(!cv.dataset.h)cv.dataset.h=cv.getAttribute("height")||cv.height;
 const h=+cv.dataset.h,w=cv.clientWidth;
 if(cv.style.height!==h+"px")cv.style.height=h+"px";
 const bw=Math.round(w*r),bh=Math.round(h*r);
 if(cv.width!==bw||cv.height!==bh){cv.width=bw;cv.height=bh}
 const g=cv.getContext("2d");g.setTransform(r,0,0,r,0,0);g.clearRect(0,0,w,h);
 return[g,w,h]}

function tick(){
  drawTop();drawSweep();
  for(const a in S)drawScope(S[a]);
  const cc=$("camcv");if(cc)drawCam(cc);
  requestAnimationFrame(tick)}

function drawTop(){
  const [g,w,h]=prep($("top"));
  const s=last;if(!s||!s.world){g.fillStyle="#8b929e";
   g.font='12px "Ubuntu Mono",monospace';
   g.fillText("en attente des trois points",12,22);return}
  const wd=s.world.screen_width_mm/1000,dc=s.world.camera_distance_mm/1000,
        lat=s.world.camera_lateral_mm/1000;
  const maxX=Math.max(dc*1.25,3),maxY=Math.max(wd*0.75,2);
  const k=Math.min((w-60)/maxX,(h-50)/(2*maxY));
  const ox=34,oy=h/2;
  const X=x=>ox+x*k, Y=y=>oy-y*k;
  // Grille au metre : donne l'echelle, qu'aucun chiffre ne remplace quand
  // on cherche a savoir si la camera a la place de reculer.
  g.strokeStyle="#23272e";g.lineWidth=1;
  for(let m=0;m<=Math.ceil(maxX);m++){
    g.beginPath();g.moveTo(X(m),Y(-maxY));g.lineTo(X(m),Y(maxY));g.stroke()}
  for(let m=-Math.ceil(maxY);m<=Math.ceil(maxY);m++){
    g.beginPath();g.moveTo(X(0),Y(m));g.lineTo(X(maxX),Y(m));g.stroke()}
  g.fillStyle="#4a515c";g.font='9px "Ubuntu Mono",monospace';
  for(let m=1;m<=Math.ceil(maxX);m++)g.fillText(m+"m",X(m)+2,Y(-maxY)-3);

  g.strokeStyle="#2b3038";g.beginPath();g.moveTo(X(0),Y(0));g.lineTo(w-8,Y(0));
  g.stroke();
  g.strokeStyle="#4a8f5a";g.lineWidth=5;g.beginPath();
  g.moveTo(X(0),Y(-wd/2));g.lineTo(X(0),Y(wd/2));g.stroke();
  g.fillStyle="#4fc3d9";g.beginPath();g.arc(X(dc),Y(lat),6,0,7);g.fill();
  g.strokeStyle="#4fc3d9";g.lineWidth=1;g.setLineDash([4,4]);
  g.beginPath();g.moveTo(X(0),Y(0));g.lineTo(X(dc),Y(lat));g.stroke();
  g.setLineDash([]);
  const lh=s.world_report&&s.world_report.lh;
  if(lh)for(const kk in lh.local){const v=lh.local[kk];
    g.fillStyle="#e05c5c";g.beginPath();g.arc(X(v[0]),Y(v[1]),5,0,7);g.fill();
    g.fillStyle="#8b929e";g.font='10px "Ubuntu Mono",monospace';
    g.fillText(kk+" "+v[2].toFixed(1)+"m",X(v[0])+8,Y(v[1])-6)}
  // Position VIVANTE des trackers, dans le repere plateau. La pastille
  // cyan ci-dessus est la camera au moment du releve ; celles-ci bougent.
  // Voir ou est reellement la camera pendant la phase Test evite de
  // confondre "le decouplage fuit" et "je suis sorti du volume".
  if(s.devices)for(const d of s.devices){
    if(!d.xy)continue;
    const col=d.role==="camera"?"#4fc3d9":(d.role?"#7fbf6a":"#5a616c");
    const r=d.role?5:3;
    g.fillStyle=col;g.beginPath();g.arc(X(d.xy[0]),Y(d.xy[1]),r,0,7);g.fill();
    if(d.role){
      g.fillStyle="#8b929e";g.font='10px "Ubuntu Mono",monospace';
      g.fillText(d.role+" "+d.xy[2].toFixed(2)+"m",
                 X(d.xy[0])+8,Y(d.xy[1])+11)}
  }

  g.fillStyle="#8b929e";g.font='11px "Ubuntu Mono",monospace';
  g.fillText("écran",X(0)-26,Y(wd/2)-8);
  g.fillText("caméra "+dc.toFixed(2)+" m",X(dc)-30,Y(lat)+20);
  g.fillText("+X →",w-56,Y(0)-8);
}

function drawSweep(){
  const [g,w,h]=prep($("sweep"));
  const s=last;
  if(s&&s.sweep)push(sweepBuf,s.sweep.n);
  const r=s&&s.sweep_result;
  g.fillStyle="#8b929e";g.font='11px "Ubuntu Mono",monospace';
  if(!s||(!s.sweep&&!r)){g.fillText("aucun balayage",12,20);return}
  if(s.sweep){g.fillText(s.sweep.n+" échantillons — balaie butée à butée",12,20);
    let mx=1;for(let i=0;i<sweepBuf.n;i++)mx=Math.max(mx,at(sweepBuf,i));
    g.strokeStyle="#4fc3d9";g.lineWidth=1.6;g.beginPath();
    const st=w/Math.max(1,N-1);
    for(let i=0;i<sweepBuf.n;i++){const x=w-(sweepBuf.n-1-i)*st,
      y=h-8-at(sweepBuf,i)/mx*(h-30);i?g.lineTo(x,y):g.moveTo(x,y)}
    g.stroke();return}
  const c=r.cal;
  g.fillText("course "+c.span_deg.toFixed(0)+"°  planéité "
    +c.planarity.toFixed(4)+"  rms "+c.rms_deg.toFixed(2)+"°",12,20);
  const mid=h/2,amp=(h/2-22);
  g.strokeStyle="#2b3038";g.beginPath();g.moveTo(0,mid);g.lineTo(w,mid);g.stroke();
  g.strokeStyle="#4fc3d9";g.lineWidth=2;g.beginPath();
  for(let i=0;i<=200;i++){const x=w*i/200,
    y=mid+amp*Math.cos(Math.PI*i/200);i?g.lineTo(x,y):g.moveTo(x,y)}
  g.stroke();
}

function drawScope(sc){
  const [g,w,h]=prep(sc.c);
  const full=sc.span*0.02,mid=h/2,k=(h/2-6)/full;
  g.fillStyle="rgba(127,191,106,.10)";
  g.fillRect(0,mid-sc.span*0.003*k,w,sc.span*0.006*k);
  g.fillStyle="rgba(226,160,63,.07)";
  g.fillRect(0,mid-sc.span*0.010*k,w,sc.span*0.020*k);
  g.strokeStyle="#4a515c";g.beginPath();g.moveTo(0,mid+.5);g.lineTo(w,mid+.5);
  g.stroke();
  tr(g,sc.buf.n,w,mid,k,"#e2a03f",1.2);tr(g,sc.buf.a,w,mid,k,"#4fc3d9",1.8);
  g.fillStyle="#8b929e";g.font='11px "Ubuntu Mono",monospace';
  g.fillText("±"+(sc.span*0.003).toFixed(2)+"° tolérance",7,14)}

function tr(g,r,w,mid,k,col,lw){if(!r.n)return;
 g.strokeStyle=col;g.lineWidth=lw;g.beginPath();
 const st=w/Math.max(1,N-1);
 for(let i=0;i<r.n;i++){const x=w-(r.n-1-i)*st,
  y=Math.max(1,Math.min(mid*2-1,mid-at(r,i)*k));i?g.lineTo(x,y):g.moveTo(x,y)}
 g.stroke()}

function drawCam(cv){const [g,w,h]=prep(cv);
 let mx=60;for(let i=0;i<camBuf.n;i++)mx=Math.max(mx,at(camBuf,i));
 g.strokeStyle="#c77dbb";g.lineWidth=1.4;g.beginPath();
 const st=w/Math.max(1,N-1);
 for(let i=0;i<camBuf.n;i++){const x=w-(camBuf.n-1-i)*st,
  y=h-at(camBuf,i)/mx*(h-10);i?g.lineTo(x,y):g.moveTo(x,y)}
 g.stroke();g.fillStyle="#8b929e";g.font='11px "Ubuntu Mono",monospace';
 g.fillText(mx.toFixed(0)+" °/s",7,14)}
</script>
"""


class Handler(BaseHTTPRequestHandler):
    hub = None
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):
        pass

    def _send(self, body, ctype="application/json"):
        b = body.encode() if isinstance(body, str) else body
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(b)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(b)

    def do_GET(self):
        path, _, query = self.path.partition("?")
        q = {}
        for part in query.split("&"):
            if "=" in part:
                k, v = part.split("=", 1)
                q[k] = unquote(v)
        h = Handler.hub

        if path in ("/", "/index.html"):
            return self._send(PAGE, "text/html; charset=utf-8")
        if path == "/config":
            return self._send(json.dumps({
                "slots": [[k, lab] for k, lab in SLOTS],
                "phases": [[n, d] for n, d in PHASES]}))
        if path == "/cmd":
            try:
                self._cmd(h, q)
            except Exception as e:                      # noqa: BLE001
                h.msg = "Erreur : %s" % e
            return self._send("{}")
        if path == "/report":
            return self._send(h.report(), "text/plain; charset=utf-8")
        if path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "keep-alive")
            self.end_headers()
            try:
                while True:
                    self.wfile.write(("data: %s\n\n"
                                      % json.dumps(h.snapshot())).encode())
                    self.wfile.flush()
                    time.sleep(0.05)
            except (BrokenPipeError, ConnectionResetError):
                return
        self.send_response(404)
        self.send_header("Content-Length", "0")
        self.end_headers()

    @staticmethod
    def _cmd(h, q):
        if "studio_capture" in q:
            h.studio_capture(q["studio_capture"], q.get("devices", "").split(","))
        elif "studio_solve" in q:
            h.studio_solve(float(q.get("floor_off") or 0),
                           float(q.get("screen_mm") or 0) or None)
        elif "studio_save" in q:
            h.studio_save()
        elif "set_camera" in q:
            h.set_camera(q["set_camera"])
        elif "set_role" in q:
            h.set_role(q["set_role"], q.get("device", ""))
        elif "sweep_start" in q:
            h.sweep_start(q["sweep_start"], q.get("device", ""))
        elif "sweep_stop" in q:
            h.sweep_stop()
        elif "sweep_save" in q:
            h.sweep_save(invert=q.get("invert") == "1")
        elif "test_arm" in q:
            h.test_arm()
        elif "phase" in q:
            h.phase_start(q["phase"])
        elif "phase_stop" in q:
            h.phase_stop()
        elif "emit_start" in q:
            h.emit_start(q.get("ehost") or "127.0.0.1", q.get("eport") or 40000)
        elif "emit_stop" in q:
            h.emit_stop()


def selftest():
    """Isole l'auto-test du disque, puis lance l'enchainement.

    L'onglet Objectifs se termine par un `sweep_save`, et l'auto-test doit
    l'exercer : c'est la derniere transition de la machine a etats. Mais il
    ecrit dans AXES_PATH, donc dans bridge/ — une calibration DEMO-CAM
    fabriquee, deposee juste avant la vraie calibration puisque le §9
    demande de lancer les auto-tests avant de toucher au materiel. Le bridge
    et la console la reliraient au demarrage.

    Les deux chemins pointent donc vers un repertoire temporaire, detruit a
    la sortie. Le Hub est construit APRES la bascule : sinon il precharge
    l'axes.json reel et l'auto-test ne dirait plus la meme chose selon
    l'etat de calibration de la machine.
    """
    global AXES_PATH, WORLD_PATH, ROLES_PATH
    saved = (AXES_PATH, WORLD_PATH, ROLES_PATH)
    tmp = tempfile.mkdtemp(prefix="vp-console-selftest-")
    AXES_PATH = os.path.join(tmp, "axes.json")
    WORLD_PATH = os.path.join(tmp, "world.json")
    ROLES_PATH = os.path.join(tmp, "roles.json")
    try:
        return _selftest_run()
    finally:
        AXES_PATH, WORLD_PATH, ROLES_PATH = saved
        shutil.rmtree(tmp, ignore_errors=True)


def _selftest_run():
    """Fait tourner les trois onglets de bout en bout, sans HTTP ni materiel.

    C'est le seul test qui exerce la MACHINE A ETATS du Hub : enchainement
    des trois releves, resolution du repere, balayage d'axe, phases du test.
    Les auto-tests des modules ne valident que leur math ; ici on verifie que
    la console les enchaine correctement.

    Dure une trentaine de secondes : la demo tourne en temps reel.
    """
    hub = Hub(demo=True)
    demo_poses(0.0, "libre")
    r, o = _DEMO_R
    hub.lighthouses = {"LH0": list(o + r @ np.array([2.1, -2.6, 2.45])),
                       "LH1": list(o + r @ np.array([2.1, 2.6, 2.45]))}
    threading.Thread(target=hub.loop, daemon=True).start()
    time.sleep(3.0)

    def wait(pred, limit=20.0):
        end = time.time() + limit
        while time.time() < end:
            if pred():
                return True
            time.sleep(0.1)
        return False

    print("horloge   : %s" % hub.clock.describe())
    assert hub.clock.scale is not None, "l'horloge aurait du se resoudre"

    # -- studio ---------------------------------------------------------
    for slot, dev in (("left", "DEMO-SURV1"), ("right", "DEMO-SURV2"),
                      ("camera", "DEMO-SURV3")):
        hub.studio_capture(slot, [dev], seconds=1.5)
        assert wait(lambda: hub.capture is None), "releve %s bloque" % slot
    hub.studio_solve(floor_offset_mm=31.0, screen_mm=4000.0)
    w = hub.world
    assert w, hub.msg
    print("studio    : ecran %.0f mm | camera %.0f mm | deport %+.0f mm | "
          "incertitude %.3f deg"
          % (w["screen_width_mm"], w["camera_distance_mm"],
             w["camera_lateral_mm"], hub.world_report["unc"]))
    assert abs(w["screen_width_mm"] - 4000.0) < 20.0
    assert abs(w["camera_lateral_mm"] - 60.0) < 20.0
    assert hub.world_report["lh"]["opposed"]

    # Deux points au lieu de trois : doit etre refuse.
    saved = hub.slots["camera"]
    hub.slots["camera"] = hub.slots["left"]
    hub.world = None
    hub.studio_solve()
    assert hub.world is None, "deux points colineaires auraient du etre refuses"
    print("garde     : trois points confondus -> refuse")
    hub.slots["camera"] = saved

    # Un point releve avec DEUX appareils poses a deux endroits : la moyenne
    # donnerait leur milieu, un point qui n'existe nulle part.
    hub.studio_capture("camera", ["DEMO-SURV3", "DEMO-SURV1"], seconds=1.5)
    assert wait(lambda: hub.capture is None), "releve a deux bloque"
    hub.world = None
    hub.studio_solve()
    assert hub.world is None, "deux appareils sur un point auraient du etre refuses"
    assert "appareils" in hub.msg, hub.msg
    print("garde     : un point releve avec deux appareils -> refuse")

    # On refait ce point proprement pour la suite de l'enchainement.
    hub.studio_capture("camera", ["DEMO-SURV3"], seconds=1.5)
    assert wait(lambda: hub.capture is None), "releve camera bloque"
    hub.studio_solve(floor_offset_mm=31.0, screen_mm=4000.0)
    assert hub.world, hub.msg

    # -- objectifs -------------------------------------------------------
    hub.set_role("camera", "DEMO-CAM")
    hub.set_role("zoom", "DEMO-ZOO")
    hub.set_role("focus", "DEMO-FOC")
    hub.set_role("survey", "DEMO-SURV1")
    assert hub.roles["camera"] == "DEMO-CAM", hub.roles
    # Un appareil, un role : le second appel doit etre refuse.
    hub.set_role("focus", "DEMO-ZOO")
    assert hub.roles["focus"] == "DEMO-FOC", hub.roles
    assert "role" in hub.msg, hub.msg
    print("garde     : un appareil pour deux roles -> refuse")
    time.sleep(0.5)
    for name, dev in (("focus", "DEMO-FOC"), ("zoom", "DEMO-ZOO")):
        hub.sweep_start(name, dev)
        time.sleep(6.0)
        hub.sweep_stop()
        rr = hub.sweep_result
        assert rr, hub.msg
        print("objectif  : %-6s %-9s course %.0f deg | planeite %.4f"
              % (name, rr["verdict"], rr["cal"]["span_deg"],
                 rr["cal"]["planarity"]))
        hub.sweep_save()
    assert set(hub.axes) == {"focus", "zoom"}

    hub.sweep_start("focus", "DEMO-CAM")
    assert hub.sweep is None, "la camera ne peut pas servir d'axe d'objectif"
    print("garde     : camera comme axe d'objectif -> refuse")

    # REMPLACEMENT D'UN TRACKER. Reassigner un role a un autre appareil doit
    # effacer la calibration qui en dependait : elle a ete relevee sur un
    # autre montage et donnerait des valeurs plausibles et fausses.
    assert set(hub.axes) == {"focus", "zoom"}
    hub.set_role("focus", "DEMO-SURV2")
    assert "focus" not in hub.axes, hub.axes
    assert "zoom" in hub.axes, "le zoom n'avait pas a bouger"
    print("garde     : axe reassigne -> sa calibration est effacee")

    # Remplacer la CAMERA fait tomber les deux axes : ils sont calibres en
    # relatif camera (cal["ref"] = conj(q_cam) * q_objectif).
    hub.set_role("focus", "DEMO-FOC")
    hub.set_role("camera", "DEMO-SURV3")
    assert hub.axes == {}, hub.axes
    print("garde     : camera remplacee -> les deux axes tombent")

    # On remet l'etat pour la suite de l'enchainement.
    hub.set_role("camera", "DEMO-CAM")
    for name, dev in (("focus", "DEMO-FOC"), ("zoom", "DEMO-ZOO")):
        hub.set_role(name, dev)
        hub.sweep_start(name)
        time.sleep(6.0)
        hub.sweep_stop()
        assert hub.sweep_result, hub.msg
        hub.sweep_save()
    assert set(hub.axes) == {"focus", "zoom"}

    # -- test -------------------------------------------------------------
    hub.test_arm()
    assert hub.tap
    hub.phase_start("repos")
    time.sleep(3.0)
    hub.phase_stop()
    assert hub.ref, "la phase repos aurait du etablir la reference"

    out = {}
    for ph in ("panoramique", "roulis"):
        hub.phase_start(ph)
        time.sleep(6.0)
        snap = hub.snapshot()
        out[ph] = snap["test"]["axes"]
        hub.phase_stop()
        for a, v in out[ph].items():
            print("test %-11s %-6s crete naif %6.3f deg | aligne %6.3f deg | "
                  "%4d counts" % (ph, a, v["peak_n"], v["peak_a"], v["counts"]))

    # Le roulis est colineaire a l'axe des pignons : c'est la phase qui doit
    # reveler la fuite, et l'alignement doit l'annuler.
    roll = out["roulis"]["focus"]
    assert roll["peak_n"] > 0.2, "le roulis aurait du faire fuir la chaine naive"
    assert roll["peak_a"] < roll["peak_n"] / 10.0, roll
    hub.stop.set()

    print("\nOK — studio, objectifs et test enchaines ; gardes actives.")
    return 0


def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=8410)
    p.add_argument("--demo", action="store_true")
    p.add_argument("--selftest", action="store_true",
                   help="enchainer les trois onglets sans HTTP (~30 s)")
    args = p.parse_args()

    if args.selftest:
        return selftest()

    hub = Hub(demo=args.demo)
    if args.demo and not hub.lighthouses:
        r, o = None, None
        demo_poses(0.0, "libre")          # initialise le repere de demo
        r, o = _DEMO_R
        hub.lighthouses = {
            "LH0": list(o + r @ np.array([2.1, -2.6, 2.45])),
            "LH1": list(o + r @ np.array([2.1, 2.6, 2.45]))}

    Handler.hub = hub
    threading.Thread(target=hub.loop, daemon=True).start()
    srv = ThreadingHTTPServer((args.host, args.port), Handler)
    srv.daemon_threads = True
    print("Console sur http://%s:%d%s"
          % (args.host, args.port, "   [DEMO]" if args.demo else ""))
    print("Depuis le Mac :  ssh -L %d:localhost:%d unreal"
          % (args.port, args.port))
    if not args.demo:
        print("ARRETE LE BRIDGE : systemctl --user stop vp-bridge")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        hub.stop.set()
        print("\nArret.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
