#!/usr/bin/env python3
"""usbreset.py — reinitialiser un tracker Vive par logiciel.

L'equivalent exact d'un debranchement/rebranchement : l'ioctl
USBDEVFS_RESET. Sans sudo, parce que la regle udev 81-vive.rules posee par
install.sh met les noeuds /dev/bus/usb/... des appareils Valve en MODE 0666.

POURQUOI CA SERT. Un tracker peut rester dans un etat ou il n'ecoute plus
les balayages : libsurvive l'ouvre, lit son firmware, mais aucune ligne
LightcapMode n'apparait et il ne produit jamais de pose. Constate le
3 septembre 2026 sur trois trackers a la fois, pendant qu'un quatrieme
voyait parfaitement les stations depuis le meme endroit. Un reset les a tous
ramenes en 238-244 Hz.

LIBSURVIVE DOIT AVOIR LACHE L'APPAREIL. Reinitialiser un peripherique qu'il
tient ouvert provoque un decrochage inutile. Arreter la console d'abord.
"""

import fcntl
import glob
import os

USBDEVFS_RESET = ord("U") << 8 | 20          # _IO('U', 20)
VALVE = "28de"


def valve_devices():
    """[(serie, chemin sysfs, noeud /dev/bus/usb/...)] des appareils Valve."""
    out = []
    for d in sorted(glob.glob("/sys/bus/usb/devices/*/")):
        try:
            if open(d + "idVendor").read().strip() != VALVE:
                continue
            serial = open(d + "serial").read().strip()
            bus = int(open(d + "busnum").read())
            dev = int(open(d + "devnum").read())
        except (OSError, ValueError):
            continue
        out.append((serial, d, "/dev/bus/usb/%03d/%03d" % (bus, dev)))
    return out


def reset(serials=None):
    """Reinitialise les appareils dont la serie est listee, ou tous.

    Retourne {serie: None si reussi, message d'erreur sinon}. Ne leve pas :
    l'appelant est souvent un chien de garde qui doit continuer a tourner
    quoi qu'il arrive.
    """
    want = set(serials) if serials else None
    res = {}
    for serial, _sysfs, node in valve_devices():
        if want is not None and serial not in want:
            continue
        try:
            fd = os.open(node, os.O_WRONLY)
        except OSError as e:
            res[serial] = "noeud inaccessible (%s) — regle udev posee ?" % e.strerror
            continue
        try:
            fcntl.ioctl(fd, USBDEVFS_RESET, 0)
            res[serial] = None
        except OSError as e:
            res[serial] = e.strerror
        finally:
            os.close(fd)
    for s in (want or set()) - set(res):
        res[s] = "non branche"
    return res


if __name__ == "__main__":
    import sys
    found = valve_devices()
    if not found:
        print("Aucun appareil Valve branche.")
        sys.exit(1)
    bad = 0
    for serial, msg in sorted(reset(sys.argv[1:] or None).items()):
        print("%-16s %s" % (serial, msg or "reinitialise"))
        bad += msg is not None
    print("\n%d appareil(s). Laisser 5 s pour la re-enumeration, puis "
          "redemarrer la console." % len(found))
    sys.exit(1 if bad else 0)
