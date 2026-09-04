#!/usr/bin/env python3
"""reset-trackers.py — reinitialise les trackers Vive par logiciel.

L'equivalent exact d'un debranchement/rebranchement, sans toucher au
materiel et sans redemarrer la station.

POURQUOI CA SERT

Un tracker peut rester dans un etat ou il n'ecoute plus les balayages des
base stations : libsurvive l'ouvre, lit son firmware, mais aucune ligne
LightcapMode n'apparait et il ne produit jamais de pose. Constate le
3 septembre 2026 sur trois trackers a la fois, alors qu'un quatrieme voyait
parfaitement les stations depuis le meme endroit. Un reset les a tous
ramenes en 238-244 Hz.

POURQUOI CA MARCHE SANS SUDO

La regle udev 81-vive.rules, posee par install.sh, met les noeuds
/dev/bus/usb/... des appareils Valve en MODE 0666. On peut donc y emettre
l'ioctl USBDEVFS_RESET sans privilege.

ARRETER LA CONSOLE D'ABORD : libsurvive est exclusif, et reinitialiser un
appareil qu'il tient ouvert le ferait decrocher inutilement.

    systemctl --user stop vp-console
    tools/reset-trackers.py
    systemctl --user start vp-console
"""

import fcntl
import glob
import os
import sys

USBDEVFS_RESET = ord("U") << 8 | 20          # _IO('U', 20)
VALVE = "28de"


def main():
    found = 0
    for d in sorted(glob.glob("/sys/bus/usb/devices/*/")):
        try:
            if open(d + "idVendor").read().strip() != VALVE:
                continue
            serial = open(d + "serial").read().strip()
            bus = int(open(d + "busnum").read())
            dev = int(open(d + "devnum").read())
        except (OSError, ValueError):
            continue

        found += 1
        node = "/dev/bus/usb/%03d/%03d" % (bus, dev)
        try:
            fd = os.open(node, os.O_WRONLY)
        except OSError as e:
            print("%-16s REFUSE : %s" % (serial, e.strerror))
            print("  la regle udev 81-vive.rules est-elle posee ? "
                  "voir install.sh")
            continue
        try:
            fcntl.ioctl(fd, USBDEVFS_RESET, 0)
            print("%-16s reinitialise" % serial)
        except OSError as e:
            print("%-16s ECHEC : %s" % (serial, e.strerror))
        finally:
            os.close(fd)

    if not found:
        print("Aucun appareil Valve branche.")
        return 1
    print("\n%d appareil(s). Laisser 5 s pour la re-enumeration, puis "
          "redemarrer la console." % found)
    return 0


if __name__ == "__main__":
    sys.exit(main())
