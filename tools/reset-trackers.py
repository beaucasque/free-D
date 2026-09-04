#!/usr/bin/env python3
"""reset-trackers.py — reinitialise les trackers Vive, sans sudo.

Enveloppe en ligne de commande de bridge/usbreset.py. Arreter la console
avant : libsurvive est exclusif, et reinitialiser un appareil qu'il tient
ouvert le ferait decrocher inutilement.

    systemctl --user stop vp-console
    tools/reset-trackers.py                # tous
    tools/reset-trackers.py LHR-F3D3F946   # un seul
    systemctl --user start vp-console

La console le fait aussi toute seule : voir son chien de garde, qui
reinitialise un tracker en service devenu muet.
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "bridge"))
import usbreset  # noqa: E402

if __name__ == "__main__":
    found = usbreset.valve_devices()
    if not found:
        print("Aucun appareil Valve branche.")
        sys.exit(1)
    bad = 0
    for serial, msg in sorted(usbreset.reset(sys.argv[1:] or None).items()):
        print("%-16s %s" % (serial, msg or "reinitialise"))
        bad += msg is not None
    print("\n%d appareil(s). Laisser 5 s pour la re-enumeration, puis "
          "redemarrer la console." % len(found))
    sys.exit(1 if bad else 0)
