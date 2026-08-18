#!/usr/bin/env python3
"""
test-sweep.py — Verdict automatique sur un balayage d'encodeur.

Le probleme que resout cet outil : lire une colonne de nombres qui defile ne
dit pas si l'aimant est bon. Ce script capture pendant que tu tournes la
roulette, puis rend un diagnostic explicite.

C'est le test a lancer en premier, des reception des aimants.

USAGE
    ./test-sweep.py --port /dev/vp_encoders --duration 15
    ./test-sweep.py --port /dev/vp_encoders --sensor Z --duration 20

PROCEDURE
    Lance le script, puis tourne LENTEMENT et REGULIEREMENT la roulette
    concernee d'au moins un tour complet, dans un seul sens.
"""

import argparse
import sys
import time
from collections import Counter

COUNTS_PER_TURN = 4096
NBINS = 64                      # granularite de l'analyse de couverture


def capture(port, baud, duration, sensor):
    import serial

    samples = []
    statuses = Counter()
    buf = b""

    with serial.Serial(port, baud, timeout=0.1) as ser:
        ser.reset_input_buffer()
        print("Capture pendant %.0f s — tourne la roulette %s maintenant."
              % (duration, sensor))
        deadline = time.monotonic() + duration
        last_print = 0.0

        while time.monotonic() < deadline:
            buf += ser.read(4096)
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.decode("utf8", "replace").strip()
                if not text.startswith("E:"):
                    continue
                values = {}
                for token in text[2:].split():
                    key, _, value = token.partition(":")
                    values[key] = value
                if sensor in values:
                    try:
                        samples.append(int(values[sensor]))
                    except ValueError:
                        continue
                if "S" in values:
                    idx = 0 if sensor == "F" else 1
                    if len(values["S"]) > idx:
                        statuses[values["S"][idx]] += 1

            now = time.monotonic()
            if now - last_print > 1.0:
                last_print = now
                remaining = deadline - now
                print("  %4.0f s restantes — %d echantillons"
                      % (remaining, len(samples)), end="\r")

    print(" " * 60, end="\r")
    return samples, statuses


def analyse(samples, statuses):
    print()
    print("=" * 64)
    print("RESULTAT")
    print("=" * 64)

    if len(samples) < 50:
        print("Trop peu d'echantillons (%d). La carte emet-elle ?" % len(samples))
        print("Verifie avec :  timeout 3 cat /dev/vp_encoders")
        return False

    print("Echantillons        : %d" % len(samples))

    # --- statut du champ magnetique ---
    total = sum(statuses.values()) or 1
    labels = {"O": "champ correct", "W": "CHAMP TROP FAIBLE",
              "S": "CHAMP TROP FORT", "X": "AIMANT NON DETECTE",
              "?": "ERREUR DE BUS"}
    print("Statut du capteur   :", end=" ")
    print(", ".join("%s %.0f%%" % (labels.get(k, k), 100.0 * v / total)
                    for k, v in statuses.most_common()))

    ok = True
    dominant = statuses.most_common(1)[0][0] if statuses else "?"
    if dominant == "W":
        print("  >> Rapproche l'aimant du boitier.")
        ok = False
    elif dominant == "S":
        print("  >> Eloigne l'aimant du boitier.")
        ok = False
    elif dominant in ("X", "?"):
        print("  >> Aimant absent ou liaison I2C defaillante.")
        return False

    # --- amplitude parcourue ---
    span = max(samples) - min(samples)
    turns = span / COUNTS_PER_TURN
    print("Course parcourue    : %d comptes (%.2f tour)" % (span, turns))

    if turns < 0.9:
        print("  >> Moins d'un tour complet balaye. Recommence en tournant plus.")
        print("     (Si tu AS tourne un tour entier, c'est le symptome n1 d'un")
        print("      aimant AXIAL : le capteur ne voit qu'une fraction de la plage.)")
        ok = False

    # --- couverture : un aimant axial laisse des trous ---
    within = [s % COUNTS_PER_TURN for s in samples]
    bins = Counter(v * NBINS // COUNTS_PER_TURN for v in within)
    empty = NBINS - len(bins)
    print("Couverture angulaire: %d/%d secteurs visites" % (len(bins), NBINS))
    if empty > NBINS // 8:
        print("  >> %d secteurs jamais atteints. Signature d'un aimant AXIAL" % empty)
        print("     ou mal centre. Refais le test de la tranche.")
        ok = False

    # --- monotonie : un balayage propre ne repart pas en arriere ---
    deltas = [b - a for a, b in zip(samples, samples[1:])]
    forward = sum(1 for d in deltas if d > 0)
    backward = sum(1 for d in deltas if d < 0)
    if forward and backward:
        minority = min(forward, backward) / (forward + backward)
        print("Regularite du sens  : %.1f%% d'inversions" % (100 * minority))
        if minority > 0.15:
            print("  >> Beaucoup d'allers-retours. Soit tu as change de sens,")
            print("     soit la lecture est bruitee (excentricite de l'aimant,")
            print("     support qui flechit, ou entrefer trop grand).")
            ok = False

    # --- sauts : detecte une perte de comptage multi-tour ---
    jumps = [d for d in deltas if abs(d) > COUNTS_PER_TURN // 4]
    if jumps:
        print("Sauts anormaux      : %d (max %d comptes)"
              % (len(jumps), max(abs(d) for d in jumps)))
        print("  >> Un saut de plus d'un quart de tour entre deux echantillons")
        print("     signale une lecture perdue. Baisse la vitesse de rotation")
        print("     ou verifie l'integrite de la liaison I2C.")
        ok = False
    else:
        print("Sauts anormaux      : aucun")

    # --- resolution effective ---
    tiny = sum(1 for d in deltas if d == 0)
    print("Echantillons figes  : %.1f%%" % (100.0 * tiny / max(1, len(deltas))))

    print("-" * 64)
    if ok:
        print("VERDICT : aimant et montage conformes. Tu peux avancer.")
    else:
        print("VERDICT : a corriger avant d'aller plus loin (voir ci-dessus).")
    print("=" * 64)
    return ok


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default="/dev/vp_encoders")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--duration", type=float, default=15.0)
    p.add_argument("--sensor", choices=("F", "Z"), default="F",
                   help="F = focus (i2c0), Z = zoom (i2c1)")
    args = p.parse_args()

    try:
        samples, statuses = capture(args.port, args.baud, args.duration, args.sensor)
    except Exception as e:
        print("Erreur d'ouverture de %s : %s" % (args.port, e), file=sys.stderr)
        return 1

    return 0 if analyse(samples, statuses) else 2


if __name__ == "__main__":
    sys.exit(main())
