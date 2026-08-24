"""
survive_clock.py — Decouvre l'horloge de libsurvive au lieu de la supposer.

LE PROBLEME

Horodater une pose a l'instant ou Python la lit dans la file est faux. Entre
le moment ou libsurvive resout une pose et le moment ou notre boucle la
draine, il s'ecoule un delai variable — periode de boucle, ordonnancement,
taille de la file. Ce delai n'est pas le meme pour deux trackers draines dans
la meme rafale, et c'est exactement l'ecart que le slerp de CameraHistory est
cense annuler. Le mesurer avec time.monotonic() revient a mesurer une regle
avec elle-meme.

L'INDICE ETAIT DANS LE CODE DEPUIS LE DEBUT

    pose = u.Pose()[0]

On prend l'element [0] d'un tuple : il y en a d'autres.
survive_simple_object_get_latest_pose() a pour signature

    SurvivePose f(const SurviveSimpleObject *, FLT *timecode)

Pose()[1] est donc tres probablement l'horodatage propre de libsurvive.

CE QU'ON NE SAIT PAS, ET QU'ON NE DEVINE PAS

Son unite. Secondes flottantes ? Tics 48 MHz du Watchman ? Millisecondes ?
Son origine ? Selon la version de libsurvive et le pilote, ce n'est pas la
meme chose.

Alors on ne devine pas : on MESURE.

  1. On collecte des couples (brut, monotonic au drain).
  2. La pente se prend par mediane de pentes appariees — pas par moindres
     carres. Le bruit de drain n'est pas symetrique : il ne fait qu'AJOUTER
     du retard, jamais en retirer. Les moindres carres se laisseraient tirer
     par la queue de la distribution ; une mediane non.
  3. Si la pente tombe a 2 % pres d'une unite connue, on s'y accroche. Une
     pente empirique legerement fausse derive avec le temps ; 1/48e6 exact ne
     derive pas.
  4. L'offset se cale sur le PERCENTILE BAS des residus, pas sur leur
     moyenne. Le drain ne peut qu'ajouter du retard : la verite est au bord
     inferieur du nuage. La moyenne donnerait la latence typique, or ce
     qu'on veut est le plancher.
  5. Si rien ne tient, on retombe sur monotonic EN LE DISANT.

CE QUE CA NE PEUT PAS RESOUDRE

Si libsurvive horodate une pose au moment ou il l'a RESOLUE plutot qu'au
moment ou les photodiodes ont ete balayees, il reste un biais que cette
regression ne verra pas : elle compare deux horloges, elle ne sait pas ce que
l'horodatage pretend representer. Seul un test physique le revele — deux
trackers rigidement solidaires, secoues ensemble, dont la rotation relative
doit rester nulle. C'est l'onglet Test.

Ce module retire le retard de FILE. Il ne certifie pas la semantique.
"""

import math
import time

import numpy as np

# Unites plausibles, et leur nom. Si la pente mesuree tombe pres de l'une
# d'elles, on prend la valeur exacte plutot que la valeur mesuree.
KNOWN = [
    (1.0, "secondes"),
    (1e-3, "millisecondes"),
    (1e-6, "microsecondes"),
    (1.0 / 48_000_000.0, "tics 48 MHz"),
    (1.0 / 32_768.0, "tics 32768 Hz"),
]

SNAP_TOL = 0.02          # 2 % : au-dela on garde la pente mesuree
MIN_SAMPLES = 240
MIN_SPAN_S = 2.0         # duree minimale d'observation avant de conclure


def read_timecode(obj):
    """Horodatage brut d'un objet pysurvive, ou None.

    On essaie Pose()[1] puis quelques accesseurs nommes, parce que le binding
    varie d'une version a l'autre. Aucun n'est garanti : c'est tout l'objet
    de ce module de le decouvrir plutot que de le presumer.
    """
    try:
        p = obj.Pose()
    except Exception:
        p = None
    if isinstance(p, (tuple, list)) and len(p) >= 2:
        v = p[1]
        if isinstance(v, (int, float)) and math.isfinite(v) and v != 0.0:
            return float(v)
    for attr in ("Timecode", "TimeCode", "LastTime", "Time"):
        fn = getattr(obj, attr, None)
        if fn is None:
            continue
        try:
            v = fn()
        except Exception:
            continue
        if isinstance(v, (int, float)) and math.isfinite(v) and v != 0.0:
            return float(v)
    return None


def robust_slope(raw, mono):
    """Pente par mediane de pentes appariees (Theil-Sen abrege).

    Chaque point est apparie a celui situe a la moitie de la fenetre : les
    paires sont largement separees, donc peu sensibles au bruit local, et la
    mediane ignore la queue de retard.
    """
    n = len(raw)
    h = n // 2
    dr = raw[h:] - raw[:n - h]
    dm = mono[h:] - mono[:n - h]
    ok = np.abs(dr) > 0
    if not ok.any():
        return None
    return float(np.median(dm[ok] / dr[ok]))


class SurviveClock:
    """Convertit l'horodatage libsurvive vers la base monotonic.

    Etat :
        "apprentissage"  pas encore assez de donnees
        "<unite>"        horloge resolue
        "monotonic"      pas d'horodatage exploitable, repli assume
    """

    def __init__(self, window=1200):
        self.window = window
        self.raw = []
        self.mono = []
        self.scale = None
        self.offset = None
        self.unit = None
        self.state = "apprentissage"
        self.latency_ms = (0.0, 0.0)     # plancher, p95
        self._floor = None
        self.absent = 0
        self._t0 = None

    # -- apprentissage -----------------------------------------------------

    def feed(self, raw, t_mono):
        if raw is None:
            self.absent += 1
            # Aucun horodatage expose : inutile d'attendre, on tranche.
            if self.absent > MIN_SAMPLES and not self.raw:
                self._fallback("aucun horodatage expose par pysurvive")
            return
        if self._t0 is None:
            self._t0 = t_mono
        self.raw.append(raw)
        self.mono.append(t_mono)
        if len(self.raw) > self.window:
            del self.raw[:len(self.raw) - self.window]
            del self.mono[:len(self.mono) - self.window]

    def ready(self):
        # La duree se compte depuis le PREMIER echantillon jamais vu, pas
        # depuis le debut de la fenetre glissante : avec plusieurs trackers a
        # cadence elevee la fenetre se remplit en moins d'une seconde et ne
        # couvrirait jamais la duree minimale.
        return (len(self.raw) >= MIN_SAMPLES and self._t0 is not None
                and (self.mono[-1] - self._t0) >= MIN_SPAN_S)

    def solve(self):
        """Tente de resoudre. Retourne True si l'horloge est utilisable."""
        if not self.ready():
            return False
        raw = np.asarray(self.raw, float)
        mono = np.asarray(self.mono, float)

        slope = robust_slope(raw, mono)
        if slope is None or not math.isfinite(slope) or slope <= 0:
            return self._fallback("pente non exploitable")

        unit = None
        for value, name in KNOWN:
            if abs(slope - value) <= SNAP_TOL * value:
                slope, unit = value, name
                break
        if unit is None:
            unit = "mesuree (%.3e s/tic)" % slope

        resid = mono - raw * slope
        lo = float(np.percentile(resid, 2))
        spread = float(np.percentile(resid, 95) - lo)

        # Un nuage de residus etale sur plus d'un quart de seconde ne decrit
        # pas un retard de file : c'est que la pente est fausse, ou que le
        # champ lu n'est pas un temps.
        if spread > 0.25:
            return self._fallback("residus etales sur %.0f ms" % (spread * 1e3))

        self.scale, self.offset, self.unit = slope, lo, unit
        self.state = unit
        self._floor = lo
        self.latency_ms = (0.0, spread * 1e3)
        return True

    def _fallback(self, why):
        self.scale = self.offset = None
        self.state = "monotonic"
        self.unit = why
        return False

    # -- usage -------------------------------------------------------------

    def to_mono(self, raw, t_drain):
        """Horodatage utilisable dans la base monotonic.

        Sans horloge resolue, on rend l'instant de drain : c'est le
        comportement d'avant, explicitement assume plutot que subi.
        """
        if self.scale is None or raw is None:
            return t_drain
        t = raw * self.scale + self.offset
        # Le suivi du plancher absorbe une derive lente entre les deux
        # horloges. On ne descend que par petits pas et on remonte encore
        # plus lentement : le plancher est une borne, pas une moyenne.
        d = t_drain - t
        if d < 0.0:
            self.offset += d
            self._floor = 0.0
            return t_drain
        if self._floor is None or d < self._floor:
            self._floor = d
        else:
            self._floor += (d - self._floor) * 1e-4
        return t

    def describe(self):
        if self.state == "apprentissage":
            return "horloge : apprentissage (%d echantillons)" % len(self.raw)
        if self.state == "monotonic" and not self.raw:
            return "horloge : monotonic — %s" % self.unit
        if self.state == "monotonic":
            return "horloge : monotonic — repli (%s)" % self.unit
        return ("horloge : %s, retard de file %.1f ms (p95)"
                % (self.unit, self.latency_ms[1]))


# ------------------------------------------------------------------ auto-test

if __name__ == "__main__":
    rng = np.random.default_rng(7)

    def simulate(scale, name, n=900, base=0.004, jit=0.012, skew_ms=3.0):
        """Deux trackers echantillonnes a skew_ms d'ecart, draines ensemble
        avec un retard variable et unilateral."""
        clk = SurviveClock()
        epoch = 12345.678
        t0 = time.monotonic()
        truth = []
        for i in range(n):
            t_true = t0 + i * 0.004
            # retard de file : plancher + queue exponentielle
            drain = t_true + base + rng.exponential(jit)
            for dev, off in (("cam", 0.0), ("lens", -skew_ms / 1000.0)):
                ts = t_true + off
                raw = (ts - epoch) / scale
                clk.feed(raw, drain)
                truth.append((dev, raw, drain, ts))
        ok = clk.solve()
        return clk, ok, truth, name

    print("%-16s %-22s %s" % ("SCENARIO", "ETAT", "ECART cam/lens retrouve"))
    print("-" * 74)

    for scale, name in [(1.0, "secondes"), (1.0 / 48e6, "tics 48 MHz"),
                        (1e-3, "millisecondes")]:
        clk, ok, truth, _ = simulate(scale, name)
        # Ce qui compte n'est pas l'heure absolue mais l'ECART entre deux
        # trackers draines ensemble : c'est lui que le slerp doit corriger.
        errs = []
        for i in range(0, len(truth) - 1, 2):
            dcam, rcam, drain, tcam = truth[i]
            dl, rl, drain2, tl = truth[i + 1]
            got = clk.to_mono(rcam, drain) - clk.to_mono(rl, drain2)
            errs.append(abs(got - (tcam - tl)) * 1e6)
        p95 = float(np.percentile(errs, 95))
        print("%-16s %-22s %.1f us (p95)" % (name, clk.state, p95))
        assert ok and clk.state == name, (name, clk.state)
        assert p95 < 50.0, (name, p95)

    # Comparaison : sans horloge, l'ecart est celui du drain, pas le vrai.
    clk, _ok, truth, _ = simulate(1.0, "secondes")
    naive = []
    for i in range(0, len(truth) - 1, 2):
        _d, _r, drain, tcam = truth[i]
        _d2, _r2, drain2, tl = truth[i + 1]
        naive.append(abs((drain - drain2) - (tcam - tl)) * 1e3)
    print("\nsans horloge   : ecart faux de %.1f ms (p95)"
          % float(np.percentile(naive, 95)))

    # Champ absent : repli assume, pas de plantage.
    clk = SurviveClock()
    t0 = time.monotonic()
    for i in range(400):
        clk.feed(None, t0 + i * 0.004)
    print("champ absent   : %s" % clk.describe())
    assert clk.to_mono(None, 42.0) == 42.0

    # Champ present mais qui n'est pas un temps : doit etre rejete.
    clk = SurviveClock()
    t0 = time.monotonic()
    for i in range(600):
        clk.feed(rng.uniform(0, 1000), t0 + i * 0.004)
    clk.solve()
    print("champ non-temps: %s" % clk.describe())
    assert clk.state == "monotonic"

    print("\nOK — unite detectee, offset cale sur le plancher, replis surs.")
