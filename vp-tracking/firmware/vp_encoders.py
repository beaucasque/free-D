"""
vp_encoders.py — Double encodeur AS5600 pour tracking objectif (production virtuelle)

Cible   : Waveshare RP2040-Zero (ou tout RP2040) sous MicroPython
Role    : lire deux AS5600 (focus + zoom) montes sur les roulettes de deux
          follow focus, gerer l'accumulation multi-tour, et emettre les
          valeurs sur USB CDC pour le bridge libsurvive -> Free-D.

CABLAGE
    AS5600 #1 (FOCUS) -> i2c0        AS5600 #2 (ZOOM) -> i2c1
      VCC -> 3V3                       VCC -> 3V3
      GND -> GND                       GND -> GND
      SDA -> GP0                       SDA -> GP2
      SCL -> GP1                       SCL -> GP3
      DIR -> GND  (IMPERATIF)          DIR -> GND  (IMPERATIF)
      OUT, GPO : non connectes         OUT, GPO : non connectes

    Bouton -> GP4 vers GND (pull-up interne)
    LED WS2812 interne du RP2040-Zero sur GP16 (optionnelle)

PROTOCOLE SERIE (115200, USB CDC)
    Sortie continue :  E:F:<pos> Z:<pos> S:<flags>\n
        <pos>   position accumulee multi-tour, en comptes (4096 par tour)
        <flags> 2 caracteres, un par capteur : O=ok, W=champ faible,
                S=champ fort, X=absent, ?=erreur bus

    Commandes (un caractere, suivi de Entree ou non) :
        d  diagnostic complet (statut, AGC, magnitude, balayage guide)
        h  homing : remet les deux compteurs a zero a la position courante
        [  enregistre la butee BASSE (infini / grand angle)
        ]  enregistre la butee HAUTE (proche / longue focale)
        w  ecrit la calibration en flash
        r  recharge la calibration depuis la flash
        p  pause / reprise de l'emission continue
        ?  etat courant lisible

NOTE MICROPYTHON : le REPL partage le port USB CDC. Au boot et a chaque reset
tu verras du bruit dans le flux. Cote bridge Python, ignore toute ligne qui
ne commence pas par "E:".
"""

import json
import sys
import time

from machine import I2C, Pin

# --------------------------------------------------------------------------
# Configuration
# --------------------------------------------------------------------------

AS5600_ADDR = 0x36          # adresse figee, non modifiable -> deux bus distincts

REG_STATUS = 0x0B           # bit5 MD (detecte), bit4 ML (trop faible), bit3 MH (trop fort)
REG_RAW_ANGLE = 0x0C        # 12 bits, non filtre, non affecte par ZPOS/MPOS
REG_AGC = 0x1A              # 0-255 : ~128 ideal a 3V3
REG_MAGNITUDE = 0x1B        # amplitude du champ vue par le CORDIC

COUNTS_PER_TURN = 4096
HALF_TURN = COUNTS_PER_TURN // 2

OUTPUT_HZ = 200
OUTPUT_PERIOD_US = 1_000_000 // OUTPUT_HZ

CAL_PATH = "/vp_cal.json"
BUTTON_PIN = 4
LED_PIN = 16                # WS2812 interne du RP2040-Zero

# Duree d'appui long sur le bouton, en millisecondes
LONG_PRESS_MS = 800


# --------------------------------------------------------------------------
# Pilote AS5600
# --------------------------------------------------------------------------

class AS5600:
    """Un AS5600 sur son propre bus I2C, avec accumulation multi-tour.

    L'accumulation repose sur le fait que la liaison ne decroche jamais :
    tant qu'on echantillonne plus vite que la moitie d'un tour entre deux
    lectures, on sait toujours dans quel sens on a franchi le zero. A 200 Hz,
    il faudrait tourner la roulette a plus de 100 tours/seconde pour perdre
    le compte — ce qui n'arrivera pas sur un follow focus.
    """

    def __init__(self, bus, name):
        self.bus = bus
        self.name = name
        self.raw = 0            # derniere lecture brute 0-4095
        self.turns = 0          # nombre de tours accumules
        self.offset = 0         # zero logique, pose par le homing
        self.lo = None          # butee basse (position accumulee)
        self.hi = None          # butee haute
        self.status = "X"       # O / W / S / X / ?
        self.present = False
        self._prev_raw = None
        self._err_count = 0

    # -- acces bas niveau --------------------------------------------------

    def _read(self, reg, nbytes):
        return self.bus.readfrom_mem(AS5600_ADDR, reg, nbytes)

    def probe(self):
        """Verifie la presence du capteur sur le bus."""
        try:
            self.present = AS5600_ADDR in self.bus.scan()
        except Exception:
            self.present = False
        return self.present

    def read_status(self):
        """Retourne (md, ml, mh) — detection, champ faible, champ fort."""
        b = self._read(REG_STATUS, 1)[0]
        return bool(b & 0x20), bool(b & 0x10), bool(b & 0x08)

    def read_raw(self):
        """Angle brut 12 bits, 0-4095."""
        d = self._read(REG_RAW_ANGLE, 2)
        return ((d[0] << 8) | d[1]) & 0x0FFF

    def read_agc(self):
        return self._read(REG_AGC, 1)[0]

    def read_magnitude(self):
        d = self._read(REG_MAGNITUDE, 2)
        return ((d[0] << 8) | d[1]) & 0x0FFF

    # -- boucle de suivi ---------------------------------------------------

    def update(self):
        """Une lecture + mise a jour de l'accumulation. A appeler a cadence fixe."""
        try:
            raw = self.read_raw()
            md, ml, mh = self.read_status()
            self._err_count = 0
        except Exception:
            self._err_count += 1
            if self._err_count > 5:
                self.status = "?"
                self.present = False
            return

        if not md:
            self.status = "X"
        elif ml:
            self.status = "W"       # entrefer trop grand, ou aimant trop faible
        elif mh:
            self.status = "S"       # entrefer trop petit
        else:
            self.status = "O"

        if self._prev_raw is not None:
            delta = raw - self._prev_raw
            if delta > HALF_TURN:
                self.turns -= 1     # franchissement 0 -> 4095 (sens negatif)
            elif delta < -HALF_TURN:
                self.turns += 1     # franchissement 4095 -> 0 (sens positif)

        self._prev_raw = raw
        self.raw = raw

    # -- position ----------------------------------------------------------

    def absolute(self):
        """Position accumulee brute, avant application du zero."""
        return self.turns * COUNTS_PER_TURN + self.raw

    def position(self):
        """Position accumulee relative au zero de homing."""
        return self.absolute() - self.offset

    def home(self):
        """Pose le zero a la position courante."""
        self.offset = self.absolute()

    def set_low(self):
        self.lo = self.position()

    def set_high(self):
        self.hi = self.position()

    def normalized(self):
        """0.0 a 1.0 sur la course calibree, ou None si pas calibre.

        Le bridge peut ignorer cette valeur et faire la conversion lui-meme ;
        elle est surtout utile pour verifier la calibration a l'oeil.
        """
        if self.lo is None or self.hi is None or self.hi == self.lo:
            return None
        v = (self.position() - self.lo) / (self.hi - self.lo)
        return max(0.0, min(1.0, v))


# --------------------------------------------------------------------------
# LED de statut (optionnelle)
# --------------------------------------------------------------------------

class StatusLed:
    """WS2812 interne. Silencieusement inactive si le module n'existe pas."""

    def __init__(self, pin):
        self.np = None
        try:
            import neopixel
            self.np = neopixel.NeoPixel(Pin(pin), 1)
        except Exception:
            pass

    def set(self, r, g, b):
        if self.np:
            try:
                self.np[0] = (r, g, b)
                self.np.write()
            except Exception:
                pass


def led_color_for(sensors):
    """Vert = tout va bien, jaune = champ hors plage, rouge = capteur absent."""
    states = [s.status for s in sensors]
    if any(s in ("X", "?") for s in states):
        return (24, 0, 0)
    if any(s in ("W", "S") for s in states):
        return (20, 12, 0)
    return (0, 16, 0)


# --------------------------------------------------------------------------
# Calibration persistante
# --------------------------------------------------------------------------

def save_cal(sensors):
    data = {}
    for s in sensors:
        data[s.name] = {"offset": s.offset, "lo": s.lo, "hi": s.hi}
    try:
        with open(CAL_PATH, "w") as f:
            json.dump(data, f)
        return True
    except Exception as e:
        print("# erreur ecriture calibration:", e)
        return False


def load_cal(sensors):
    try:
        with open(CAL_PATH) as f:
            data = json.load(f)
    except Exception:
        return False

    for s in sensors:
        d = data.get(s.name)
        if d:
            # L'offset stocke n'a de sens que si la position absolue au boot
            # est identique — ce qui n'est PAS garanti sur un multi-tour.
            # On recharge donc lo/hi (la course) mais on redemande un homing.
            s.lo = d.get("lo")
            s.hi = d.get("hi")
    return True


# --------------------------------------------------------------------------
# Diagnostic
# --------------------------------------------------------------------------

def diagnose(sensors):
    """Verifie l'aimant et le montage. C'est la routine a lancer en premier."""
    print()
    print("=" * 62)
    print("DIAGNOSTIC AS5600")
    print("=" * 62)

    for s in sensors:
        print()
        print("--- %s ---" % s.name)

        if not s.probe():
            print("  ABSENT du bus. Verifie : alimentation 3V3, SDA/SCL non")
            print("  inverses, pull-ups presentes, soudures.")
            continue

        try:
            md, ml, mh = s.read_status()
            agc = s.read_agc()
            mag = s.read_magnitude()
        except Exception as e:
            print("  Erreur de lecture:", e)
            continue

        print("  MD (aimant detecte) : %s" % ("oui" if md else "NON"))
        print("  ML (champ faible)   : %s" % ("OUI" if ml else "non"))
        print("  MH (champ fort)     : %s" % ("OUI" if mh else "non"))
        print("  AGC                 : %d   (viser ~128 ; 0 ou 255 = hors plage)" % agc)
        print("  Magnitude           : %d" % mag)

        if not md:
            print("  >> Aucun aimant vu. Rapproche-le, ou il est trop faible.")
        elif ml:
            print("  >> Entrefer TROP GRAND. Rapproche l'aimant du boitier.")
        elif mh:
            print("  >> Entrefer TROP PETIT. Eloigne l'aimant.")
        else:
            print("  >> Champ correct.")

    print()
    print("-" * 62)
    print("TEST D'AIMANTATION — le point critique.")
    print("Tourne LENTEMENT chaque roulette d'un tour complet.")
    print("Attendu : une rampe reguliere de 0 a 4095, sans saut ni palier.")
    print("Si la valeur reste bloquee, saute, ou ne balaie qu'une fraction")
    print("de la plage, l'aimant est aimante AXIALEMENT -> inutilisable.")
    print("Ctrl-C pour sortir.")
    print("-" * 62)

    try:
        while True:
            line = []
            for s in sensors:
                if s.present:
                    try:
                        line.append("%s=%4d" % (s.name, s.read_raw()))
                    except Exception:
                        line.append("%s=ERR " % s.name)
                else:
                    line.append("%s=---- " % s.name)
            print("  " + "   ".join(line))
            time.sleep_ms(100)
    except KeyboardInterrupt:
        print()
        print("Diagnostic termine.")
        print("=" * 62)


# --------------------------------------------------------------------------
# Lecture non bloquante de l'entree serie
# --------------------------------------------------------------------------

class SerialInput:
    def __init__(self):
        try:
            import uselect
            self._poll = uselect.poll()
            self._poll.register(sys.stdin, uselect.POLLIN)
            self._ok = True
        except Exception:
            self._ok = False

    def read_char(self):
        if not self._ok:
            return None
        if self._poll.poll(0):
            c = sys.stdin.read(1)
            if c and c not in ("\r", "\n"):
                return c.lower()
        return None


# --------------------------------------------------------------------------
# Programme principal
# --------------------------------------------------------------------------

def main():
    i2c0 = I2C(0, scl=Pin(1), sda=Pin(0), freq=400_000)
    i2c1 = I2C(1, scl=Pin(3), sda=Pin(2), freq=400_000)

    focus = AS5600(i2c0, "F")
    zoom = AS5600(i2c1, "Z")
    sensors = (focus, zoom)

    button = Pin(BUTTON_PIN, Pin.IN, Pin.PULL_UP)
    led = StatusLed(LED_PIN)
    serial = SerialInput()

    led.set(0, 0, 20)
    print("# vp_encoders — demarrage")

    for s in sensors:
        s.probe()
        print("# capteur %s : %s" % (s.name, "present" if s.present else "ABSENT"))

    if load_cal(sensors):
        print("# calibration rechargee (course seulement)")
    print("# 'd' diagnostic  'h' homing  '[' ']' butees  'w' sauver  '?' etat")

    # Bouton maintenu au boot -> diagnostic direct
    if button.value() == 0:
        time.sleep_ms(50)
        if button.value() == 0:
            diagnose(sensors)

    # Amorcage : une premiere lecture pour initialiser _prev_raw sans
    # generer un faux franchissement de zero.
    for s in sensors:
        if s.present:
            s.update()

    emitting = True
    next_out = time.ticks_us()
    next_led = time.ticks_ms()
    btn_down_at = None

    while True:
        # --- acquisition ---
        for s in sensors:
            if s.present:
                s.update()

        # --- bouton : appui court = homing, appui long = butee basse ---
        if button.value() == 0:
            if btn_down_at is None:
                btn_down_at = time.ticks_ms()
        else:
            if btn_down_at is not None:
                held = time.ticks_diff(time.ticks_ms(), btn_down_at)
                if held >= LONG_PRESS_MS:
                    for s in sensors:
                        s.set_low()
                    print("# butee BASSE enregistree")
                elif held > 30:
                    for s in sensors:
                        s.home()
                    print("# homing effectue")
                btn_down_at = None

        # --- commandes serie ---
        c = serial.read_char()
        if c == "d":
            diagnose(sensors)
            for s in sensors:
                s._prev_raw = None      # on rearme proprement l'accumulation
                if s.present:
                    s.update()
        elif c == "h":
            for s in sensors:
                s.home()
            print("# homing effectue")
        elif c == "[":
            for s in sensors:
                s.set_low()
            print("# butee BASSE enregistree")
        elif c == "]":
            for s in sensors:
                s.set_high()
            print("# butee HAUTE enregistree")
        elif c == "w":
            print("# calibration sauvee" if save_cal(sensors) else "# echec sauvegarde")
        elif c == "r":
            print("# calibration rechargee" if load_cal(sensors) else "# aucun fichier")
        elif c == "p":
            emitting = not emitting
            print("# emission %s" % ("active" if emitting else "en pause"))
        elif c == "?":
            for s in sensors:
                n = s.normalized()
                print("# %s pos=%d raw=%d tours=%d lo=%s hi=%s norm=%s statut=%s"
                      % (s.name, s.position(), s.raw, s.turns, s.lo, s.hi,
                         "%.3f" % n if n is not None else "-", s.status))

        # --- emission a cadence fixe ---
        now = time.ticks_us()
        if emitting and time.ticks_diff(now, next_out) >= 0:
            next_out = time.ticks_add(next_out, OUTPUT_PERIOD_US)
            # Si on a pris du retard, on resynchronise plutot que de rattraper
            if time.ticks_diff(now, next_out) > OUTPUT_PERIOD_US:
                next_out = time.ticks_add(now, OUTPUT_PERIOD_US)
            print("E:F:%d Z:%d S:%s%s"
                  % (focus.position(), zoom.position(), focus.status, zoom.status))

        # --- LED de statut, rafraichie lentement ---
        if time.ticks_diff(time.ticks_ms(), next_led) >= 0:
            next_led = time.ticks_add(time.ticks_ms(), 250)
            r, g, b = led_color_for(sensors)
            led.set(r, g, b)


if __name__ == "__main__":
    main()
