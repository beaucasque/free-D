"""
freed.py — Encodage du protocole Free-D (message D1).

Le message D1 fait 29 octets, big-endian, et transporte :
    - 3 angles  (pan, tilt, roll)
    - 3 positions (X, Y, Z)
    - 2 valeurs d'objectif (zoom, focus)

C'est exactement la forme de nos donnees : le tracker Vive fournit les six
premiers champs, les deux AS5600 fournissent les deux derniers. Un seul
paquet, pas de canal parallele a synchroniser.

FORMAT
    offset  taille  champ
    0       1       0xD1 (identifiant du message)
    1       1       identifiant de camera (0-255)
    2       3       pan    signe, LSB = 1/32768 degre
    5       3       tilt   signe, LSB = 1/32768 degre
    8       3       roll   signe, LSB = 1/32768 degre
    11      3       X      signe, LSB = 1/64 mm
    14      3       Y      signe, LSB = 1/64 mm
    17      3       Z      signe, LSB = 1/64 mm
    20      3       zoom   non signe
    23      3       focus  non signe
    26      2       reserve
    28      1       checksum

    checksum = (0x40 - somme des 28 premiers octets) mod 256

CONVENTION DE REPERE
    Free-D est traditionnellement en metres/degres avec Y vers l'avant.
    Unreal est gaucher, en centimetres, Z vers le haut.
    libsurvive sort du droitier Z-up en metres.

    La conversion libsurvive -> Free-D est faite ici dans survive_to_freed().
    Si l'orientation dans Unreal est miroir ou tournee de 90 degres, c'est
    ici qu'on ajuste — pas dans Unreal, ou les corrections se cumulent et
    deviennent impossibles a raisonner.
"""

import math
import socket
import struct

FREED_D1_SIZE = 29
FREED_D1_ID = 0xD1

ANGLE_LSB = 1.0 / 32768.0     # degres par comptage
POSITION_LSB = 1.0 / 64.0     # millimetres par comptage


def _pack_i24(value):
    """Entier signe 24 bits, big-endian, avec saturation."""
    value = int(round(value))
    value = max(-8388608, min(8388607, value))
    if value < 0:
        value += 0x1000000
    return bytes(((value >> 16) & 0xFF, (value >> 8) & 0xFF, value & 0xFF))


def _pack_u24(value):
    """Entier non signe 24 bits, big-endian, avec saturation."""
    value = int(round(value))
    value = max(0, min(0xFFFFFF, value))
    return bytes(((value >> 16) & 0xFF, (value >> 8) & 0xFF, value & 0xFF))


def encode_d1(pan_deg, tilt_deg, roll_deg,
              x_mm, y_mm, z_mm,
              zoom=0, focus=0,
              camera_id=1):
    """Construit un paquet Free-D D1 complet (29 octets)."""
    payload = bytearray()
    payload.append(FREED_D1_ID)
    payload.append(camera_id & 0xFF)

    payload += _pack_i24(pan_deg / ANGLE_LSB)
    payload += _pack_i24(tilt_deg / ANGLE_LSB)
    payload += _pack_i24(roll_deg / ANGLE_LSB)

    payload += _pack_i24(x_mm / POSITION_LSB)
    payload += _pack_i24(y_mm / POSITION_LSB)
    payload += _pack_i24(z_mm / POSITION_LSB)

    payload += _pack_u24(zoom)
    payload += _pack_u24(focus)

    payload += b"\x00\x00"                      # reserve

    checksum = (0x40 - sum(payload)) & 0xFF
    payload.append(checksum)

    assert len(payload) == FREED_D1_SIZE, len(payload)
    return bytes(payload)


def decode_d1(packet):
    """Decode un paquet D1. Utile pour verifier ce qu'on emet vraiment."""
    if len(packet) != FREED_D1_SIZE:
        raise ValueError("taille %d, attendu %d" % (len(packet), FREED_D1_SIZE))
    if packet[0] != FREED_D1_ID:
        raise ValueError("identifiant 0x%02X, attendu 0xD1" % packet[0])

    expected = (0x40 - sum(packet[:28])) & 0xFF
    if packet[28] != expected:
        raise ValueError("checksum 0x%02X, attendu 0x%02X" % (packet[28], expected))

    def i24(off):
        v = (packet[off] << 16) | (packet[off + 1] << 8) | packet[off + 2]
        return v - 0x1000000 if v & 0x800000 else v

    def u24(off):
        return (packet[off] << 16) | (packet[off + 1] << 8) | packet[off + 2]

    return {
        "camera_id": packet[1],
        "pan": i24(2) * ANGLE_LSB,
        "tilt": i24(5) * ANGLE_LSB,
        "roll": i24(8) * ANGLE_LSB,
        "x": i24(11) * POSITION_LSB,
        "y": i24(14) * POSITION_LSB,
        "z": i24(17) * POSITION_LSB,
        "zoom": u24(20),
        "focus": u24(23),
    }


def quat_to_pan_tilt_roll(w, x, y, z):
    """Quaternion -> angles Free-D en degres.

    libsurvive sort un quaternion (w, x, y, z) en repere droitier Z-up.
    Free-D attend pan (lacet), tilt (tangage), roll (roulis).

    Si tes axes sortent inverses dans Unreal, c'est ici qu'il faut agir :
    change le signe ou permute deux composantes, puis revalide. Ne compense
    jamais cote Unreal.
    """
    # lacet autour de Z
    pan = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    # tangage autour de Y, avec protection du gimbal lock
    sin_tilt = 2.0 * (w * y - z * x)
    sin_tilt = max(-1.0, min(1.0, sin_tilt))
    tilt = math.asin(sin_tilt)
    # roulis autour de X
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))

    return math.degrees(pan), math.degrees(tilt), math.degrees(roll)


def survive_to_freed(pose, zoom=0, focus=0, camera_id=1):
    """Convertit une pose libsurvive en paquet Free-D.

    `pose` : (x, y, z, qw, qx, qy, qz) en metres, repere droitier Z-up.
    """
    px, py, pz, qw, qx, qy, qz = pose
    pan, tilt, roll = quat_to_pan_tilt_roll(qw, qx, qy, qz)
    return encode_d1(pan, tilt, roll,
                     px * 1000.0, py * 1000.0, pz * 1000.0,
                     zoom=zoom, focus=focus, camera_id=camera_id)


class FreeDSender:
    """Emetteur UDP. Unreal ecoute par defaut sur le port 40000."""

    def __init__(self, host="127.0.0.1", port=40000):
        self.addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sent = 0

    def send(self, packet):
        self.sock.sendto(packet, self.addr)
        self.sent += 1

    def close(self):
        self.sock.close()


if __name__ == "__main__":
    # Auto-test : encodage puis decodage, on verifie qu'on retrouve nos valeurs.
    cases = [
        (0, 0, 0, 0, 0, 0, 0, 0),
        (45.5, -12.25, 3.125, 1000.0, -2500.0, 1750.0, 4096, 2048),
        (-179.9, 89.9, -89.9, -8000.0, 8000.0, 0.0, 0xFFFFFF, 0),
    ]
    for c in cases:
        pkt = encode_d1(*c)
        got = decode_d1(pkt)
        print("%d octets  pan=%.4f tilt=%.4f roll=%.4f  x=%.2f y=%.2f z=%.2f  zoom=%d focus=%d"
              % (len(pkt), got["pan"], got["tilt"], got["roll"],
                 got["x"], got["y"], got["z"], got["zoom"], got["focus"]))
        assert abs(got["pan"] - c[0]) < 0.001
        assert abs(got["x"] - c[3]) < 0.02
    print("OK — encodage/decodage coherents, checksum valide.")
