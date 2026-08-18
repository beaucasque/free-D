#!/usr/bin/env bash
# Identifie le RP2040 et genere la regle udev correspondante.
set -euo pipefail

echo "=== Ports serie presents ==="
ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || echo "(aucun)"
echo

echo "=== Peripheriques RP2040 (VID 2e8a) ==="
for dev in /dev/ttyACM*; do
    [[ -e "$dev" ]] || continue
    VID=$(udevadm info -q property -n "$dev" | grep -m1 '^ID_VENDOR_ID=' | cut -d= -f2 || true)
    [[ "$VID" == "2e8a" ]] || continue
    SERIAL=$(udevadm info -q property -n "$dev" | grep -m1 '^ID_SERIAL_SHORT=' | cut -d= -f2 || true)
    MODEL=$(udevadm info -q property -n "$dev" | grep -m1 '^ID_MODEL=' | cut -d= -f2 || true)
    echo "$dev  modele=$MODEL  serie=$SERIAL"
    echo
    echo "Regle udev a placer dans /etc/udev/rules.d/99-vp-encoders.rules :"
    echo "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"2e8a\", ATTRS{serial}==\"$SERIAL\", SYMLINK+=\"vp_encoders\", ENV{ID_MM_DEVICE_IGNORE}=\"1\", MODE=\"0660\", GROUP=\"dialout\""
done

echo
echo "=== Verifications ==="
id -nG | grep -qw dialout \
  && echo "OK  : membre du groupe dialout" \
  || echo "MANQUE : sudo usermod -aG dialout \$USER  (puis nouvelle session SSH)"

systemctl is-active --quiet ModemManager 2>/dev/null \
  && echo "ATTENTION : ModemManager est actif. Il sonde les nouveaux ports serie
            et corrompt les premieres secondes du flux. La regle udev ci-dessus
            l'en empeche (ID_MM_DEVICE_IGNORE)." \
  || echo "OK  : ModemManager inactif"
