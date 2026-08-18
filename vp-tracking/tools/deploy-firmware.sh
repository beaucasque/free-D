#!/usr/bin/env bash
# Deploie le firmware sur le RP2040 et relance la carte.
#
# ATTENTION : le RP2040 n'a qu'un seul port CDC. Si le bridge tourne, il
# tient le port et ce script echouera. On l'arrete donc d'abord.
set -euo pipefail

PORT="${PORT:-/dev/vp_encoders}"
FW="$(dirname "$0")/../firmware/vp_encoders.py"

command -v mpremote >/dev/null || { echo "mpremote absent : pip install mpremote"; exit 1; }
[[ -e "$PORT" ]] || { echo "Port $PORT introuvable. Lance tools/find-device.sh"; exit 1; }

BRIDGE_WAS_UP=0
if systemctl --user is-active --quiet vp-bridge.service 2>/dev/null; then
    echo "Arret du bridge (il occupe le port serie)..."
    systemctl --user stop vp-bridge.service
    BRIDGE_WAS_UP=1
    sleep 1
fi

echo "Deploiement de $(basename "$FW") vers $PORT..."
mpremote connect "$PORT" cp "$FW" :main.py
echo "Reset de la carte..."
mpremote connect "$PORT" reset
sleep 2

echo "Verification (3 s de flux) :"
timeout 3 cat "$PORT" | head -5 || true

if [[ $BRIDGE_WAS_UP -eq 1 ]]; then
    echo "Redemarrage du bridge..."
    systemctl --user start vp-bridge.service
fi
echo "Termine."
