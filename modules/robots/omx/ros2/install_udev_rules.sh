#!/bin/bash
# Install udev rules for the OpenManipulator-X U2D2 serial bridge.
# This makes /dev/ttyUSB* readable/writable by non-root users and lowers
# the FTDI latency_timer so Dynamixel control loop jitter stays bounded.
#
# Run once after installing the OMX module. Idempotent.

set -e

RULES_SRC="$(dirname "$(readlink -f "$0")")/src/open_manipulator_bringup/open-manipulator-cdc.rules"
RULES_DST="/etc/udev/rules.d/99-open-manipulator-cdc.rules"

if [ ! -f "$RULES_SRC" ]; then
    echo "❌ Source rules file not found: $RULES_SRC"
    exit 1
fi

if [ "$(id -u)" -ne 0 ]; then
    SUDO="sudo"
else
    SUDO=""
fi

echo "📋 Installing udev rules: $RULES_SRC -> $RULES_DST"
$SUDO cp -f "$RULES_SRC" "$RULES_DST"
$SUDO udevadm control --reload-rules
$SUDO udevadm trigger

echo "✅ OpenManipulator-X udev rules installed."
echo "   Re-plug the U2D2 USB cable if a device was already connected."
