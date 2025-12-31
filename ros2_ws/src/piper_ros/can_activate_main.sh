#!/bin/bash
# -----------------------------------------------------------------------------
# CAN Interface Auto-Configuration Script
# This script automatically configures CAN interfaces based on their USB bus info.
# -----------------------------------------------------------------------------

# Check if ethtool is installed
if ! dpkg -l | grep -q "ethtool"; then
    echo "Error: ethtool not detected in the system."
    echo "Please install ethtool using the following command:"
    echo "sudo apt update && sudo apt install ethtool"
    exit 1
fi

# Check if can-utils is installed
if ! dpkg -l | grep -q "can-utils"; then
    echo "Error: can-utils not detected in the system."
    echo "Please install can-utils using the following command:"
    echo "sudo apt update && sudo apt install can-utils"
    exit 1
fi

echo "✅ Both ethtool and can-utils are installed."
echo "---"

# -----------------------------------------------------------------------------
# Configuration Section
# Define the default bitrate for all automatically detected CAN interfaces.
# -----------------------------------------------------------------------------
DEFAULT_BITRATE="1000000"

# -----------------------------------------------------------------------------
# Simplified logic: discover all CAN interfaces, rename sequentially to can0, can1...
# and bring them up with the desired bitrate. No interactive prompts.
# -----------------------------------------------------------------------------

echo "🔍 Discovering CAN interfaces..."
mapfile -t IFACES < <(ip -br link show type can | awk '{print $1}')

if [ "${#IFACES[@]}" -eq 0 ]; then
    echo "❌ No CAN interfaces were detected. Please check your hardware connections."
    exit 1
fi

echo "Found interfaces: ${IFACES[*]}"

# Build table of iface and bus-info to sort for deterministic naming
declare -A BUS_MAP
for iface in "${IFACES[@]}"; do
    BUS=$(ethtool -i "$iface" 2>/dev/null | grep "bus-info" | awk '{print $2}')
    BUS_MAP["$iface"]="$BUS"
done

# Sort by bus-info (fallback to iface name) for stable can0, can1, ...
IFS=$'\n' SORTED=($(for iface in "${IFACES[@]}"; do
    bus="${BUS_MAP[$iface]}"
    if [ -z "$bus" ]; then
        echo "zzz-$iface $iface"  # ensure driver-missing goes last
    else
        echo "$bus $iface"
    fi
done | sort | awk '{print $2}'))

INDEX=0
for iface in "${SORTED[@]}"; do
    TARGET_NAME="can${INDEX}"
    echo "--------------------------- $iface -> $TARGET_NAME ------------------------------"

    if ip link show "$TARGET_NAME" &>/dev/null && [ "$TARGET_NAME" != "$iface" ]; then
        echo "[INFO]: Bringing down existing $TARGET_NAME to allow renaming."
        ip link set "$TARGET_NAME" down || true
    fi

    if [ "$iface" != "$TARGET_NAME" ]; then
        if ip link set "$iface" down && ip link set "$iface" name "$TARGET_NAME"; then
            echo "[INFO]: Renamed '$iface' to '$TARGET_NAME'"
        else
            echo "[WARN]: Failed to rename '$iface' to '$TARGET_NAME', keeping original name."
            TARGET_NAME="$iface"
        fi
    else
        ip link set "$TARGET_NAME" down || true
    fi

    ip link set "$TARGET_NAME" type can bitrate "$DEFAULT_BITRATE" || true
    ip link set "$TARGET_NAME" up || true
    ip -details link show "$TARGET_NAME" | grep -E "bitrate|state|bus-info" || true

    INDEX=$((INDEX + 1))
done

echo "-----------------------------------------------------------------"
echo "[RESULT]: ✅ CAN interfaces configured (names: can0..can$((INDEX-1))) based on bus order."
