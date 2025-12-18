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
# Script Logic
# -----------------------------------------------------------------------------

# Parsing parameters
IGNORE_CHECK=false
for arg in "$@"; do
    if [ "$arg" == "--ignore" ]; then
        IGNORE_CHECK=true
    fi
done

# Step 1: Discover and map CAN interfaces dynamically
echo "🔍 Discovering CAN interfaces and mapping to USB ports..."
declare -A USB_PORTS
CURRENT_CAN_COUNT=0
CAN_INDEX=0

for iface in $(ip -br link show type can | awk '{print $1}'); do
    BUS_INFO=$(sudo ethtool -i "$iface" | grep "bus-info" | awk '{print $2}')
    if [ -n "$BUS_INFO" ]; then
        TARGET_NAME="can_$CAN_INDEX"
        USB_PORTS["$BUS_INFO"]="$TARGET_NAME:$DEFAULT_BITRATE"
        CAN_INDEX=$((CAN_INDEX + 1))
        CURRENT_CAN_COUNT=$((CURRENT_CAN_COUNT + 1))
    fi
done

if [ "$CURRENT_CAN_COUNT" -eq 0 ]; then
    echo "❌ No CAN interfaces were detected. Please check your hardware connections."
    exit 1
fi

echo "---"
echo "🔧 Automatically generated USB_PORTS configuration:"
for k in "${!USB_PORTS[@]}"; do
    echo "  \"$k\"=\"${USB_PORTS[$k]}\""
done
echo "---"

# Step 2: Perform CAN quantity check
PREDEFINED_COUNT=${#USB_PORTS[@]}

if [ "$IGNORE_CHECK" = false ] && [ "$CURRENT_CAN_COUNT" -ne "$PREDEFINED_COUNT" ]; then
    echo "[WARN]: The detected number of CAN modules ($CURRENT_CAN_COUNT) does not match the expected number ($PREDEFINED_COUNT)."
    read -p "Do you want to continue? (y/N): " user_input
    case "$user_input" in
        [yY]|[yY][eE][sS])
            echo "Continue execution..."
            ;;
        *)
            echo "Exited."
            exit 1
            ;;
    esac
else
    echo "CAN quantity check ignored or matched, continuing..."
fi

# Step 3: Configure CAN interfaces
SUCCESS_COUNT=0
FAILED_COUNT=0
declare -A USB_PORT_STATUS
for k in "${!USB_PORTS[@]}"; do
    USB_PORT_STATUS["$k"]="pending"
done

for iface in $(ip -br link show type can | awk '{print $1}'); do
    echo "--------------------------- $iface ------------------------------"
    BUS_INFO=$(sudo ethtool -i "$iface" | grep "bus-info" | awk '{print $2}')

    if [ -z "$BUS_INFO" ]; then
        echo "[ERROR]: Unable to get bus-info information for interface '$iface'."
        continue
    fi

    echo "[INFO]: System interface '$iface' is plugged into USB port '$BUS_INFO'"
    if [ -n "${USB_PORTS[$BUS_INFO]}" ]; then
        IFS=':' read -r TARGET_NAME TARGET_BITRATE <<< "${USB_PORTS[$BUS_INFO]}"

        IS_LINK_UP=$(ip link show "$iface" | grep -q "UP" && echo "yes" || "no")
        CURRENT_BITRATE=$(ip -details link show "$iface" | grep -oP 'bitrate \K\d+')

        if [ "$IS_LINK_UP" = "yes" ] && [ "$CURRENT_BITRATE" -eq "$TARGET_BITRATE" ]; then
            echo "[INFO]: Interface '$iface' is activated and bitrate is $TARGET_BITRATE"
            if [ "$iface" != "$TARGET_NAME" ]; then
                echo "[INFO]: Rename interface '$iface' to '$TARGET_NAME'"
                sudo ip link set "$iface" down
                sudo ip link set "$iface" name "$TARGET_NAME"
                sudo ip link set "$TARGET_NAME" up
                echo "[INFO]: The interface was renamed to '$TARGET_NAME' and reactivated."
            else
                echo "[INFO]: The USB port '$BUS_INFO' interface name is already '$TARGET_NAME'"
            fi
        else
            if ip link show "$TARGET_NAME" &>/dev/null; then
                echo "[WARN]: Cannot rename '$iface' to '$TARGET_NAME' because interface '$TARGET_NAME' already exists."
                echo "[HINT]: Please check if another interface already occupies this name, or fix your USB_PORTS configuration."
                continue
            fi
            
            if [ "$IS_LINK_UP" = "yes" ]; then
                echo "[INFO]: Interface '$iface' is activated, but the bitrate $CURRENT_BITRATE does not match the set $TARGET_BITRATE."
            else
                echo "[INFO]: Interface '$iface' is not activated or the bitrate is not set."
            fi

            sudo ip link set "$iface" down
            sudo ip link set "$iface" type can bitrate "$TARGET_BITRATE"
            sudo ip link set "$iface" up
            echo "[INFO]: Interface '$iface' has been reset to bitrate $TARGET_BITRATE and activated."

            if [ "$iface" != "$TARGET_NAME" ]; then
                echo "[INFO]: Rename interface $iface to '$TARGET_NAME'"
                sudo ip link set "$iface" down
                sudo ip link set "$iface" name "$TARGET_NAME"
                sudo ip link set "$TARGET_NAME" up
                echo "[INFO]: The interface was renamed to '$TARGET_NAME' and reactivated."
            fi
        fi
        SUCCESS_COUNT=$((SUCCESS_COUNT+1))
        USB_PORT_STATUS["$BUS_INFO"]="success"
    else
        echo "[ERROR]: The USB port '$BUS_INFO' of interface '$iface' was not found in the predefined USB_PORTS list."
        echo "[INFO]: Current predefined USB_PORTS configuration:"
        for k in "${!USB_PORTS[@]}"; do
            echo "        '$k'"
        done
        echo "[HINT]: Please check if the USB device is inserted into the correct port, or update the USB_PORTS config if needed."
    fi
    echo "-----------------------------------------------------------------"
done

# Step 4: Final Summary
echo "---"
for k in "${!USB_PORT_STATUS[@]}"; do
    if [ "${USB_PORT_STATUS[$k]}" != "success" ]; then
        echo "❌ Expected CAN interface on USB port '$k' was not found or not activated."
        FAILED_COUNT=$((FAILED_COUNT+1))
    fi
done

if [ "$SUCCESS_COUNT" -gt 0 ]; then
    echo "[RESULT]: ✅ $SUCCESS_COUNT expected CAN interfaces processed successfully."
else
    echo "[RESULT]: ❌ No USB interface matches the preset CAN configuration, please check whether the USB port is connected correctly."
fi

if [ "$FAILED_COUNT" -gt 0 ]; then
    echo "[RESULT]: 🚫 $FAILED_COUNT expected CAN interfaces failed to activate or were not found."
fi