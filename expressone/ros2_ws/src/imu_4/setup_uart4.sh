#!/bin/bash
#
# setup_uart4.sh — Enable UART4 on Orange Pi 5 Plus (RK3588)
# ==================================================================
#
# This script enables the UART4_M2 overlay so that UART4 is available
# on the 40-pin GPIO header:
#   - Pin 19: UART4_RX  (connect to HI12 UART1_TXD)
#   - Pin 23: UART4_TX  (connect to HI12 UART1_RXD)
#
# After running this script, REBOOT the Orange Pi.
#
# Usage:
#   chmod +x setup_uart4.sh
#   sudo ./setup_uart4.sh
#
# Manual method (if this script does not work):
#   sudo orangepi-config
#   → System → Hardware → UART → uart4-m2 → Save → Reboot
# ==================================================================

set -euo pipefail

# ---- Check root ----
if [ "$(id -u)" -ne 0 ]; then
    echo "ERROR: This script must be run as root (sudo)."
    exit 1
fi

echo "=== Orange Pi 5 Plus — UART4_M2 Enabler ==="
echo ""

BOARD_DETECT=""
if grep -qi "orangepi.*5.*plus" /proc/device-tree/model 2>/dev/null; then
    BOARD_DETECT="Orange Pi 5 Plus"
elif grep -qi "rk3588" /proc/device-tree/compatible 2>/dev/null; then
    BOARD_DETECT="RK3588-based board (likely Orange Pi 5 Plus)"
fi

if [ -n "$BOARD_DETECT" ]; then
    echo "Detected: $BOARD_DETECT"
else
    echo "WARNING: Could not verify board model. Proceeding anyway."
fi

echo ""
echo "UART4_M2 pin mapping on 40-pin header:"
echo "  Pin 19 (GPIO1_B3) = UART4_RX  ← HI12 UART1_TXD"
echo "  Pin 23 (GPIO1_B2) = UART4_TX  → HI12 UART1_RXD"
echo ""

# ---- Check for orangepi-config ----
if command -v orangepi-config &>/dev/null; then
    echo "orangepi-config found. You can also use the interactive menu:"
    echo "  sudo orangepi-config → System → Hardware → UART → uart4-m2"
    echo ""
    echo "Proceeding with overlay enable via environment file..."
else
    echo "orangepi-config not found. Using direct overlay method."
fi

# ---- Method 1: Ubuntu-Rockchip (Joshua Riek) style ----
if [ -f /boot/firmware/ubuntuEnv.txt ]; then
    OVERLAY_FILE="/boot/firmware/ubuntuEnv.txt"
elif [ -f /boot/orangepiEnv.txt ]; then
    OVERLAY_FILE="/boot/orangepiEnv.txt"
elif [ -f /boot/armbianEnv.txt ]; then
    OVERLAY_FILE="/boot/armbianEnv.txt"
else
    OVERLAY_FILE=""
fi

if [ -n "$OVERLAY_FILE" ]; then
    echo "Found boot config: $OVERLAY_FILE"

    # Check if uart4-m2 is already enabled
    if grep -q "uart4-m2" "$OVERLAY_FILE" 2>/dev/null; then
        echo "✓ uart4-m2 overlay already present in $OVERLAY_FILE"
    else
        echo "Adding uart4-m2 overlay..."
        # Backup
        cp "$OVERLAY_FILE" "$OVERLAY_FILE.bak.$(date +%Y%m%d_%H%M%S)"

        # Add to overlays line
        if grep -q "^overlays=" "$OVERLAY_FILE"; then
            sed -i 's/^overlays=.*/& uart4-m2/' "$OVERLAY_FILE"
        else
            echo "overlays=uart4-m2" >> "$OVERLAY_FILE"
        fi
        echo "✓ uart4-m2 overlay added"
    fi
fi

# ---- Method 2: Direct DTBO symlink (Armbian / legacy) ----
DTBO_DIR=""
for d in /boot/dtb/rockchip/overlay /boot/overlay-user /boot/overlays; do
    if [ -d "$d" ]; then
        DTBO_DIR="$d"
        break
    fi
done

if [ -n "$DTBO_DIR" ]; then
    DTBO_SRC="$DTBO_DIR/rk3588-uart4-m2.dtbo"
    if [ -f "$DTBO_SRC" ]; then
        echo "✓ DTBO file found: $DTBO_SRC"
    else
        echo "NOTE: $DTBO_SRC not found — overlay may be compiled into kernel"
    fi
fi

# ---- Check current UART status ----
echo ""
echo "=== Current UART devices ==="
ls -la /dev/ttyS* 2>/dev/null || echo "  No /dev/ttyS* devices found (normal before reboot)"
echo ""

# ---- Done ----
echo "=== Setup complete ==="
echo ""
echo "NEXT STEPS:"
echo "  1. Reboot:  sudo reboot"
echo "  2. After reboot, verify UART4 is available:"
echo "     ls /dev/ttyS*"
echo "     Expected: /dev/ttyS4 (or /dev/ttyS6 on some kernels)"
echo ""
echo "  3. Test with loopback (connect Pin 19 to Pin 23):"
echo "     # Terminal 1 (receiver):"
echo "     sudo stty -F /dev/ttyS4 115200 raw -echo"
echo "     sudo cat /dev/ttyS4"
echo ""
echo "     # Terminal 2 (sender):"
echo "     echo 'hello' | sudo tee /dev/ttyS4"
echo ""
echo "  4. Run HI12 reader:"
echo "     python3 hi12_reader.py --port /dev/ttyS4 --baud 115200"
echo ""
echo "  5. If /dev/ttyS4 doesn't appear, try:"
echo "     sudo orangepi-config  →  System  →  Hardware  →  UART  →  uart4-m2"
echo ""
