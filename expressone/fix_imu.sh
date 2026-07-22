#!/bin/bash
# fix_imu.sh — 自动检测 CP210x IMU 序列号并更新 udev 规则，重启驱动
# 用法: bash fix_imu.sh
set -e

RULE=/etc/udev/rules.d/99-hi12-imu.rules
NVM_SH=~/.nvm/nvm.sh

echo "=== 1. 查找 CP210x 设备 ==="
DEV=$(for d in /dev/ttyUSB*; do
    [ -e "$d" ] && udevadm info -q property "$d" 2>/dev/null | grep -q ID_VENDOR_ID=10c4 && \
    udevadm info -q property "$d" 2>/dev/null | grep -q ID_MODEL_ID=ea60 && echo "$d" && break
done)

if [ -z "$DEV" ]; then
    echo "错误: 未找到 CP210x (10c4:ea60) 设备，请检查 IMU 是否已连接"
    exit 1
fi
echo "找到: $DEV"

NEW=$(udevadm info -q property "$DEV" 2>/dev/null | grep ID_SERIAL_SHORT= | cut -d= -f2)
OLD=$(grep -oP 'ATTRS\{serial\}=="\K[^"]+' "$RULE" 2>/dev/null || echo "NONE")
echo "设备序列号: $NEW"
echo "规则序列号: $OLD"

if [ "$NEW" = "$OLD" ]; then
    echo "序列号一致，无需更新"
else
    echo "=== 2. 更新 udev 规则 ==="
    sudo sed -i "s/ATTRS{serial}==\"[^\"]*\"/ATTRS{serial}==\"$NEW\"/" "$RULE"
    echo "已更新:"
    grep serial "$RULE"

    echo "=== 3. 重载 udev ==="
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    sleep 1

    if [ -L /dev/hi12_imu ]; then
        echo "/dev/hi12_imu -> $(readlink /dev/hi12_imu)"
    else
        echo "警告: /dev/hi12_imu 未创建"
    fi
fi

echo "=== 4. 重启 IMU 驱动 ==="
[ -f "$NVM_SH" ] && source "$NVM_SH" 2>/dev/null || true
pm2 restart hi12_imu

sleep 3
echo ""
echo "=== 5. 最新日志 ==="
tail -3 ~/.pm2/logs/hi12-imu-error.log 2>/dev/null
echo ""
echo "完成!"
