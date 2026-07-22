#!/bin/bash
# enable-x11-gpu.sh — GDM Wayland → X11，启用 Mali GPU 硬件渲染
#
# 问题: rviz2 在 GNOME Wayland 下 GLX 回落 llvmpipe，CPU 300%+
# 修复: 切到原生 X11，GLX→DRI3→panfrost→GPU，CPU ~12%
#
# 用法: sudo bash enable-x11-gpu.sh [--reboot]

set -e

GDM_CONF="/etc/gdm3/custom.conf"
KEY="WaylandEnable=false"

if [ "$(id -u)" -ne 0 ]; then
    echo "错误: 请用 sudo 运行"
    exit 1
fi

echo "=== EXP2 X11 GPU 渲染修复 ==="
echo ""

if grep -q "^WaylandEnable" "$GDM_CONF" 2>/dev/null; then
    CURRENT=$(grep "^WaylandEnable" "$GDM_CONF")
    if echo "$CURRENT" | grep -q "false"; then
        echo "✓ 已配置 ($CURRENT)，无需修改"
    else
        echo "→ 当前为 Wayland ($CURRENT)，正在修改..."
        sed -i "s/^WaylandEnable.*/WaylandEnable=false/" "$GDM_CONF"
        echo "✓ 已修改"
    fi
else
    echo "→ 未找到 WaylandEnable，正在追加..."
    echo "WaylandEnable=false" >> "$GDM_CONF"
    echo "✓ 已追加"
fi

echo ""
echo "当前配置:"
grep -v "^#" "$GDM_CONF" | grep -v "^$" | while read line; do echo "  $line"; done
echo ""

if echo "${XDG_SESSION_TYPE:-}" | grep -q "x11"; then
    echo "✓ 当前已在 X11 会话中"
elif echo "${XDG_SESSION_TYPE:-}" | grep -q "wayland"; then
    echo "⚠ 当前仍在 Wayland，需重启生效"
fi

if [ "${1:-}" = "--reboot" ]; then
    echo "→ 5 秒后重启..."
    sleep 5
    reboot
else
    echo "→ 请手动重启: sudo reboot"
fi
