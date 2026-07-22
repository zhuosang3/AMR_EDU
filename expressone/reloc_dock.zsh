#!/usr/bin/env zsh
# 充电桩标定定位 —— 插上充电线后手动执行
#   1. 从 MQTT 取一条消息，确认在充电 (charge_flag=1)
#   2. 调用 /reloc/dock_calib 触发重定位
#   3. 退出

source /opt/ros/jazzy/setup.zsh

echo '=== 充电桩标定定位 ==='

# 取一条 MQTT 消息，最多等 5 秒
echo -n '等待 MQTT 电源消息... '
PAYLOAD=$(mosquitto_sub -t /car/power -C 1 -W 5 2>&1)

if [[ $? -ne 0 || -z "$PAYLOAD" ]]; then
    echo '失败'
    echo '[错误] 未收到 MQTT 消息，power_monitor 是否在运行？'
    exit 1
fi
echo 'OK'

# 解析字段
POWER=$(echo "$PAYLOAD" | jq -r '.power // "?"')
CHARGE_FLAG=$(echo "$PAYLOAD" | jq -r '.charge_flag // -1')
CURRENT=$(echo "$PAYLOAD" | jq -r '.current // "?"')

echo "  电量: ${POWER}%  充电标志: ${CHARGE_FLAG}  电流: ${CURRENT}mA"

# 检查是否在充电
if [[ "$CHARGE_FLAG" != "1" ]]; then
    echo ''
    echo '[中止] 当前未在充电，请先插上充电线再执行。'
    exit 2
fi

# 触发标定
echo ''
echo '已确认在充电 → 触发充电桩标定定位...'
ros2 service call /reloc/dock_calib std_srvs/srv/Trigger

echo ''
echo '完成。'
