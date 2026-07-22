#!/usr/bin/env python3
"""直线测距脚本 — 点到点直线运动。

对比 odom 报告距离 vs 地面实际距离。

用法:
  1. pm2 stop canopen_ros2
  2. 在轮子正下方地面贴胶带标记起点
  3. 前方 6m 内无障碍物
  4. 运行: python3 test_straight_line.py [--distance 5] [--speed 0.15]
  5. 车走直线, odom 达到目标距离后自动停
  6. 用卷尺量轮子从起点到终点的实际距离
  7. 恢复: pm2 start canopen_ros2

⚠️ 纯直线运动 (angular=0), 不修正航向, 可能会慢慢偏。
   Ctrl+C 随时急停(发零速)。
"""
import argparse
import asyncio
import json
import math
import signal
import sys

import websockets


async def test_straight(url: str, distance: float, speed: float):
    print(f"连接 {url} ...")
    async with websockets.connect(url) as ws:
        print(f"目标: 直线 {distance} m, 速度 {speed} m/s")
        print("前进中 — Ctrl+C 急停\n")

        x_start = None
        y_start = None
        traveled = 0.0
        stop = False

        def handle_sigint(*_):
            nonlocal stop
            stop = True

        signal.signal(signal.SIGINT, handle_sigint)

        cmd = json.dumps({"linear": speed, "angular": 0.0})
        last_print = 0.0

        while not stop:
            await ws.send(cmd)
            try:
                msg = await asyncio.wait_for(ws.recv(), timeout=0.5)
            except asyncio.TimeoutError:
                continue
            try:
                data = json.loads(msg)
            except json.JSONDecodeError:
                continue
            if "error" in data and data["error"]:
                print(f"\n⚠️ 驱动报错: {data['error']} — 停止")
                break
            x = data.get("pose", {}).get("x")
            y = data.get("pose", {}).get("y")
            if x is None or y is None:
                continue
            if x_start is None:
                x_start = x
                y_start = y
            traveled = math.hypot(x - x_start, y - y_start)
            # 每 0.5m 打印进度
            if traveled - last_print >= 0.5:
                last_print = traveled
                print(f"  odom 距离: {traveled:.2f} / {distance:.1f} m")
            if traveled >= distance:
                break

        # 停车
        stop_cmd = json.dumps({"linear": 0.0, "angular": 0.0})
        for _ in range(5):
            await ws.send(stop_cmd)
            await asyncio.sleep(0.05)

        print("\n══════════════ 结果 ══════════════")
        print(f"odom 报告距离: {traveled:.3f} m")
        print(f"速度: {speed} m/s")
        print()
        print("现在用卷尺量轮子从起点到终点的直线距离 (精确到 cm)")
        print()
        print("误差率 = (实际距离 - odom距离) / odom距离 × 100%")
        if traveled > 0:
            print(f"\n参考: 若实际 5.00 m → 误差率 = {(5.0 - traveled) / traveled * 100:.1f}%")


def main():
    p = argparse.ArgumentParser(description="直线测距")
    p.add_argument("--url", default="ws://127.0.0.1:9090")
    p.add_argument("--distance", type=float, default=5.0, help="目标距离 m (默认 5)")
    p.add_argument("--speed", type=float, default=0.15,
                   help="线速度 m/s (默认 0.15, 与导航限速一致)")
    args = p.parse_args()
    try:
        asyncio.run(test_straight(args.url, args.distance, args.speed))
    except (ConnectionRefusedError, OSError) as e:
        print(f"连接失败: {e}")
        print("检查 hls_canopen 是否在跑, 以及 canopen_ros2 是否已停 "
              "(pm2 stop canopen_ros2 — WebSocket 只接受一个客户端)")
        sys.exit(1)


if __name__ == "__main__":
    main()
