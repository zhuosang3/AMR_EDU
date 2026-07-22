#!/usr/bin/env python3
"""轮距 (WHEEL_BASE) 标定脚本 — 原地旋转法。

原理:
  odom 的角速度 = (v_r - v_l) / WHEEL_BASE。
  若名义轮距与有效轮距(受轮胎接触面/负载影响)有偏差,
  odom 累计角度 theta_odom 与实际旋转角度 theta_real 不一致:
      B_effective = B_nominal * theta_odom / theta_real

用法:
  1. 停掉 ROS2 桥(独占 WebSocket): pm2 stop canopen_ros2
  2. 在地面贴胶带标记车头初始朝向
  3. 运行: python3 calibrate_wheelbase.py [--turns 10] [--speed 0.15]
  4. 车自转,直到 odom 认为转满 N 圈后自动停止
  5. 人工数实际转了几圈(含不足一圈的角度,目测到 ±5° 即可)
  6. 按脚本最后输出的公式算出有效轮距,改 main.rs 的 WHEEL_BASE
  7. 恢复: pm2 start canopen_ros2

⚠️ 车会原地旋转,确保周围 1m 无障碍物。Ctrl+C 随时急停(发零速)。
"""
import argparse
import asyncio
import json
import math
import signal
import sys

import websockets

NOMINAL_WHEEL_BASE = 0.418  # 与 hls_canopen main.rs 的 WHEEL_BASE 保持一致


async def calibrate(url: str, turns: float, speed: float):
    target = turns * 2.0 * math.pi
    print(f"连接 {url} ...")
    async with websockets.connect(url) as ws:
        print(f"目标: odom 累计 {turns} 圈 ({target:.2f} rad), 角速度 {speed} rad/s")
        print("开始旋转 — Ctrl+C 急停\n")

        theta_start = None
        theta_now = 0.0
        stop = False

        def handle_sigint(*_):
            nonlocal stop
            stop = True

        signal.signal(signal.SIGINT, handle_sigint)

        cmd = json.dumps({"linear": 0.0, "angular": speed})
        last_print = 0.0

        while not stop:
            await ws.send(cmd)  # 周期发送,喂狗 (watchdog 1s)
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
            theta = data.get("pose", {}).get("theta")
            if theta is None:
                continue
            if theta_start is None:
                theta_start = theta
            theta_now = theta - theta_start
            if abs(theta_now) - last_print >= math.pi / 2:  # 每 90° 打印
                last_print = abs(theta_now)
                print(f"  odom 累计 {math.degrees(theta_now):8.1f}° "
                      f"({theta_now / (2 * math.pi):5.2f} 圈)")
            if abs(theta_now) >= target:
                break

        # 停车
        stop_cmd = json.dumps({"linear": 0.0, "angular": 0.0})
        for _ in range(5):
            await ws.send(stop_cmd)
            await asyncio.sleep(0.05)

        odom_turns = theta_now / (2.0 * math.pi)
        print("\n══════════════ 结果 ══════════════")
        print(f"odom 累计角度: {math.degrees(theta_now):.1f}° = {odom_turns:.3f} 圈")
        print()
        print("现在人工数实际转的圈数 (对照胶带标记, 估到 ±5°),")
        print("然后计算有效轮距:")
        print(f"  B_eff = {NOMINAL_WHEEL_BASE} × {odom_turns:.3f} / 实际圈数")
        print()
        print("示例: 实际转了 9.5 圈 →")
        print(f"  B_eff = {NOMINAL_WHEEL_BASE} × {odom_turns:.3f} / 9.5 "
              f"= {NOMINAL_WHEEL_BASE * odom_turns / 9.5:.4f} m")
        print()
        print("把 B_eff 写入 hls_canopen/src/main.rs 的 WHEEL_BASE, 重新编译。")
        print("建议正反各转一次取平均, 消除单边打滑偏差。")


def main():
    p = argparse.ArgumentParser(description="轮距标定 — 原地旋转法")
    p.add_argument("--url", default="ws://127.0.0.1:9090")
    p.add_argument("--turns", type=float, default=10.0, help="目标圈数 (odom)")
    p.add_argument("--speed", type=float, default=0.15,
                   help="角速度 rad/s (默认 0.15, 与导航限速一致)")
    args = p.parse_args()
    try:
        asyncio.run(calibrate(args.url, args.turns, args.speed))
    except (ConnectionRefusedError, OSError) as e:
        print(f"连接失败: {e}")
        print("检查 hls_canopen 是否在跑, 以及 canopen_ros2 是否已停 "
              "(pm2 stop canopen_ros2 — WebSocket 只接受一个客户端)")
        sys.exit(1)


if __name__ == "__main__":
    main()
