"""
EXP2 灯光客户端 (Python) v2.0

用法:
    from lights_client import LightsClient

    # 在 ROS2 节点中
    lights = LightsClient(node)

    lights.on(1, "r")            # 左前红
    lights.on("lf", "green")     # 左前绿
    lights.on("右后", "黄")       # 右后黄
    lights.off("rf", "red")      # 右前红灭
    lights.flash("lb", "y", 500) # 左后黄闪
    lights.all_on()              # 全开 (每个方位亮红色)
    lights.all_off()             # 全关

=== v2.0 注意事项 ===
底层已实现方位互斥: 每个方位同时只能点亮一种颜色。
调用 on() 时会自动关断同方位其他颜色。

依赖: rclpy, std_msgs
前提: lights_bridge 节点必须运行中
"""

from std_msgs.msg import String

POS_MAP = {
    "1": "1", "lf": "1", "左前": "1",
    "2": "2", "rf": "2", "右前": "2",
    "3": "3", "lb": "3", "左后": "3",
    "4": "4", "rb": "4", "右后": "4",
}

COLOR_MAP = {
    "r": "r", "red": "r", "红": "r", "红色": "r",
    "g": "g", "green": "g", "绿": "g", "绿色": "g",
    "y": "y", "yellow": "y", "黄": "y", "黄色": "y",
}


def _resolve_pos(pos):
    if isinstance(pos, int):
        return str(pos)
    return POS_MAP.get(pos, str(pos))


def _resolve_color(color):
    return COLOR_MAP.get(color, str(color))


class LightsClient:
    """灯光控制客户端 — 封装 /lights/command 话题发布 (v2.0 方位互斥)"""

    def __init__(self, node, topic="/lights/command"):
        """
        Args:
            node:  rclpy.node.Node 实例
            topic: 灯光命令话题名
        """
        self._pub = node.create_publisher(String, topic, 10)

    # ── 语义 API ──────────────────────────────────

    def on(self, pos, color):
        """点亮灯光 (自动关断同方位其他颜色)"""
        cmd = f"on {_resolve_pos(pos)} {_resolve_color(color)}"
        self._pub.publish(String(data=cmd))

    def off(self, pos, color):
        """关闭灯光"""
        cmd = f"off {_resolve_pos(pos)} {_resolve_color(color)}"
        self._pub.publish(String(data=cmd))

    def flash(self, pos, color, ms):
        """闪烁灯光 (ms 毫秒, 由 lights_bridge 端阻塞执行)"""
        cmd = f"flash {_resolve_pos(pos)} {_resolve_color(color)} {ms}"
        self._pub.publish(String(data=cmd))

    def all_on(self):
        """全开 (每个方位亮红色)"""
        self._pub.publish(String(data="all_on"))

    def all_off(self):
        """全关"""
        self._pub.publish(String(data="all_off"))
