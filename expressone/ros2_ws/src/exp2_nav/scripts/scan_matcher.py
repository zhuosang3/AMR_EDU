#!/usr/bin/env python3
"""
scan_matcher.py — 激光→地图距离场匹配器（纯计算，无 ROS 依赖）。

仅供 reloc_manager 的 mh(多假设)模式做 top-N 多解诊断：给一帧激光 + rviz 点选粗位，
在点击 ±window 内全 yaw 粗搜，返回空间互异的 top-N 候选(到墙 trimmed 均距最低的几个)。
定位决策不依赖本 matcher(单帧在此环境多解), 位姿由 AMCL 收敛给出; 这里只展示多解供日志。

打分原理：每个候选位姿, 把扫描点投到地图距离场(每像素到最近墙的距离)上,
trimmed mean(取贴合最好的 N% 点, 扔动态杂物离群)越小越贴合。
"""
import os, math
import numpy as np
import cv2

OCC_THRESH = 100  # PGM 灰度 < 此值视为障碍(墙)


class ScanMatcher:
    def __init__(self, map_yaml_path, cap_m=8.0):
        self.cap_m = cap_m
        res, origin, img_path = self._load_yaml(map_yaml_path)
        self.res = res
        self.origin = origin
        gray = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
        if gray is None:
            raise RuntimeError(f"读不到地图: {img_path}")
        self.occ = (gray < OCC_THRESH).astype(np.uint8) * 255          # 障碍=255
        # 距离场: 每像素到最近墙的距离(像素); 墙=0, 空闲>0
        self.dt = cv2.distanceTransform((self.occ == 0).astype(np.uint8),
                                        cv2.DIST_L2, 3).astype(np.float32)
        self.h, self.w = self.occ.shape

    @staticmethod
    def _load_yaml(path):
        res, origin, img = 0.05, [-11.431, -11.672, 0.0], "my_map.pgm"
        d = os.path.dirname(path)
        for line in open(path):
            if ':' not in line:
                continue
            k, _, v = line.partition(':')
            k, v = k.strip(), v.strip()
            if k == 'resolution':
                res = float(v)
            elif k == 'origin':
                origin = [float(t) for t in v.strip('[]').split(',')]
            elif k == 'image':
                img = v
        img_path = img if os.path.isabs(img) else os.path.join(d, img)
        return res, origin, img_path

    def _world_to_px(self, x, y):
        return int(round((x - self.origin[0]) / self.res)), \
               int(round(self.h - (y - self.origin[1]) / self.res))

    def _px_to_world(self, px, py):
        return px * self.res + self.origin[0], (self.h - py) * self.res + self.origin[1]

    def _render(self, ranges, angles):
        """扫描 -> 命中点偏移列表(像素, 图像系 y down)。过滤 inf/0/>cap。"""
        offsets = []
        for rng, a in zip(ranges, angles):
            if not np.isfinite(rng) or rng <= 0 or rng >= self.cap_m:
                continue
            offsets.append((math.cos(a) * rng / self.res, -math.sin(a) * rng / self.res))
        return offsets

    def _score_at_arr(self, arr, px, py, yaw_steps_deg, trim_frac):
        """给定位置遍历 yaw, 取 trimmed 平均到墙距离(米)最小的 (距离米, yaw°)。arr=预转换 ndarray。"""
        if len(arr) == 0:
            return None
        best_md, best_deg = None, 0
        for deg in yaw_steps_deg:
            th = math.radians(deg)
            c, s = math.cos(th), math.sin(th)
            rx = c * arr[:, 0] - s * arr[:, 1]
            ry = s * arr[:, 0] + c * arr[:, 1]
            xs = np.round(px + rx).astype(int)
            ys = np.round(py + ry).astype(int)
            m = (xs >= 0) & (xs < self.w) & (ys >= 0) & (ys < self.h)
            if not m.any():
                continue
            vals = np.sort(self.dt[ys[m], xs[m]])
            keep = max(1, int(len(vals) * trim_frac))
            md = float(vals[:keep].mean()) * self.res
            if best_md is None or md < best_md:
                best_md, best_deg = md, deg
        return (best_md, best_deg) if best_md is not None else None

    def match_topn(self, ranges, angle_min, angle_increment,
                   click_x, click_y, topn=5,
                   window_m=2.0, coarse_step_m=1.0, coarse_yaw_step_deg=10,
                   trim_frac=0.85, min_range_m=0.05, distinct_m=0.5):
        """点击 ±window 内全 yaw 粗搜, 返回空间互异(>distinct_m)的 top-N 候选, 按 score 升序。
        供 mh 模式日志展示多解(定位决策不依赖)。返回 [{x,y,yaw_deg,score_m}, ...]。"""
        n = len(ranges)
        angles = angle_min + angle_increment * np.arange(n)
        rng_arr = np.asarray(ranges, dtype=float)
        mask = np.isfinite(rng_arr) & (rng_arr >= min_range_m) & (rng_arr < self.cap_m)
        offsets = self._render(rng_arr[mask], angles[mask])
        if len(offsets) < 8:
            return []
        arr = np.array(offsets, dtype=np.float64)
        cpx, cpy = self._world_to_px(click_x, click_y)
        rad_px = int(window_m / self.res)
        coarse_step_px = max(1, int(round(coarse_step_m / self.res)))
        cyaws = [d % 360 for d in range(0, 360, max(1, int(coarse_yaw_step_deg)))]
        coarse = []
        for ddy in range(-rad_px, rad_px + 1, coarse_step_px):
            for ddx in range(-rad_px, rad_px + 1, coarse_step_px):
                px, py = cpx + ddx, cpy + ddy
                if not (0 <= px < self.w and 0 <= py < self.h):
                    continue
                if self.occ[py, px]:
                    continue
                r = self._score_at_arr(arr, px, py, cyaws, trim_frac)
                if r is None:
                    continue
                coarse.append((r[0], px, py, r[1]))
        coarse.sort(key=lambda c: c[0])
        out = []
        for score, px, py, deg in coarse:
            wx, wy = self._px_to_world(px, py)
            if all(math.hypot(wx - o['x'], wy - o['y']) > distinct_m for o in out):
                out.append({'x': wx, 'y': wy, 'yaw_deg': deg, 'score_m': score})
                if len(out) >= topn:
                    break
        return out


if __name__ == '__main__':
    # CLI 自测: python3 scan_matcher.py /tmp/scan.json clickx clicky [truex truey]
    import sys, json
    sm = ScanMatcher('/home/expressone/maps/my_map.yaml')
    d = json.load(open(sys.argv[1]))
    cx, cy = float(sys.argv[2]), float(sys.argv[3])
    tops = sm.match_topn(d['ranges'], d['angle_min'], d['angle_increment'],
                         cx, cy, topn=8, window_m=2.0, trim_frac=0.85)
    print(f"click=({cx:+.2f},{cy:+.2f}) top{len(tops)} 多解:")
    for i, t in enumerate(tops):
        line = f"  #{i+1} ({t['x']:+.2f},{t['y']:+.2f},yaw{t['yaw_deg']:.0f}°,s{t['score_m']:.3f}," \
               f"距点击{math.hypot(t['x']-cx, t['y']-cy):.2f}m)"
        if len(sys.argv) >= 6:
            tx, ty = float(sys.argv[4]), float(sys.argv[5])
            line += f" 距真值{math.hypot(t['x']-tx, t['y']-ty):.2f}m"
        print(line)
