"""sim_world.py —— 离线碰撞世界：不用 ROS / MoveIt，给 planner_bench 当 /check_state_validity 的替身。

和真机 MoveIt 用同一套碰撞模型：
  · 机器人：URDF 里每个 link 的 <collision>（凸包 STL / box / cylinder），全部转成凸包（顶点 + 半空间方程）；
  · 自碰撞：SRDF 里 disable_collisions 的 link 对不查（同 MoveIt 的 ACM）；
  · 环境：octomap 占据体素 = res 边长的立方体。
判撞（凸包 vs 凸包，对立方体也是凸包）：任一方的表面采样点落进另一方内部 → 撞。
表面按 ~1cm 采样，体素 3cm；对凸体这个判法在 1cm 量级上和 FCL 一致，benchmark 足够。
只动一条手臂（7 关节），其余关节固定在给定值。坐标统一用手臂链根系 arm_base_link（和规划器一致）。
"""
import os
import re
import struct
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import ConvexHull, cKDTree
from urdf_parser_py.urdf import URDF, Box, Cylinder, Mesh, Sphere

from .kin import arm_kin, ARM_ROOT, BASE


def _read_stl(path):
    with open(path, "rb") as f:
        data = f.read()
    if data[:5] == b"solid" and b"facet" in data[:400]:
        v = re.findall(rb"vertex\s+(\S+)\s+(\S+)\s+(\S+)", data)
        return np.array(v, dtype=float)
    n = struct.unpack("<I", data[80:84])[0]
    rec = np.frombuffer(data[84:84 + 50 * n], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    return rec["v"].reshape(-1, 3).astype(float)


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _T(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = _rpy(*rpy)
    T[:3, 3] = xyz
    return T


def _axis_rot(axis, a):
    k = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    T = np.eye(4)
    T[:3, :3] = np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * K @ K
    return T


class Hull:
    """凸包：顶点、表面采样点、半空间方程（A x + b ≤ 0 为内部）。"""
    def __init__(self, V, spacing=0.01):
        h = ConvexHull(V)
        self.V = V[h.vertices]
        self.eq = h.equations
        pts = [self.V]
        for s in h.simplices:                         # 三角面上按 spacing 撒点
            a, b, c = V[s]
            m = int(max(np.linalg.norm(b - a), np.linalg.norm(c - a), np.linalg.norm(c - b)) / spacing)
            if m >= 2:
                u, w = np.meshgrid(np.linspace(0, 1, m + 1), np.linspace(0, 1, m + 1))
                keep = (u + w) <= 1
                pts.append(a + u[keep, None] * (b - a) + w[keep, None] * (c - a))
        S = np.concatenate(pts)                       # 体素下采样到 ~spacing（凸包三角面很多，原始点过密）
        _, i = np.unique(np.floor(S / (1.2 * spacing)).astype(np.int64), axis=0, return_index=True)
        self.S = S[np.sort(i)]
        self.center = self.V.mean(0)
        self.radius = float(np.max(np.linalg.norm(self.V - self.center, axis=1)))

    def moved(self, T):
        h = object.__new__(Hull)
        R, t = T[:3, :3], T[:3, 3]
        h.V, h.S = self.V @ R.T + t, self.S @ R.T + t
        n = self.eq[:, :3] @ R.T
        h.eq = np.hstack([n, (self.eq[:, 3] - n @ t)[:, None]])
        h.center, h.radius = self.center @ R.T + t, self.radius
        return h

    def inside(self, P, margin=0.0):
        return np.all(P @ self.eq[:, :3].T + self.eq[:, 3] < -margin, axis=1)


def _box_pts(size):
    return np.array([[sx, sy, sz] for sx in (-.5, .5) for sy in (-.5, .5) for sz in (-.5, .5)]) * np.asarray(size)


def _cyl_pts(r, L, n=16):
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    ring = np.stack([r * np.cos(a), r * np.sin(a), np.zeros(n)], 1)
    return np.concatenate([ring + [0, 0, -L / 2], ring + [0, 0, L / 2]])


class SimWorld:
    def __init__(self, urdf_str, srdf_str, arm, joints, mesh_root, hand_radius=0.08):
        """joints: {关节名: 值}，没给的按 0。arm: 规划哪条手臂（它的 7 关节随 valid(q) 变）。"""
        self.robot = URDF.from_xml_string(urdf_str)
        self.arm, self.kin = arm, arm_kin(urdf_str, arm)
        self.joints = dict(joints)
        self.hand_radius = hand_radius
        self.disabled = set()
        for e in ET.fromstring(srdf_str).iter("disable_collisions"):
            self.disabled.add(frozenset((e.get("link1"), e.get("link2"))))
        # 每个 link 的凸包（link 系）
        self.hulls = {}
        for link in self.robot.links:
            hs = []
            for c in link.collisions or []:
                g = c.geometry
                if isinstance(g, Mesh):
                    path = g.filename.replace("package://", "")
                    pkg, rel = path.split("/", 1)
                    V = _read_stl(os.path.join(mesh_root, pkg, rel)) * (np.asarray(g.scale) if g.scale else 1.0)
                elif isinstance(g, Box):
                    V = _box_pts(g.size)
                elif isinstance(g, Cylinder):
                    V = _cyl_pts(g.radius, g.length)
                elif isinstance(g, Sphere):
                    V = _cyl_pts(g.radius, 2 * g.radius) * 1.0
                else:
                    continue
                o = c.origin
                T = _T(o.xyz if o else [0, 0, 0], o.rpy if o else [0, 0, 0])
                hs.append(Hull(V @ T[:3, :3].T + T[:3, 3]))
            if hs:
                self.hulls[link.name] = hs
        self._fk_all()
        self.moving = [self.kin.chain.getSegment(i).getName() for i in range(self.kin.chain.getNrOfSegments())]
        self.moving_hulls = [l for l in self.moving if l in self.hulls]
        # 静止部分（根系）：每个 link 的凸包
        self.static = {l: [h.moved(self.T_root[l]) for h in hs]
                       for l, hs in self.hulls.items() if l not in self.moving}
        self.set_obstacles(np.empty((0, 3)), 0.03)

    def _fk_all(self):
        """整机 FK：每个 link 在 base_link 和手臂根系里的位姿（arm 关节取 self.joints）。"""
        T = {BASE: np.eye(4)}
        children = {}
        for j in self.robot.joints:
            children.setdefault(j.parent, []).append(j)
        stack = [BASE]
        while stack:
            p = stack.pop()
            for j in children.get(p, []):
                o = j.origin
                Tj = T[p] @ _T(o.xyz if o else [0, 0, 0], o.rpy if o else [0, 0, 0])
                q = float(self.joints.get(j.name, 0.0))
                if j.type in ("revolute", "continuous"):
                    Tj = Tj @ _axis_rot(j.axis, q)
                elif j.type == "prismatic":
                    Tj = Tj.copy(); Tj[:3, 3] += Tj[:3, :3] @ (np.asarray(j.axis) * q)
                T[j.child] = Tj
                stack.append(j.child)
        self.T_base = T
        Tr_inv = np.linalg.inv(T[ARM_ROOT])
        self.T_root = {k: Tr_inv @ v for k, v in T.items()}
        self.root_in_base = T[ARM_ROOT]

    def base_to_root(self, P):
        P = np.asarray(P, float)
        R, t = self.root_in_base[:3, :3], self.root_in_base[:3, 3]
        return (P - t) @ R

    def up_in_root(self):
        return self.root_in_base[:3, :3].T @ np.array([0.0, 0.0, 1.0])

    def set_obstacles(self, P_root, res):
        """octomap 体素中心（根系）。"""
        self.obs = np.asarray(P_root, float).reshape(-1, 3)
        self.res = res
        self.obs_tree = cKDTree(self.obs) if len(self.obs) else None
        if len(self.obs):                             # 体素立方体的表面采样（用来查"体素点落进 link 凸包"）
            g = np.linspace(-res / 2, res / 2, 3)
            cube = np.array([[x, y, z] for x in g for y in g for z in g])
            self.obs_pts = (self.obs[:, None, :] + cube[None]).reshape(-1, 3)
            self.obs_pts_tree = cKDTree(self.obs_pts)

    def link_hulls(self, q):
        """手臂各 link 在根系里的凸包。"""
        import PyKDL as kdl
        arr = self.kin._arr(q)
        out = {}
        for i, name in enumerate(self.moving):
            if name not in self.hulls:
                continue
            f = kdl.Frame()
            self.kin._fk.JntToCart(arr, f, i + 1)
            T = np.eye(4)
            T[:3, :3] = [[f.M[r, c] for c in range(3)] for r in range(3)]
            T[:3, 3] = [f.p[0], f.p[1], f.p[2]]
            out[name] = [h.moved(T) for h in self.hulls[name]]
        return out

    @staticmethod
    def _hull_hit(a, b):
        if np.linalg.norm(a.center - b.center) > a.radius + b.radius:
            return False
        Sb = b.S[np.sum((b.S - a.center) ** 2, axis=1) <= a.radius ** 2]
        if len(Sb) and a.inside(Sb).any():
            return True
        Sa = a.S[np.sum((a.S - b.center) ** 2, axis=1) <= b.radius ** 2]
        return bool(len(Sa) and b.inside(Sa).any())

    def valid(self, q):
        if np.any(q < self.kin.lower - 1e-6) or np.any(q > self.kin.upper + 1e-6):
            return False
        H = self.link_hulls(q)
        names = list(H)
        for i, la in enumerate(names):
            # 自碰撞：手臂内部
            for lb in names[i + 1:]:
                if frozenset((la, lb)) in self.disabled:
                    continue
                if any(self._hull_hit(a, b) for a in H[la] for b in H[lb]):
                    return False
            # 手臂 vs 身体/头/另一条臂
            for lb, hs in self.static.items():
                if frozenset((la, lb)) in self.disabled:
                    continue
                if any(self._hull_hit(a, b) for a in H[la] for b in hs):
                    return False
            # 手臂 vs octomap
            if self.obs_tree is not None:
                for a in H[la]:
                    idx = self.obs_pts_tree.query_ball_point(a.center, a.radius + 1e-3)
                    if idx and a.inside(self.obs_pts[idx]).any():
                        return False
                    near = self.obs_tree.query_ball_point(a.center, a.radius + self.res)
                    if near:
                        c = self.obs[near]
                        inside = np.all(np.abs(a.S[:, None, :] - c[None]) <= self.res / 2, axis=2)
                        if inside.any():
                            return False
        return True

    def clearance(self, q):
        """手臂表面离最近障碍体素中心的距离、手(末端球代理)离障碍的距离（URDF 没有手，真机 MoveIt 也不查手）。"""
        if self.obs_tree is None:
            return np.inf, np.inf
        H = self.link_hulls(q)
        d_arm = min(float(self.obs_tree.query(h.S)[0].min()) for hs in H.values() for h in hs)
        p = self.kin.fk(q)[0]
        d_hand = float(self.obs_tree.query(p)[0]) - self.hand_radius
        return d_arm - self.res / 2, d_hand - self.res / 2
