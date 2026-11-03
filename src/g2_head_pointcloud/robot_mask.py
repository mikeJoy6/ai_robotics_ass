"""robot_mask.py —— 在深度图上"抠掉"手臂：把手臂各 link 的碰撞凸包投影成图像上的轮廓，给出每个像素的"手臂深度门限"。

head_depth_cloud 用它：轮廓里测到的深度落在 [该 link 最近点 − margin, 最远点 + far_margin] 的像素一律丢——手臂挡住的
地方相机不可能看到它后面的东西，测到的是假的（近距盲区测距错、边缘飞点、运动时时间差）。比手臂明显更近的（真夹在中间的
物体）、远在手臂后面的（轮廓罩大了、其实看得见的背景）保留。
"""
import os
import re
import struct

import cv2
import numpy as np
from ament_index_python.packages import get_package_share_directory
from scipy import ndimage
from scipy.spatial import ConvexHull
from urdf_parser_py.urdf import URDF, Box, Cylinder, Mesh, Sphere

from .voxel_util import quat_to_mat


def read_stl(path):
    """返回 (N*3, 3) 顶点，支持二进制 / ASCII STL。"""
    with open(path, "rb") as f:
        data = f.read()
    if data[:5] == b"solid" and b"facet" in data[:400]:
        return np.array(re.findall(rb"vertex\s+(\S+)\s+(\S+)\s+(\S+)", data), dtype=float)
    n = struct.unpack("<I", data[80:84])[0]
    rec = np.frombuffer(data[84:84 + 50 * n], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    return rec["v"].reshape(-1, 3).astype(float)


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _geometry_points(g):
    if isinstance(g, Mesh):
        pkg, rel = g.filename.replace("package://", "").split("/", 1)
        V = read_stl(os.path.join(get_package_share_directory(pkg), rel))
        return V * (np.asarray(g.scale) if g.scale else 1.0)
    if isinstance(g, Box):
        return np.array([[x, y, z] for x in (-.5, .5) for y in (-.5, .5) for z in (-.5, .5)]) * np.asarray(g.size)
    if isinstance(g, (Cylinder, Sphere)):
        r = g.radius
        L = g.length if isinstance(g, Cylinder) else 2 * r
        a = np.linspace(0, 2 * np.pi, 12, endpoint=False)
        ring = np.stack([r * np.cos(a), r * np.sin(a), np.zeros(12)], 1)
        return np.concatenate([ring + [0, 0, -L / 2], ring + [0, 0, L / 2]])
    return None


def link_hulls(urdf_str, prefixes):
    """{link 名: 凸包顶点(link 系)}，只取名字以 prefixes 之一开头、带碰撞几何的 link。"""
    robot = URDF.from_xml_string(urdf_str)
    out = {}
    for link in robot.links:
        if not link.name.startswith(tuple(prefixes)) or not link.collisions:
            continue
        pts = []
        for c in link.collisions:
            V = _geometry_points(c.geometry)
            if V is None:
                continue
            o = c.origin
            R = _rpy(*(o.rpy if o else (0, 0, 0)))
            pts.append(V @ R.T + np.asarray(o.xyz if o else (0, 0, 0)))
        if pts:
            V = np.concatenate(pts)
            out[link.name] = V[ConvexHull(V).vertices]
    return out


def depth_threshold(hulls, poses, grid_shape, fx, fy, cx, cy, stride, margin, far_margin, dilate_cells):
    """hulls: {link: 顶点}；poses: [(link, 平移, 四元数xyzw)]（link 在深度光学系里的位姿，可同一 link 多个时刻）。
    返回发布网格(grid_shape)上每格的深度区间 (lo, hi)：被手臂轮廓盖住的格 lo = 该 link 最近点 − margin、
    hi = 该 link 最远点 + far_margin，其他格 lo=inf、hi=-inf。深度落在 [lo, hi] 的像素是手臂/手臂造成的假点。
    hi 的用处：手臂贴近镜头时，碰撞模型/标定几 cm 的误差投影出来就是几十像素，轮廓会罩到后面真实可见的远处背景；
    手臂测错的深度、边缘飞点都离手臂不远（实测在手臂后 6~15cm），远处的墙不可能是手臂造成的，保留。"""
    h, w = grid_shape
    lo = np.full((h, w), np.inf, np.float32)
    hi = np.full((h, w), -np.inf, np.float32)
    mask = np.zeros((h, w), np.uint8)
    for link, t, q in poses:
        P = hulls[link] @ quat_to_mat(*q).T + t
        P = P[P[:, 2] > 0.02]                                 # 相机后面/贴着镜头的顶点不投影
        if len(P) < 3:
            continue
        u = np.clip((fx * P[:, 0] / P[:, 2] + cx) / stride, -1e4, 1e4)
        v = np.clip((fy * P[:, 1] / P[:, 2] + cy) / stride, -1e4, 1e4)
        poly = cv2.convexHull(np.round(np.stack([u, v], 1) * 16).astype(np.int32))
        mask[:] = 0
        cv2.fillConvexPoly(mask, poly, 1, lineType=cv2.LINE_8, shift=4)
        m = mask.astype(bool)
        lo[m] = np.minimum(lo[m], float(P[:, 2].min()) - margin)
        hi[m] = np.maximum(hi[m], float(P[:, 2].max()) + far_margin)
    if dilate_cells > 0:                                       # 轮廓往外扩几格：盖住边缘飞点
        size = 2 * dilate_cells + 1
        lo = ndimage.minimum_filter(lo, size=size)
        hi = ndimage.maximum_filter(hi, size=size)
    return lo, hi
