#!/usr/bin/env python3
"""add_collision_mesh.py —— 把一个 STL 作为碰撞体加进 MoveIt planning scene（测试避障用的虚拟障碍物）。

等 /apply_planning_scene 可用后加一次就退出。MoveIt 的碰撞检查（含我们规划器用的 /check_state_validity）都会算它；
相机看不到它（虚拟的），所以 octomap 里那片是空的，不冲突。
参数：mesh(STL 路径，单位 m)、frame、xyz、rpy(rad)、scale、object_id。
  ros2 run genie_robot_agibot_g2_hardware add_collision_mesh.py --ros-args -p mesh:=/path/x.stl -p xyz:="'[0.8,0,-0.06]'"
"""
import math
import re
import struct

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Point, Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from shape_msgs.msg import Mesh, MeshTriangle


def read_stl(path):
    """返回 (N*3, 3) 顶点（每 3 个一个三角形），支持二进制 / ASCII。"""
    with open(path, "rb") as f:
        data = f.read()
    if data[:5] == b"solid" and b"facet" in data[:400]:
        return np.array(re.findall(rb"vertex\s+(\S+)\s+(\S+)\s+(\S+)", data), dtype=float)
    n = struct.unpack("<I", data[80:84])[0]
    rec = np.frombuffer(data[84:84 + 50 * n], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    return rec["v"].reshape(-1, 3).astype(float)


def rpy_to_quat(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r / 2), math.sin(r / 2), math.cos(p / 2), math.sin(p / 2),
                              math.cos(y / 2), math.sin(y / 2))
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def main():
    rclpy.init()
    node = Node("add_collision_mesh")
    p = node.declare_parameter
    mesh_path = p("mesh", "").value
    frame = p("frame", "base_link").value
    # 字符串 "[x, y, z]"：launch 里写 [0,0,1.57] 这种整数/小数混写也行（ROS 数组参数不许混类型）
    xyz = [float(v) for v in yaml.safe_load(p("xyz", "[0.8, 0.0, -0.06]").value)]
    rpy = [float(v) for v in yaml.safe_load(p("rpy", "[0.0, 0.0, 0.0]").value)]
    scale = float(p("scale", 1.0).value)
    oid = p("object_id", "test_obstacle").value

    V = read_stl(mesh_path) * scale
    mesh = Mesh()
    mesh.vertices = [Point(x=float(a), y=float(b), z=float(c)) for a, b, c in V]
    mesh.triangles = [MeshTriangle(vertex_indices=[3 * i, 3 * i + 1, 3 * i + 2]) for i in range(len(V) // 3)]
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = xyz
    q = rpy_to_quat(*rpy)
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q

    co = CollisionObject()
    co.header.frame_id = frame
    co.id = oid
    co.operation = CollisionObject.ADD                # 同 id 再加一次 = 替换（改位置后重跑即可）
    co.meshes.append(mesh)
    co.mesh_poses.append(pose)
    scene = PlanningScene(is_diff=True)
    scene.world.collision_objects.append(co)

    cli = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    node.get_logger().info("等 /apply_planning_scene（move_group）…")
    while rclpy.ok() and not cli.wait_for_service(timeout_sec=2.0):
        pass
    fut = cli.call_async(ApplyPlanningScene.Request(scene=scene))
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
    ok = fut.done() and fut.result() is not None and fut.result().success
    lo, hi = V.min(0), V.max(0)
    node.get_logger().info(
        f"{'已加入' if ok else '加入失败'} {oid}：{mesh_path}（{len(V) // 3} 三角形，尺寸 {np.round(hi - lo, 3).tolist()}m）"
        f" @ {frame} xyz={xyz} rpy={rpy}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
