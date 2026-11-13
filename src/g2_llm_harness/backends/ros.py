"""真机器人:手臂运控(harness/motion/)+ 头部 RGBD(harness/perception/),都是本项目里的代码。

  · 手臂末端:ArmMotion —— 先积分笛卡尔直线,会撞再 plan_detour(via_then_rrt,和 pose_ab_search 同一套)
  · 关节组(腰/头/双臂):WholeBody —— 关节直线插值 + 整机碰撞检查,会撞再 JointDetour(经录好位姿中转 / 关节 RRT)

运行前要 source ~/agi@g2_robot/env_g2.sh(GDK 的 DDS 配置),并且 hardware.launch.py 已经在跑。
"""
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import MultiThreadedExecutor
from tf2_ros import TransformBroadcaster

from ..geometry import matrix_to_rpy_deg, rpy_deg_to_matrix
from ..motion.arm_motion import CART_SPEED, ArmMotion
from ..motion.joint_detour import JointDetour
from ..motion.whole_body import WholeBody
from ..perception.calib import CALIB
from ..perception.head_aim import HEAD_JOINTS, HeadAim
from ..perception.rgbd_source import RosSource
from ..perception.sam3_client import Sam3Client
from ..perception.sam3_objects import objects_from_masks, public, save_segmentation
from ..perception.scene3d import Rgbd
from ..perception.sensor_node import SensorNode
from .color_object import find_objects, merge_frames



def slow_down(plan, speed):
    """把轨迹按比例放慢到末端 speed m/s,形状不变。"""
    k = CART_SPEED / speed
    if k <= 1.0:
        return plan
    for point in plan.traj.points:
        t = (point.time_from_start.sec + point.time_from_start.nanosec * 1e-9) * k
        point.time_from_start.sec, point.time_from_start.nanosec = divmod(round(t * 1e9), 10**9)
        point.velocities = [v / k for v in point.velocities]
        if point.accelerations:
            point.accelerations = [a / k / k for a in point.accelerations]
    plan.duration *= k
    return plan


class RosBackend:
    def __init__(self, vision_cfg, speed_m_s, debug_dir="traces", poses=None):
        self.vision = vision_cfg
        self.speed = speed_m_s
        self.debug_dir = Path(debug_dir)
        self.poses = poses or {}            # JointDetour 的中转候选:{位姿名: {关节名: 弧度}}
        self.rgbd = Rgbd()
        self.block_xyz = None
        self._limits = None
        self._head_aim = None                # 第一次 head_angles_to 时用 URDF 建
        sam3 = vision_cfg["sam3"]
        self.sam3 = Sam3Client(sam3["url"], timeout_s=sam3["timeout_s"])

        rclpy.init()
        self.node = rclpy.create_node("robot_harness")
        self.arm = ArmMotion("robot_harness_arm")
        self.body = WholeBody(self.node)        # 订阅 /joint_states,连 4 个 JTC 和 /check_state_validity
        self.camera = SensorNode("robot_harness_camera")
        self.tf = TransformBroadcaster(self.node)
        self.node.create_timer(0.1, self._publish_tf)

        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.node)
        self.executor.add_node(self.arm)
        self.thread = threading.Thread(target=self._spin, daemon=True)
        self.thread.start()

        try:
            self._wait_until_ready()
        except Exception:
            self.close()
            raise

    def _spin(self):
        try:
            self.executor.spin()
        except Exception:      # 退出时 executor 关闭的竞争,不影响结果
            pass

    def _wait_until_ready(self):
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and not self.arm.ready():
            time.sleep(0.05)
        if not self.arm.ready():
            raise RuntimeError("10s 没收到 /robot_description 或 /joint_states,hardware.launch.py 在跑吗?")
        if not self.arm._sv.wait_for_service(timeout_sec=5.0):   # 否则 ArmMotion 会跳过自碰撞检查
            raise RuntimeError("/check_state_validity 不可用,move_group 在跑吗?")

    def locate_object(self, color, ranges, reference_xyz, reference_name):
        """同色物体有多个时,选离 reference_xyz(要用的那条手臂的末端)最近的。"""
        source = RosSource(sensor=self.camera)
        per_frame, rejected, last = [], [], None
        deadline = time.monotonic() + self.vision["timeout_s"]
        try:
            while len(per_frame) < self.vision["frames"] and time.monotonic() < deadline:
                frame = source.next(None)
                if frame is None:
                    time.sleep(0.01)
                    continue
                bgr, depth_mm, T_base_head, _ = frame
                depth_m = depth_mm.astype(np.float32) / 1000.0
                points, _ = self.rgbd.points_base(depth_m, T_base_head)
                found, why = find_objects(bgr, depth_m, points, CALIB["depth"]["fx"], ranges, self.vision)
                per_frame.append(found)
                rejected = why or rejected
                last = bgr
        finally:
            source.close()

        if not per_frame:
            return {"ok": False, "error": "没有收到头部相机画面(/gdk/camera/head_color、head_depth、/gdk/tf)"}
        block, error = merge_frames(per_frame, self.vision["min_good_frames"], self.vision["max_spread_m"],
                                    reference_xyz)
        self._save_debug_image(last, per_frame[-1], block)
        if block is None:
            return {"ok": False, "error": error, "rejected": rejected[:3]}

        self.block_xyz = block["xyz_m"]
        return {"ok": True, "frame": "base_link", "tf": "target_object", "color": color,
                "xyz_m": [round(float(v), 4) for v in block["xyz_m"]],
                "pixels": block["pixels"], "depth_m": block["depth_m"], "size_m": block["size_m"],
                "candidates": block["candidates"], "selected": f"离{reference_name}末端最近",
                "distance_to_arm_end_m": block["distance_to_arm_end_m"]}

    # ---------------- SAM 3:按名字找物体 ----------------
    def _grab_frame(self, timeout_s):
        """取一帧同步的 (彩色 BGR, 深度 mm, base_link→head_link3);超时返回 None。"""
        source = RosSource(sensor=self.camera)
        deadline = time.monotonic() + timeout_s
        try:
            while time.monotonic() < deadline:
                frame = source.next(None)
                if frame is not None:
                    return frame[:3]
                time.sleep(0.01)
        finally:
            source.close()
        return None

    def find_objects(self, query, suffix=""):
        """一帧画面 → SAM 3 找所有 query → 每个掩码 + 深度 → base_link 坐标,按离相机由近到远编号。
        每次都把画面、掩码、结果存到 traces/sam3/<时间>_<短语><suffix>/。"""
        cfg = self.vision["sam3"]
        frame = self._grab_frame(self.vision["timeout_s"])
        if frame is None:
            return {"ok": False, "error": "没有收到头部相机画面(/gdk/camera/head_color、head_depth、/gdk/tf)"}
        bgr, depth_mm, T_base_head = frame
        t0 = time.monotonic()
        try:
            results = self.sam3.segment(bgr, [query], cfg["threshold"])
        except RuntimeError as e:
            return {"ok": False, "error": str(e)}
        segment_ms = round((time.monotonic() - t0) * 1000)

        depth_m = depth_mm.astype(np.float32) / 1000.0
        points, cam_xyz = self.rgbd.points_base(depth_m, T_base_head)
        principal = (CALIB["depth"]["cx"], CALIB["depth"]["cy"])
        objects, rejected = objects_from_masks(results, depth_m, points, cam_xyz, principal,
                                               cfg["min_valid_depth_pixels"])
        meta = {"threshold": cfg["threshold"], "segment_ms": segment_ms, "sam3_timing_ms": self.sam3.last_timing_ms,
                "camera_xyz_m": [round(float(v), 4) for v in cam_xyz],
                "base_link_T_head_link3": np.round(T_base_head, 6).tolist(),
                "head_joints_deg": {j: round(float(np.degrees(self.body.snapshot()[j])), 2) for j in HEAD_JOINTS},
                "rejected": rejected}
        folder = save_segmentation(self.debug_dir / "sam3", query, bgr, depth_mm, results, objects, meta, suffix)
        return {"ok": True, "query": query, "objects": [public(o) for o in objects],
                "rejected": rejected[:3], "log_dir": str(folder)}

    def head_angles_to(self, xyz):
        """头部关节角:让相机光轴对准 xyz(base_link)。只改 yaw、pitch,夹在 URDF 限位里。"""
        if self._head_aim is None:
            self._head_aim = HeadAim(self.arm.urdf)
        target, residual, at_limit = self._head_aim.solve(self.joint_state(), xyz, self.joint_limits())
        return {"ok": True, "target": target, "residual_deg": residual, "at_limit": at_limit}

    def _save_debug_image(self, bgr, candidates, block):
        """黄框是所有候选,绿框是最终选中的目标。"""
        image = bgr.copy()
        for c in candidates:
            cv2.rectangle(image, (c["x"], c["y"]), (c["x"] + c["w"], c["y"] + c["h"]), (0, 255, 255), 1)
        if block is not None:
            x, y, w, h = block["x"], block["y"], block["w"], block["h"]
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        self.debug_dir.mkdir(exist_ok=True)
        cv2.imwrite(str(self.debug_dir / "last_object.jpg"), image)

    def _publish_tf(self):
        if self.block_xyz is None:
            return
        t = TransformStamped()
        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.child_frame_id = "target_object"
        t.transform.translation.x = float(self.block_xyz[0])
        t.transform.translation.y = float(self.block_xyz[1])
        t.transform.translation.z = float(self.block_xyz[2])
        t.transform.rotation.w = 1.0
        self.tf.sendTransform(t)

    # ---------------- 读 ----------------
    def end_pose(self, arm):
        """arm_{l,r}_end_link 在 base_link 下的位姿:由实测关节角正运动学得到,和 TF 同一条运动链。"""
        p, R = self.arm.pose(arm)
        return {"xyz_m": p.tolist(), "rpy_deg": matrix_to_rpy_deg(R)}

    def left_end_xyz(self):
        return self.end_pose("left")["xyz_m"]

    def joint_state(self):
        return self.body.snapshot()

    def joint_limits(self):
        """URDF 里每个关节的 (下限, 上限),弧度。第一次调用时解析,之后复用。"""
        if self._limits is None:
            from urdf_parser_py.urdf import URDF
            robot = URDF.from_xml_string(self.arm.urdf)
            self._limits = {j.name: (j.limit.lower, j.limit.upper) for j in robot.joints
                            if j.limit is not None and j.type != "continuous"}
        return dict(self._limits)

    # ---------------- 规划(不动) ----------------
    def plan_arm_to(self, arm, target, rpy_deg=None):
        """末端到 target(base_link,米)。rpy_deg=None 保持当前姿态。
        先试积分笛卡尔直线;直线不行再 plan_detour(via_then_rrt),和 pose_ab_search 一样。"""
        target = np.array(target, dtype=float)
        R = None if rpy_deg is None else rpy_deg_to_matrix(rpy_deg)
        plan = self.arm.plan_cartesian(arm, target, R)
        if isinstance(plan, str):
            detour = self.arm.plan_detour(arm, target, R)
            if isinstance(detour, str):
                return {"ok": False, "error": f"直线:{plan};绕障:{detour}"}
            plan = detour
        slow_down(plan, self.speed)
        return {"ok": True, "kind": "arm", "plan": plan, "summary": plan.summary(),
                "lowest_z_m": self._lowest_z(arm, plan.path)}

    def plan_left_to(self, target):
        return self.plan_arm_to("left", target)

    def _lowest_z(self, arm, path):
        """整条路径上末端的最低高度(base_link),用于防止路径穿过桌面。"""
        p_root, R_root = self.arm._root_in_base()
        kin = self.arm.kins[arm]
        return min(float((p_root + R_root @ kin.fk(np.asarray(q))[0])[2]) for q in path)

    def plan_joints(self, target, speed):
        """关节组到 target({关节名: 弧度}):整机碰撞检查,直线会撞就经录好位姿中转 / 关节 RRT。"""
        detour = JointDetour(self.body._valid, self.joint_limits(), self.poses)
        plan = self.body.plan(target, speed=speed, detour=detour)
        if isinstance(plan, str):
            return {"ok": False, "error": plan}
        return {"ok": True, "kind": "joints", "plan": plan, "summary": plan.summary()}

    # ---------------- 执行 ----------------
    def execute(self, plan):
        if plan["kind"] == "arm":
            ok, msg = self.arm.execute(plan["plan"])
        else:
            ok, msg = self.body.run(plan["plan"])
        return {"ok": ok, "msg": msg}

    def close(self):
        self.arm.cancel()
        self.body.cancel()
        self.camera.destroy()
        self.executor.shutdown(timeout_sec=2.0)
        self.thread.join(timeout=2.0)
        self.arm.destroy_node()
        self.node.destroy_node()
        rclpy.try_shutdown()
