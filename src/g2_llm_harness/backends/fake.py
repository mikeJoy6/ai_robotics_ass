"""假机器人:不需要 ROS,用来测试 harness 的逻辑。方法和 ros.py 的 RosBackend 一一对应。"""
import math

from ..motion.joints import GROUPS
from ..perception.sam3_objects import slug

ARMS = ("left", "right")


def _distance(obj):
    return obj["distance_m"]


FAKE_CAMERA = (0.20, 0.0, 1.68)    # 头部相机大概的位置(base_link,米),零位时往下看约 25°
FAKE_CAMERA_TILT_DEG = 25.0


class FakeBackend:
    def __init__(self, block=(0.55, 0.25, 0.60), end=(0.59, 0.42, 0.77), visible=("red", "green"), plan_error=None,
                 joints=None, limit_rad=3.0, sam_objects=None, sam_error=None):
        self.block = list(block)
        # SAM 3 假结果:{英文短语: [物体坐标, ...]};sam_error 模拟服务没开
        self.sam_objects = sam_objects if sam_objects is not None else {
            "phone": [(0.75, -0.10, 0.80), (0.60, 0.30, 0.78)]}
        self.sam_error = sam_error
        self.segment_calls = []         # 每次 find_objects 的 (短语, 后缀)
        self.ends = {"left": {"xyz_m": list(end), "rpy_deg": [0.0, 0.0, 0.0]},
                     "right": {"xyz_m": [end[0], -end[1], end[2]], "rpy_deg": [0.0, 0.0, 0.0]}}
        self.visible = visible
        self.plan_error = plan_error
        self.joints = {j: 0.0 for _, names in GROUPS.values() for j in names}
        self.joints.update(joints or {})
        self.limits = {j: (-limit_rad, limit_rad) for j in self.joints}
        self.executed = []              # 每次执行的目标:手臂是 [x, y, z],关节是 {关节名: 弧度}

    # ---- 读 ----
    def locate_object(self, color, ranges, reference_xyz, reference_name):
        if color not in self.visible:
            return {"ok": False, "error": f"画面里没有看到 {color} 物体"}
        self.locate_reference = list(reference_xyz)
        return {"ok": True, "frame": "base_link", "tf": "target_object", "color": color, "xyz_m": list(self.block),
                "selected": f"离{reference_name}末端最近"}

    def find_objects(self, query, suffix=""):
        """和 RosBackend.find_objects 返回格式一样。偏离画面中心的像素按当前头部角度粗算。"""
        self.segment_calls.append((query, suffix))
        if self.sam_error:
            return {"ok": False, "error": self.sam_error}
        objects = []
        for xyz in self.sam_objects.get(query, []):
            yaw, pitch = self._head_angles_for(xyz)
            off_h = math.degrees(self.joints["idx11_head_joint1"] - yaw)       # 正 = 目标在右边
            off_v = math.degrees(pitch - self.joints["idx13_head_joint3"])     # 正 = 目标在下边
            objects.append({"label": query, "score": 0.9, "xyz_m": list(xyz),
                            "distance_m": round(math.dist(xyz, FAKE_CAMERA), 3),
                            "offset_from_center_px": [round(off_h * 5.3, 1), round(off_v * 5.3, 1)]})
        objects.sort(key=_distance)
        for k, obj in enumerate(objects, start=1):
            obj["id"] = f"{slug(query)}_{k}"
        return {"ok": True, "query": query, "objects": objects, "log_dir": f"traces/sam3/fake_{query}{suffix}"}

    def _head_angles_for(self, xyz):
        dx, dy, dz = (xyz[0] - FAKE_CAMERA[0], xyz[1] - FAKE_CAMERA[1], xyz[2] - FAKE_CAMERA[2])
        yaw = math.atan2(dy, dx)
        pitch = math.atan2(-dz, math.hypot(dx, dy)) - math.radians(FAKE_CAMERA_TILT_DEG)
        return yaw, pitch

    def head_angles_to(self, xyz):
        """和 RosBackend.head_angles_to 一样的返回格式。"""
        yaw, pitch = self._head_angles_for(xyz)
        target = {"idx11_head_joint1": yaw, "idx12_head_joint2": self.joints["idx12_head_joint2"],
                  "idx13_head_joint3": pitch}
        return {"ok": True, "target": target, "residual_deg": [0.0, 0.0], "at_limit": False}

    def end_pose(self, arm):
        return {"xyz_m": list(self.ends[arm]["xyz_m"]), "rpy_deg": list(self.ends[arm]["rpy_deg"])}

    def left_end_xyz(self):
        return self.end_pose("left")["xyz_m"]

    def joint_state(self):
        return dict(self.joints)

    def joint_limits(self):
        return dict(self.limits)

    # ---- 规划(不动) ----
    def plan_arm_to(self, arm, target, rpy_deg=None):
        if self.plan_error:
            return {"ok": False, "error": self.plan_error}
        start = self.ends[arm]["xyz_m"]
        return {"ok": True, "kind": "arm", "arm": arm, "target": list(target),
                "rpy_deg": list(rpy_deg) if rpy_deg is not None else list(self.ends[arm]["rpy_deg"]),
                "summary": "假规划", "lowest_z_m": min(start[2], target[2])}   # 直线路径的最低点在两端之一

    def plan_left_to(self, target):
        return self.plan_arm_to("left", target)

    def plan_joints(self, target, speed):
        if self.plan_error:
            return {"ok": False, "error": self.plan_error}
        dmax = max(abs(v - self.joints[j]) for j, v in target.items())
        return {"ok": True, "kind": "joints", "target": dict(target),
                "summary": f"假关节规划,最大关节变化 {math.degrees(dmax):.1f}°"}

    # ---- 执行 ----
    def execute(self, plan):
        if plan["kind"] == "arm":
            self.executed.append(plan["target"])
            self.ends[plan["arm"]] = {"xyz_m": list(plan["target"]), "rpy_deg": list(plan["rpy_deg"])}
        else:
            self.executed.append(dict(plan["target"]))
            self.joints.update(plan["target"])
        return {"ok": True, "msg": "假执行完成"}

    def close(self):
        pass
