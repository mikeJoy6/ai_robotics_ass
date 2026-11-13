"""模型和机器人之间的关卡:坐标在这里计算,安全检查也在这里,模型绕不过去。

碰撞由规划器负责(整机自碰撞 + octomap + 碰撞体,会撞就绕),这里只守底线:
  · approach_object:目标点由最近一次定位结果算出来,模型给不了坐标
  · move_arm_end:   只能给相对当前末端的偏移和转角;目标要在工作范围内
  · move_joints:    关节增量或目标角;要在 URDF 限位内,单次变化有上限(防止模型把度当成弧度之类的错)
  · go_to_pose:     只能选 configs/poses.yaml 里人录好的位姿
所有运动都是 先规划 → 检查 → (只规划模式到此为止)→ 人按 y → 执行。上限都在 configs/harness.yaml。
"""
import math
import time

from .geometry import rotate_rpy_deg, rotation_between_deg
from .motion.joints import GROUPS
from .perception.sam3_objects import slug

ARMS = ("left", "right")
ARM_NAME = {"left": "左臂", "right": "右臂"}
GROUP_NAME = {"body": "腰", "head": "头", "left_arm": "左臂", "right_arm": "右臂"}
LIMIT_MARGIN_RAD = 0.02         # 目标关节角离 URDF 限位至少留这么多


def _round(xyz):
    return [round(float(v), 4) for v in xyz]


def _is_number(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def _number_list(value, n, name):
    """检查 value 是不是 n 个数字的列表。是 → 返回 (列表, None);不是 → 返回 (None, 错误)。"""
    if not isinstance(value, list) or len(value) != n or not all(_is_number(v) for v in value):
        return None, f"{name} 必须是 {n} 个数字的列表"
    return [float(v) for v in value], None


class Gateway:
    def __init__(self, backend, motion_cfg, colors, confirm=None, plan_only=False, poses=None):
        self.backend = backend
        self.cfg = motion_cfg
        self.colors = colors            # {颜色名: HSV 范围列表}
        self.confirm = confirm          # confirm(提示文字) → True/False;None 表示不询问
        self.plan_only = plan_only      # True:只规划,绝不发送运动
        self.poses = poses or {}        # configs/poses.yaml:{位姿名: {"groups": [...], "joints": {...}}}
        self.target = None
        self.target_time = None
        self.target_arm = None          # locate_object 按哪条手臂选的“最近”
        self.objects = {}               # find_objects 找到的物体:{编号: {"xyz_m", "time", "label"}};坐标只存在这里
        self.moves = 0

    def execute(self, name, args):
        handlers = {"locate_object": self._locate,
                    "get_left_end_pose": self._end_pose,
                    "get_robot_state": self._robot_state,
                    "approach_object": self._approach,
                    "move_arm_end": self._move_arm_end,
                    "move_joints": self._move_joints,
                    "go_to_pose": self._go_to_pose,
                    "find_objects": self._find_objects,
                    "look_at_object": self._look_at_object}
        if name not in handlers:
            return {"ok": False, "error": f"没有工具 '{name}'"}
        try:
            return handlers[name](args)
        except Exception as e:
            return {"ok": False, "error": f"{type(e).__name__}: {e}"}

    # ================= 只读 =================
    def _locate(self, args):
        color = args.get("color")
        if color not in self.colors:
            return {"ok": False, "error": f"不认识颜色 '{color}',可选:{list(self.colors)}"}
        arm = args.get("arm", "left")
        if arm not in ARMS:
            return {"ok": False, "error": f"arm 只能是 {list(ARMS)}"}
        self.target = None              # 定位失败时不能沿用上一次的目标
        reference = self.backend.end_pose(arm)["xyz_m"]     # 同色多个时,选离这条手臂末端最近的
        result = self.backend.locate_object(color, self.colors[color], reference, ARM_NAME[arm])
        if result.get("ok"):
            self.target = list(result["xyz_m"])
            self.target_time = time.monotonic()
            self.target_arm = arm
        return result

    def _find_objects(self, args):
        """SAM 3 按英文短语找物体。坐标存进 self.objects,模型以后只用编号指代它们。"""
        query = args.get("query")
        if not isinstance(query, str) or not query.strip():
            return {"ok": False, "error": "query 必须是非空的英文名词短语,例如 phone"}
        query = query.strip()
        if not query.isascii():
            return {"ok": False, "error": "query 要用英文名词短语(SAM 3 只认英文),例如 手机 → phone"}
        result = self.backend.find_objects(query)
        if not result.get("ok"):
            return result
        prefix = slug(query) + "_"
        for key in list(self.objects):                 # 同一个短语重新找:旧编号作废,免得指到旧位置
            if key.startswith(prefix):
                del self.objects[key]
        now = time.monotonic()
        for obj in result["objects"]:
            self.objects[obj["id"]] = {"xyz_m": list(obj["xyz_m"]), "time": now, "label": query}
        if not result["objects"]:
            return {**result, "ok": False, "error": f"画面里没有找到 {query}"}
        return {**result, "count": len(result["objects"]), "order": "按离相机由近到远,第一个最近"}

    def _look_at_object(self, args):
        """转头,让 find_objects 找到的某个物体在画面中央。只动头部 3 个关节。"""
        object_id = args.get("object_id")
        obj = self.objects.get(object_id)
        if obj is None:
            known = list(self.objects) or "还没有,先调用 find_objects"
            return {"ok": False, "error": f"没有物体 '{object_id}',已找到的:{known}"}
        age = time.monotonic() - obj["time"]
        if age > self.cfg["detection_max_age_s"]:
            return {"ok": False, "error": f"{object_id} 的位置已过期({age:.0f}s),重新调用 find_objects"}
        refused = self._no_moves_left()
        if refused:
            return refused

        aim = self.backend.head_angles_to(obj["xyz_m"])
        if not aim.get("ok"):
            return aim
        current = self.backend.joint_state()
        limit = self.cfg["max_joint_delta_deg"]["head"]
        for name, value in aim["target"].items():
            delta = math.degrees(value - current[name])
            if abs(delta) > limit:
                return {"ok": False, "error": f"{name} 要转 {delta:.0f}°,超过单次上限 {limit}°"}
        if all(abs(aim["target"][j] - current[j]) < math.radians(0.5) for j in aim["target"]):
            return {"ok": True, "executed": False, "object_id": object_id,
                    "note": "物体已经在画面中央附近(差不到 0.5°),不需要转头"}

        result = self._plan_and_run_joints(aim["target"], ["head"], f"转头看 {object_id}")
        result.update(object_id=object_id, object_xyz_m=_round(obj["xyz_m"]),
                      expected_offset_deg=aim["residual_deg"], head_at_limit=aim["at_limit"])
        if aim["at_limit"]:
            result["note"] = "头已转到关节限位,物体可能无法完全居中"
        if result.get("executed"):
            result["check_after"] = self._check_centered(obj)
        return result

    def _check_centered(self, obj):
        """转头之后再分割一次,看物体离画面中心多少像素。这次的画面也存进日志(文件夹名带 _after)。"""
        after = self.backend.find_objects(obj["label"], suffix="_after")
        if not after.get("ok"):
            return {"ok": False, "error": after.get("error")}
        if not after["objects"]:
            return {"ok": False, "error": f"转头后画面里没找到 {obj['label']}", "log_dir": after.get("log_dir")}
        same, gap = None, None                          # 转头后编号可能变,按 3D 位置找回同一个物体
        for candidate in after["objects"]:
            d = math.dist(candidate["xyz_m"], obj["xyz_m"])
            if gap is None or d < gap:
                same, gap = candidate, d
        if gap > self.cfg["look_at_match_m"]:
            return {"ok": False, "error": "转头后没认出同一个物体(位置差太多)", "log_dir": after.get("log_dir")}
        return {"ok": True, "offset_from_center_px": same["offset_from_center_px"], "log_dir": after.get("log_dir")}

    def _end_pose(self, _args):
        return {"ok": True, "frame": "base_link", "link": "arm_l_end_link",
                "xyz_m": _round(self.backend.left_end_xyz())}

    def _robot_state(self, _args):
        joints = self.backend.joint_state()
        groups = {g: [round(math.degrees(joints[j]), 2) for j in names] for g, (_, names) in GROUPS.items()}
        ends = {}
        for arm in ARMS:
            pose = self.backend.end_pose(arm)
            ends[arm] = {"link": f"arm_{arm[0]}_end_link", "xyz_m": _round(pose["xyz_m"]),
                         "rpy_deg": [round(float(v), 1) for v in pose["rpy_deg"]]}
        return {"ok": True, "frame": "base_link", "joints_deg": groups, "ends": ends,
                "moves_left": self.cfg["max_moves_per_command"] - self.moves}

    # ================= 运动:共用的部分 =================
    def _no_moves_left(self):
        if self.moves >= self.cfg["max_moves_per_command"]:
            return {"ok": False, "executed": False, "error": "这条指令的运动次数已用完,请用户重新下指令"}
        return None

    def _confirm_and_run(self, plan, prompt, info):
        """规划和检查都通过之后:只规划模式到此为止;否则问人,按 y 才执行。"""
        if self.plan_only:
            return {"ok": True, "executed": False, "note": "只规划模式,机器人没有动", **info}
        if self.confirm and not self.confirm(f"{prompt}。输入 y 执行:"):
            return {"ok": False, "executed": False, "error": "用户拒绝了这次运动", **info}
        self.moves += 1
        done = self.backend.execute(plan)
        return {"ok": done["ok"], "executed": done["ok"], "msg": done["msg"], **info}

    def _check_workspace(self, arm, target):
        for axis, value in zip("xyz", target):
            low, high = self.cfg["workspace"][arm][axis]
            if not low <= value <= high:
                return f"目标 {axis}={value:.3f}m 超出{ARM_NAME[arm]}工作范围 [{low}, {high}]"
        return None

    # ================= approach_object:末端到物体正上方 =================
    def _approach(self, args):
        cfg = self.cfg
        arm = args.get("arm", "left")
        if arm not in ARMS:
            return {"ok": False, "error": f"arm 只能是 {list(ARMS)}"}

        object_id = args.get("object_id")
        if object_id is not None:
            # find_objects(SAM 3)找到的物体:按编号查表
            obj = self.objects.get(object_id)
            if obj is None:
                known = list(self.objects) or "还没有,先调用 find_objects"
                return {"ok": False, "error": f"没有物体 '{object_id}',已找到的:{known}"}
            object_xyz, found_time, again = obj["xyz_m"], obj["time"], "find_objects"
        else:
            # locate_object(按颜色)定位的物体
            if self.target is None:
                return {"ok": False, "error": "还没有定位物体:按颜色用 locate_object,按物体名用 find_objects 再传 object_id"}
            if self.target_arm != arm:  # “最近的”是按另一条手臂选的,可能根本不是用户要的那个
                return {"ok": False, "error": f"刚才是按离{ARM_NAME[self.target_arm]}最近选的物体,现在要用{ARM_NAME[arm]};"
                                              f"先调用 locate_object(arm={arm}) 重新定位"}
            object_xyz, found_time, again = self.target, self.target_time, "locate_object"
        age = time.monotonic() - found_time
        if age > cfg["detection_max_age_s"]:
            return {"ok": False, "error": f"物体位置已过期({age:.0f}s),重新调用 {again}"}
        refused = self._no_moves_left()
        if refused:
            return refused

        standoff = args.get("standoff_cm", cfg["standoff_default_m"] * 100)
        if not _is_number(standoff):
            return {"ok": False, "error": "standoff_cm 必须是数字"}
        standoff = float(standoff) / 100
        if not cfg["standoff_min_m"] <= standoff <= cfg["standoff_max_m"]:
            return {"ok": False, "error": f"standoff_cm 必须在 {cfg['standoff_min_m'] * 100:.0f} 到 "
                                          f"{cfg['standoff_max_m'] * 100:.0f} 之间"}

        target = [object_xyz[0], object_xyz[1], object_xyz[2] + standoff]
        error = self._check_workspace(arm, target)
        if error:
            return {"ok": False, "error": error}

        start = self.backend.end_pose(arm)["xyz_m"]
        travel = math.dist(start, target)
        if travel > cfg["max_travel_m"]:
            return {"ok": False, "error": f"移动距离 {travel:.2f}m 超过上限 {cfg['max_travel_m']}m"}

        plan = self.backend.plan_arm_to(arm, target)
        info = {"arm": arm, "target_xyz_m": _round(target), "standoff_m": round(standoff, 3),
                "travel_m": round(travel, 3), "plan": plan["summary"] if plan["ok"] else None}
        if not plan["ok"]:
            return {"ok": False, "executed": False, "error": f"规划失败:{plan['error']}", **info}
        floor = object_xyz[2] + cfg["min_path_above_object_m"]
        if plan["lowest_z_m"] < floor:
            return {"ok": False, "executed": False,
                    "error": f"路径最低点 {plan['lowest_z_m']:.2f}m 低于 {floor:.2f}m,可能穿过桌面。"
                             f"请用户先把{ARM_NAME[arm]}移到桌面上方。", **info}

        result = self._confirm_and_run(
            plan, f"{ARM_NAME[arm]}末端将移动到 {_round(target)}(直线距离 {travel * 100:.1f}cm,"
                  f"物体上方 {standoff * 100:.0f}cm),{plan['summary']}", info)
        if result.get("executed"):
            end = self.backend.end_pose(arm)["xyz_m"]
            result.update(end_xyz_m=_round(end), error_to_target_mm=round(math.dist(end, target) * 1000, 1),
                          distance_to_object_m=round(math.dist(end, object_xyz), 3))
        return result

    # ================= move_arm_end:末端相对移动 / 转动 =================
    def _move_arm_end(self, args):
        cfg = self.cfg
        arm = args.get("arm")
        if arm not in ARMS:
            return {"ok": False, "error": f"arm 只能是 {list(ARMS)}"}
        refused = self._no_moves_left()
        if refused:
            return refused

        offset_cm, error = _number_list(args.get("offset_cm", [0, 0, 0]), 3, "offset_cm")
        if error:
            return {"ok": False, "error": error}
        rotate_deg, error = _number_list(args.get("rotate_deg", [0, 0, 0]), 3, "rotate_deg")
        if error:
            return {"ok": False, "error": error}
        if not any(offset_cm) and not any(rotate_deg):
            return {"ok": False, "error": "offset_cm 和 rotate_deg 都是 0,不需要运动"}

        offset_m = [v / 100 for v in offset_cm]
        distance = math.hypot(*offset_m)
        if distance > cfg["max_offset_m"]:
            return {"ok": False, "error": f"一次移动 {distance:.2f}m 超过上限 {cfg['max_offset_m']}m,请分几次移动"}

        now = self.backend.end_pose(arm)
        start, start_rpy = now["xyz_m"], now["rpy_deg"]
        target = [s + d for s, d in zip(start, offset_m)]
        error = self._check_workspace(arm, target)
        if error:
            return {"ok": False, "error": error}

        target_rpy = None                                   # None = 保持当前姿态
        turn = 0.0
        if any(rotate_deg):
            target_rpy = rotate_rpy_deg(start_rpy, rotate_deg)
            turn = rotation_between_deg(start_rpy, target_rpy)
            if turn > cfg["max_rotate_deg"]:
                return {"ok": False, "error": f"一次转动 {turn:.0f}° 超过上限 {cfg['max_rotate_deg']}°,请分几次转"}

        plan = self.backend.plan_arm_to(arm, target, target_rpy)
        info = {"arm": arm, "start_xyz_m": _round(start), "target_xyz_m": _round(target),
                "target_rpy_deg": target_rpy, "turn_deg": round(turn, 1),
                "plan": plan["summary"] if plan["ok"] else None}
        if not plan["ok"]:
            return {"ok": False, "executed": False, "error": f"规划失败:{plan['error']}", **info}
        dip = cfg.get("max_path_dip_m")                     # None = 不检查,碰撞完全交给规划器
        floor = None if dip is None else min(start[2], target[2]) - dip
        if floor is not None and plan["lowest_z_m"] < floor:
            return {"ok": False, "executed": False,
                    "error": f"规划出的路径最低点 {plan['lowest_z_m']:.2f}m 比起点和终点低太多(下限 {floor:.2f}m),"
                             "可能扫到桌面,已拒绝", **info}

        what = f"偏移 {[round(v, 1) for v in offset_cm]}cm" + (f",转 {turn:.0f}°" if turn else "")
        result = self._confirm_and_run(plan, f"{ARM_NAME[arm]}末端 {_round(start)} → {_round(target)}({what}),"
                                             f"{plan['summary']}", info)
        if result.get("executed"):
            end = self.backend.end_pose(arm)
            result.update(end_xyz_m=_round(end["xyz_m"]), end_rpy_deg=end["rpy_deg"],
                          error_to_target_mm=round(math.dist(end["xyz_m"], target) * 1000, 1))
        return result

    # ================= move_joints:某一组关节转动 =================
    def _move_joints(self, args):
        group = args.get("group")
        if group not in GROUPS:
            return {"ok": False, "error": f"group 只能是 {list(GROUPS)}"}
        refused = self._no_moves_left()
        if refused:
            return refused

        names = GROUPS[group][1]
        current = self.backend.joint_state()
        has_deltas, has_angles = "deltas_deg" in args, "angles_deg" in args
        if has_deltas == has_angles:
            return {"ok": False, "error": "deltas_deg(相对转多少)和 angles_deg(转到多少)必须给且只给一个"}
        if has_deltas:
            values, error = _number_list(args["deltas_deg"], len(names), "deltas_deg")
            if error:
                return {"ok": False, "error": error}
            target = {j: current[j] + math.radians(d) for j, d in zip(names, values)}
        else:
            values, error = _number_list(args["angles_deg"], len(names), "angles_deg")
            if error:
                return {"ok": False, "error": error}
            target = {j: math.radians(a) for j, a in zip(names, values)}

        limit = self.cfg["max_joint_delta_deg"][group]
        for i, j in enumerate(names, start=1):
            delta = math.degrees(target[j] - current[j])
            if abs(delta) > limit:
                return {"ok": False, "error": f"{GROUP_NAME[group]}第 {i} 关节要转 {delta:.0f}°,"
                                              f"超过单次上限 {limit}°,请分几次转"}
        if all(abs(target[j] - current[j]) < 1e-4 for j in names):
            return {"ok": False, "error": "目标和当前关节角相同,不需要运动"}
        return self._plan_and_run_joints(target, [group], f"{GROUP_NAME[group]}关节")

    # ================= go_to_pose:去录好的位姿 =================
    def _go_to_pose(self, args):
        name = args.get("name")
        if name not in self.poses:
            return {"ok": False, "error": f"没有位姿 '{name}',可选:{list(self.poses)}"}
        refused = self._no_moves_left()
        if refused:
            return refused
        pose = self.poses[name]
        groups = args.get("groups") or list(pose["groups"])
        if not isinstance(groups, list) or any(g not in pose["groups"] for g in groups):
            return {"ok": False, "error": f"位姿 {name} 只包含这些组:{pose['groups']}"}
        target = {j: float(pose["joints"][j]) for g in groups for j in GROUPS[g][1]}
        return self._plan_and_run_joints(target, groups, f"位姿 {name}")

    def _plan_and_run_joints(self, target, groups, label):
        """move_joints 和 go_to_pose 共用:限位检查 → 规划(碰撞检查 + 绕行) → 确认 → 执行。"""
        limits = self.backend.joint_limits()
        for j, v in target.items():
            if j in limits:
                low, high = limits[j]
                if not low + LIMIT_MARGIN_RAD <= v <= high - LIMIT_MARGIN_RAD:
                    return {"ok": False, "error": f"{j} 目标 {math.degrees(v):.1f}° 超出限位 "
                                                  f"[{math.degrees(low):.1f}°, {math.degrees(high):.1f}°]"}

        current = self.backend.joint_state()
        speed = min(self.cfg["joint_speed_rad_s"][g] for g in groups)
        changes = {g: round(max(abs(math.degrees(target[j] - current[j])) for j in GROUPS[g][1]), 1)
                   for g in groups}
        plan = self.backend.plan_joints(target, speed)
        info = {"groups": groups, "max_change_deg": changes, "plan": plan["summary"] if plan["ok"] else None}
        if not plan["ok"]:
            return {"ok": False, "executed": False, "error": f"规划失败:{plan['error']}", **info}

        text = ",".join(f"{GROUP_NAME[g]}最大 {d}°" for g, d in changes.items())
        result = self._confirm_and_run(plan, f"{label}:{text},峰值速度 {speed} rad/s,{plan['summary']}", info)
        if result.get("executed"):
            now = self.backend.joint_state()
            result["joints_deg"] = {g: [round(math.degrees(now[j]), 2) for j in GROUPS[g][1]] for g in groups}
        return result
