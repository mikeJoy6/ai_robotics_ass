"""新的运动工具:get_robot_state / move_arm_end / move_joints / go_to_pose。全部用假机器人,不需要 ROS。

    python3 -m pytest tests/test_motion_tools.py -q
"""
import math

import yaml

from harness.backends.fake import FakeBackend
from harness.gateway import Gateway
from harness.geometry import rotation_between_deg
from harness.loop import run_agent
from harness.motion.joints import GROUPS
from harness.tools.objects import make_tools

CFG = yaml.safe_load(open("configs/harness.yaml", encoding="utf-8"))
MOTION = CFG["motion"]
POSES = yaml.safe_load(open("configs/poses.yaml", encoding="utf-8"))
HEAD = GROUPS["head"][1]


def gateway_for(backend=None, **kwargs):
    backend = backend or FakeBackend()
    return backend, Gateway(backend, MOTION, {"red": []}, poses=POSES, **kwargs)


# ---------------- 工具说明和 gateway 要对得上 ----------------
def test_every_tool_has_a_handler():
    _, gateway = gateway_for()
    for tool in make_tools(["red"], list(POSES)):
        name = tool["function"]["name"]
        assert "没有工具" not in gateway.execute(name, {}).get("error", ""), name


def test_go_to_pose_hidden_without_poses():
    names = [t["function"]["name"] for t in make_tools(["red"])]
    assert "go_to_pose" not in names and "move_joints" in names


# ---------------- get_robot_state ----------------
def test_robot_state_has_all_groups_and_both_ends():
    _, gateway = gateway_for()
    state = gateway.execute("get_robot_state", {})
    assert {g: len(v) for g, v in state["joints_deg"].items()} == {"body": 5, "head": 3, "left_arm": 7, "right_arm": 7}
    assert set(state["ends"]) == {"left", "right"}
    assert state["moves_left"] == MOTION["max_moves_per_command"]


# ---------------- move_arm_end ----------------
def test_lift_left_arm():
    backend, gateway = gateway_for()
    start = backend.end_pose("left")["xyz_m"]
    result = gateway.execute("move_arm_end", {"arm": "left", "offset_cm": [0, 0, 10]})
    assert result["executed"]
    assert backend.executed == [[start[0], start[1], start[2] + 0.10]]
    assert result["error_to_target_mm"] < 0.01


def test_right_arm_moves_only_right_arm():
    backend, gateway = gateway_for()
    left_before = backend.end_pose("left")
    assert gateway.execute("move_arm_end", {"arm": "right", "offset_cm": [5, 0, 0]})["executed"]
    assert backend.end_pose("left") == left_before


def test_offset_too_large_refused():
    backend, gateway = gateway_for()
    too_far = MOTION["max_offset_m"] * 100 + 1
    assert "分几次" in gateway.execute("move_arm_end", {"arm": "left", "offset_cm": [0, 0, too_far]})["error"]
    assert backend.executed == []


def test_arm_target_outside_workspace_refused():
    backend, gateway = gateway_for(FakeBackend(end=(0.80, 0.40, 0.90)))
    assert "工作范围" in gateway.execute("move_arm_end", {"arm": "left", "offset_cm": [10, 0, 0]})["error"]
    assert backend.executed == []


def test_bad_offset_values_refused():
    _, gateway = gateway_for()
    for bad in ([0, 0], [0, 0, "10"], [0, 0, True], "上抬"):
        assert not gateway.execute("move_arm_end", {"arm": "left", "offset_cm": bad})["ok"], bad
    assert not gateway.execute("move_arm_end", {"arm": "middle", "offset_cm": [0, 0, 5]})["ok"]
    assert not gateway.execute("move_arm_end", {"arm": "left"})["ok"]           # 全是 0


def test_rotation_applied_and_limited():
    backend, gateway = gateway_for()
    result = gateway.execute("move_arm_end", {"arm": "left", "rotate_deg": [0, 0, 30]})
    assert result["executed"] and abs(result["turn_deg"] - 30) < 0.1
    assert abs(rotation_between_deg([0, 0, 0], backend.end_pose("left")["rpy_deg"]) - 30) < 0.1
    too_much = MOTION["max_rotate_deg"] + 5
    assert "分几次" in gateway.execute("move_arm_end", {"arm": "left", "rotate_deg": [too_much, 0, 0]})["error"]


def test_path_dip_check_is_optional():
    class DippingBackend(FakeBackend):
        def plan_arm_to(self, arm, target, rpy_deg=None):
            plan = super().plan_arm_to(arm, target, rpy_deg)
            plan["lowest_z_m"] -= 0.20                     # 绕障时往下绕了 20cm
            return plan

    cfg = dict(MOTION, max_path_dip_m=None)
    backend = DippingBackend()
    assert Gateway(backend, cfg, {}).execute("move_arm_end", {"arm": "left", "offset_cm": [0, 0, 5]})["executed"]
    cfg = dict(MOTION, max_path_dip_m=0.05)
    backend = DippingBackend()
    assert "最低点" in Gateway(backend, cfg, {}).execute("move_arm_end",
                                                          {"arm": "left", "offset_cm": [0, 0, 5]})["error"]
    assert backend.executed == []


# ---------------- move_joints ----------------
def test_turn_head_by_delta():
    backend, gateway = gateway_for()
    result = gateway.execute("move_joints", {"group": "head", "deltas_deg": [30, 0, 0]})
    assert result["executed"]
    assert math.isclose(backend.joints[HEAD[0]], math.radians(30))
    assert result["joints_deg"]["head"] == [30.0, 0.0, 0.0]


def test_turn_head_to_absolute_angle():
    backend, gateway = gateway_for(FakeBackend(joints={HEAD[0]: math.radians(20)}))
    assert gateway.execute("move_joints", {"group": "head", "angles_deg": [0, 0, 0]})["executed"]
    assert abs(backend.joints[HEAD[0]]) < 1e-9


def test_joint_delta_limit():
    backend, gateway = gateway_for()
    too_much = MOTION["max_joint_delta_deg"]["body"] + 1
    assert "分几次" in gateway.execute("move_joints", {"group": "body", "deltas_deg": [too_much, 0, 0, 0, 0]})["error"]
    assert backend.executed == []


def test_joint_urdf_limit():
    backend, gateway = gateway_for(FakeBackend(limit_rad=0.5))
    assert "限位" in gateway.execute("move_joints", {"group": "head", "deltas_deg": [40, 0, 0]})["error"]
    assert backend.executed == []


def test_bad_joint_args_refused():
    _, gateway = gateway_for()
    assert not gateway.execute("move_joints", {"group": "head", "deltas_deg": [10, 0]})["ok"]       # 长度不对
    assert not gateway.execute("move_joints", {"group": "head"})["ok"]                              # 两个都没给
    assert not gateway.execute("move_joints", {"group": "head", "deltas_deg": [1, 0, 0],
                                               "angles_deg": [1, 0, 0]})["ok"]                      # 两个都给了
    assert not gateway.execute("move_joints", {"group": "tail", "deltas_deg": [1, 0, 0]})["ok"]
    assert not gateway.execute("move_joints", {"group": "head", "deltas_deg": [0, 0, 0]})["ok"]     # 不用动


# ---------------- go_to_pose ----------------
def test_go_to_pose_moves_listed_groups_only():
    name = next(iter(POSES))
    backend, gateway = gateway_for()
    result = gateway.execute("go_to_pose", {"name": name, "groups": ["head"]})
    assert result["executed"]
    assert set(backend.executed[0]) == set(HEAD)
    assert all(math.isclose(backend.joints[j], POSES[name]["joints"][j]) for j in HEAD)


def test_go_to_unknown_pose_refused():
    backend, gateway = gateway_for()
    assert "没有位姿" in gateway.execute("go_to_pose", {"name": "dance"})["error"]
    assert backend.executed == []


# ---------------- 所有运动都受 只规划 / 人工确认 / 次数 约束 ----------------
MOVES = [("move_arm_end", {"arm": "left", "offset_cm": [0, 0, 5]}),
         ("move_joints", {"group": "head", "deltas_deg": [10, 0, 0]}),
         ("go_to_pose", {"name": next(iter(POSES)), "groups": ["head"]})]


def test_plan_only_never_moves():
    for name, args in MOVES:
        backend, gateway = gateway_for(plan_only=True)
        result = gateway.execute(name, args)
        assert result["ok"] and not result["executed"], name
        assert backend.executed == [], name


def test_user_refusal_never_moves():
    for name, args in MOVES:
        prompts = []
        backend, gateway = gateway_for(confirm=lambda text: prompts.append(text) or False)
        assert "拒绝" in gateway.execute(name, args)["error"], name
        assert backend.executed == [] and len(prompts) == 1, name


def test_move_count_shared_across_tools():
    backend, gateway = gateway_for()
    limit = MOTION["max_moves_per_command"]
    for k in range(limit):
        assert gateway.execute("move_joints", {"group": "head", "deltas_deg": [1 if k % 2 else -1, 0, 0]})["executed"]
    assert "次数已用完" in gateway.execute("move_arm_end", {"arm": "left", "offset_cm": [0, 0, 5]})["error"]
    assert len(backend.executed) == limit


# ---------------- 整个循环 ----------------
class ScriptedLLM:
    def __init__(self, replies):
        self.replies = list(replies)

    def chat(self, messages, tools):
        return self.replies.pop(0)


def call(name, arguments):
    return {"role": "assistant", "content": "", "tool_calls": [
        {"id": "c1", "type": "function", "function": {"name": name, "arguments": arguments}}]}


def test_agent_loop_turns_head():
    backend, gateway = gateway_for()
    llm = ScriptedLLM([call("move_joints", '{"group": "head", "deltas_deg": [20, 0, 0]}'),
                       {"role": "assistant", "content": "头已经转了 20 度"}])
    assert run_agent("头往左转 20 度", llm, gateway, make_tools(["red"], list(POSES))) == "头已经转了 20 度"
    assert math.isclose(backend.joints[HEAD[0]], math.radians(20))
