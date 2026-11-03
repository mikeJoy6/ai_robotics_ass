"""pose_ab_search —— A/B 位姿往返测试，用末端绕障规划器（planners/，--planner 选，默认 planners.DEFAULT）规划，不用 OMPL。

和 pose_ab 的区别：
  · pose_ab 测 IK：只走关节直线，直线撞了就放弃（genie 原设计）。
  · 本工具测绕障：arm_motion.plan_detour —— 先试末端直线，撞了就用 --planner 选的规划器绕行（默认 planners.DEFAULT：
    先试 V/梯形/U 绕行模板，不通再末端 RRT），最后捷径平滑成几段直线；
    每段积分笛卡尔从上一段的实际关节角接着走，构型连续不抡大圈；整臂碰撞(含 octomap)。
  · 记录的是 base_link 系末端位姿（不是关节角）：每次规划按当前实测腰关节换到手臂根系，腰动过也能用。
  · 执行走 JTC action，末端匀速、单关节超速/超加速度处自动压速(arm_motion.cart_profile)、中途不停；执行在后台线程，空格随时取消。

    source ~/agi@g2_robot/env_g2.sh
    ros2 run genie_robot_agibot_g2_motion pose_ab_search --arm right
    ros2 run genie_robot_agibot_g2_motion pose_ab_search --arm right --planner astar_w25   # 换规划器对比
"""
import argparse
import json
import math
import os
from pathlib import Path
import select
import sys
import termios
import threading
import time
import tty

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.utilities import remove_ros_args

from .algorithms import mat_to_rpy, rotvec
from . import planners
from .arm_motion import ArmMotion, Plan

HELP = '''A/B 绕障测试（末端空间网格搜索）：
  Shift+A / Shift+B  把当前末端位姿录成 A / B（base_link 系）
  a / b              规划并前往 A / B（先试直线，撞了就网格绕行）
  c                  开关往返循环：a → 到达 → 停 3s → b → 到达 → 停 3s → ...
  空格               取消当前动作并停止循环（不是急停）
  p                  打印记录      q  取消并退出
'''


def _pose_text(p, R):
    r, pi, y = np.degrees(mat_to_rpy(R))
    return f"({p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.3f}) m  rpy({r:+.1f}, {pi:+.1f}, {y:+.1f})°"


class PoseABSearch:
    def __init__(self, node, args):
        self.node, self.args, self.arm = node, args, args.arm
        self.path = Path(args.records.format(arm=args.arm)).expanduser()
        self.records = json.loads(self.path.read_text()) if self.path.exists() else {}
        self.worker = None
        self.cycling = False
        self.next_target = "a"
        self.cycle_due = math.inf
        self.stats = {"a": [], "b": []}             # 每次的规划耗时(s)

    def say(self, text):
        print(text, flush=True)

    @property
    def busy(self):
        return self.worker is not None and self.worker.is_alive()

    def record(self, name):
        if self.busy:
            raise ValueError("执行中不能录制")
        p, R = self.node.pose(self.arm)
        rec = dict(position=p.tolist(), rotation=R.tolist(), joints=self.node.q_arm(self.arm).tolist(),
                   stamp=time.strftime("%Y-%m-%d %H:%M:%S"))
        self.records[name] = rec
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix(".tmp")
        tmp.write_text(json.dumps(self.records, indent=2) + "\n")
        tmp.replace(self.path)
        self.say(f"已录 {name.upper()}: {_pose_text(p, R)}")

    def go(self, name):
        if self.busy:
            raise ValueError("上一个动作还没完；空格取消")
        if name not in self.records:
            raise ValueError(f"先按 Shift+{name.upper()} 录制")
        self.worker = threading.Thread(target=self._go, args=(name,), daemon=True)
        self.worker.start()

    def _go(self, name):
        rec = self.records[name]
        p_t, R_t = np.array(rec["position"]), np.array(rec["rotation"])
        p0, R0 = self.node.pose(self.arm)
        self.say(f"\n→ {name.upper()}：末端直线距离 {np.linalg.norm(p_t - p0)*100:.1f}cm，"
                 f"姿态差 {np.degrees(np.linalg.norm(rotvec(R_t @ R0.T))):.1f}°；规划中…")
        t0 = time.monotonic()
        plan = self.node.plan_detour(self.arm, p_t, R_t, planner=self.args.planner)
        dt = time.monotonic() - t0
        if not isinstance(plan, Plan):
            self.say(f"  ✗ 规划失败（{dt:.2f}s）：{plan}")
            self._stop_cycle()
            return
        self.stats[name].append(dt)
        self.say(f"  ✓ 规划 {dt:.2f}s：{plan.summary()}")
        ok, msg = self.node.execute(plan)
        p1, R1 = self.node.pose(self.arm)
        self.say(f"  {'✓' if ok else '✗'} {msg}；末端离目标 {np.linalg.norm(p1 - p_t)*1000:.1f}mm / "
                 f"{np.degrees(np.linalg.norm(rotvec(R_t @ R1.T))):.2f}°")
        if not ok:
            self._stop_cycle()
        elif self.cycling:
            self.next_target = "b" if name == "a" else "a"
            self.cycle_due = time.monotonic() + self.args.rest
            self.say(f"  停 {self.args.rest:.0f}s 后去 {self.next_target.upper()}")

    def toggle_cycle(self):
        if self.cycling:
            self._stop_cycle()
            return
        if self.busy:
            raise ValueError("等当前动作走完再开始循环")
        if not all(k in self.records for k in "ab"):
            raise ValueError("先录 A 和 B")
        self.cycling, self.next_target, self.cycle_due = True, "a", time.monotonic()
        self.say("往返循环开始（再按 c 停止后续循环，空格取消当前动作）")

    def _stop_cycle(self):
        if self.cycling:
            self.say("往返循环已停止")
        self.cycling, self.cycle_due = False, math.inf

    def tick(self):
        if self.cycling and not self.busy and time.monotonic() >= self.cycle_due:
            self.cycle_due = math.inf
            self.go(self.next_target)

    def cancel(self):
        self._stop_cycle()
        if self.node.cancel():
            self.say("已请求取消当前动作（不是急停）")

    def summary(self):
        for k, v in self.stats.items():
            if v:
                self.say(f"去 {k.upper()} 规划 {len(v)} 次：平均 {np.mean(v):.2f}s，最长 {max(v):.2f}s")


def main():
    parser = argparse.ArgumentParser(description=HELP, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--arm", choices=["left", "right"], default="right")
    parser.add_argument("--records", default="~/.ros/g2_motion/{arm}_ab_search.json")
    parser.add_argument("--planner", choices=list(planners.discover()[0]), default=planners.DEFAULT,
                        help=f"绕障规划器（默认 {planners.DEFAULT}；离线对比：ros2 run genie_robot_agibot_g2_motion planner_bench）")
    parser.add_argument("--rest", type=float, default=3.0, help="往返循环每次到达后停多久(s)")
    parser.add_argument("--cart-speed", type=float, default=None,
                        help="末端巡航速度 m/s（默认 arm_motion.CART_SPEED=0.15；单关节超 URDF 速度限位或 CART_JOINT_ACCEL=3rad/s² 处自动压速）")
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])
    if not sys.stdin.isatty():
        parser.error("请在交互终端里运行（需要读键盘）")

    if args.cart_speed is not None:
        if not 0.0 < args.cart_speed <= 0.3:
            parser.error("--cart-speed 要在 (0, 0.3] m/s")
        from . import arm_motion
        arm_motion.CART_SPEED = args.cart_speed
    rclpy.init()
    node = ArmMotion("g2_pose_ab_search")
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    print("等待 /robot_description 和 /joint_states …", flush=True)
    t0 = time.monotonic()
    while not node.ready() and time.monotonic() - t0 < 10:
        time.sleep(0.1)
    if not node.ready():
        print("10s 内没等到 URDF / 关节反馈：hardware.launch.py 在跑吗？", flush=True)
    app = PoseABSearch(node, args)
    print(f"手臂 {args.arm}；规划器 {args.planner}；记录文件 {app.path}\n{HELP}", flush=True)

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while rclpy.ok():
            app.tick()
            if not select.select([fd], [], [], 0.05)[0]:
                continue
            key = os.read(fd, 1).decode(errors="ignore")
            try:
                if key in ("A", "B"):
                    if app.cycling:
                        raise ValueError("先按 c 停止循环再录制")
                    app.record(key.lower())
                elif key in ("a", "b"):
                    if app.cycling:
                        raise ValueError("循环中不接受手动目标；先按 c 停止")
                    app.go(key)
                elif key == "c":
                    app.toggle_cycle()
                elif key == " ":
                    app.cancel()
                elif key == "p":
                    for k, r in sorted(app.records.items()):
                        app.say(f"{k.upper()}: {_pose_text(np.array(r['position']), np.array(r['rotation']))}  录于 {r['stamp']}")
                elif key == "q":
                    break
            except (ValueError, RuntimeError) as e:
                app.say(f"这次没执行: {e}")
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        app.cancel()
        if app.busy:
            app.worker.join(timeout=3.0)
        app.summary()
        ex.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
