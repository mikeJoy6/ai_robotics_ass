"""Recorded-pose IK/direct-connection experiment; execution through JTC action.

Ported from g1_pointcloud_grasping genie_motion/pose_ab.py. Changes for G2 only:
  · --arm left|right (G1 was right-arm only), chains/controllers from differential_ik.ARMS
  · arm speed + whole-body snapshot from /joint_states (G1 read /hal/right_arm_data)
  · waist-unchanged check compares TF base_link <- arm_base_link (G1: base_link <- base_link_r)
  · algorithm menu default g2_seeded_lm = the sanofi-lineage IK that arm_motion uses
    (genie default 'ik' solves 2/30 on G2 offline; genie's best is multi_ik_adaptive)
  · extra algorithm g2_seeded_lm = the G2 arm_motion IK, for head-to-head timing on the robot
The IK still receives ONLY the measured start joints and the recorded TCP pose — never the
recorded target joints (except the explicit 'joint' replay baseline).

    source ~/agi@g2_robot/env_g2.sh
    ros2 run genie_robot_agibot_g2_motion pose_ab --arm right
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import select
import sys
import termios
import time
import tty

import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetStateValidity
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.utilities import remove_ros_args
from urdf_parser_py.urdf import URDF

from .differential_ik import ARMS, DifferentialIk
from .ik_lab.kinematics import ChainKin
from .ik_lab.algorithms import PlanningRequest, JointReplayRequest, discover
from .ik_lab.planning import MotionSettings, plan_motion

ARM_KEY = {'left': '1', 'right': '2'}
DEFAULT_ALGORITHM = 'g2_seeded_lm'   # sanofi 那套(arm_motion 同款)；genie 的是 multi_ik_adaptive
HELP = '''A/B 测试：A/B 录制；a/b 前往；c 开关往返循环；p 显示记录；空格 取消；q 取消并退出。
循环：a → 成功到达 → 停 3 s → b → 成功到达 → 停 3 s → 重复。
启动时自动发现算法；交互选择或用 --algorithm；--list-algorithms 只列出。
需要硬件 + MoveIt 在跑（只借用碰撞检查，不调 OMPL）。同一时间不许有别的程序控制这条手臂。
'''


class PoseAB(DifferentialIk):
    def __init__(self, args, algorithm):
        self.algorithm = algorithm
        super().__init__(args, node_name='g2_pose_ab')
        self.key = ARM_KEY[args.arm]
        self.side, self.root, self.tip, self.controller, self.arm_joints = ARMS[self.key]
        self.goal = None
        self.busy = False
        self.cancel_sent = False
        self.cycling = False
        self.cycle_target = 'a'
        self.cycle_due = math.inf
        self.action = ActionClient(self, FollowJointTrajectory,
                                   f'/{self.controller}/follow_joint_trajectory')
        self.validity = self.create_client(GetStateValidity, '/check_state_validity')
        self.create_timer(0.1, self.watch_feedback)
        self.path = Path(args.records.format(arm=args.arm)).expanduser()
        self.records = json.loads(self.path.read_text()) if self.path.exists() else {}
        self.say(f'手臂: {self.side}；记录文件: {self.path}；算法: {args.algorithm}')

    def on_description(self, message):
        if self.xml is None:
            self.xml = message.data

    def ready(self, stationary=True):
        self.require_ready(self.key)
        speed = self.velocity[self.key]
        if stationary and (not np.all(np.isfinite(speed)) or np.max(np.abs(speed)) > 0.02):
            raise ValueError('手臂还没停稳；等它停下再录制/规划')
        if self.key not in self.kin:
            kin = ChainKin(self.xml, self.root, self.tip)
            if kin.joint_names != self.arm_joints:
                raise ValueError('URDF 手臂关节顺序不符')
            self.kin[self.key] = kin
        if not hasattr(self, 'velocity_limits'):
            robot = URDF.from_xml_string(self.xml)
            self.required_joints = {j.name for j in robot.joints if j.type in ('revolute', 'continuous', 'prismatic') and j.mimic is None}
            self.velocity_limits = np.array([robot.joint_map[j].limit.velocity for j in self.arm_joints], dtype=float)
            if not np.all(np.isfinite(self.velocity_limits)) or np.min(self.velocity_limits) <= 0:
                raise ValueError('URDF 速度限位无效')
        return self.measured[self.key].copy()

    def wait(self, future, deadline):
        while not future.done() and rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.01)
        if not future.done():
            future.cancel()
            raise ValueError('计算/服务等待超时；这次不发轨迹')
        return future.result()

    def base_transform(self):
        # The arm-only KDL chain excludes the waist. Preserve base-frame target
        # by requiring the arm root transform to match the recording in v1.
        tf = self.tf_buffer.lookup_transform('base_link', self.root, rclpy.time.Time())
        stamp = rclpy.time.Time.from_msg(tf.header.stamp)
        if stamp.nanoseconds and (self.get_clock().now()-stamp).nanoseconds > 1e9:
            raise ValueError('手臂根 TF 过期')
        t, q = tf.transform.translation, tf.transform.rotation
        return np.array([t.x, t.y, t.z, q.x, q.y, q.z, q.w])

    def record(self, name):
        if self.busy:
            raise ValueError('执行中不能录制')
        q = self.ready()
        kin = self.kin[self.key]
        if np.any(q < kin.lower) or np.any(q > kin.upper):
            raise ValueError('实测关节超出 URDF 限位；不能录成目标')
        p, rot = kin.fk(q)
        record = dict(joints=q.tolist(), position=p.tolist(), rotation=rot.tolist(),
                      root_in_base=self.base_transform().tolist(),
                      model_sha256=hashlib.sha256(self.xml.encode()).hexdigest(),
                      root=self.root, tip=self.tip, joint_names=kin.joint_names)
        updated = dict(self.records, **{name: record})
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix('.tmp')
        temporary.write_text(json.dumps(updated, indent=2) + '\n')
        temporary.replace(self.path)
        self.records = updated
        self.say(f'已录 {name.upper()}: q={np.round(q, 4)} (rad)')

    def check_state(self, q, snapshot, deadline):
        request = GetStateValidity.Request()
        # Empty group checks the entire robot, including the other arm/body.
        request.robot_state.is_diff = True          # 保留附着物(纸板)，见 arm_motion.valid
        values = dict(snapshot)
        values.update(zip(self.arm_joints, map(float, q)))
        request.robot_state.joint_state.name = list(values)
        request.robot_state.joint_state.position = list(values.values())
        future = self.validity.call_async(request)
        try:
            valid = self.wait(future, deadline).valid
        finally:
            if future.cancelled():
                self.validity.remove_pending_request(future)
        return valid

    def move(self, name):
        if self.busy:
            raise ValueError('上一个动作还没完；按空格取消并等结果后再发新目标')
        if name not in self.records:
            raise ValueError(f'先按 {name.upper()} 录制')
        start_time = time.monotonic()
        deadline = start_time + self.args.plan_timeout
        q0 = self.ready()
        kin = self.kin[self.key]
        record = self.records[name]
        if record.get('model_sha256') != hashlib.sha256(self.xml.encode()).hexdigest():
            raise ValueError('机器人模型变了；请重新录 A/B')
        if record['joint_names'] != kin.joint_names or record['tip'] != self.tip:
            raise ValueError('记录与当前运动学链不符；请重新录')
        old_tf, now_tf = np.array(record['root_in_base']), self.base_transform()
        if np.linalg.norm(old_tf[:3] - now_tf[:3]) > 0.001 or abs(np.dot(old_tf[3:], now_tf[3:])) < 0.999999:
            raise ValueError('腰部动过，手臂根相对 base_link 变了；本版请重新录 A/B')
        if self.joints is None or time.monotonic() - self.joints_at > 1.0:
            raise ValueError('全身 /joint_states 缺失或过期')
        snapshot = self.joints.copy()
        if not self.required_joints.issubset(snapshot):
            raise ValueError('全身关节快照不完整；没法做整机碰撞检查')
        if not self.validity.service_is_ready() or not self.action.server_is_ready():
            raise ValueError(f'/check_state_validity 和 {self.controller} action 都要就绪')
        inputs = dict(
            start=q0.copy(),
            goal_position=np.array(record['position']),
            goal_rotation=np.array(record['rotation']), kin=kin, deadline=deadline,
            check_state=lambda q: self.check_state(q, snapshot, deadline),
            log=self.say, options=dict(self.args.algorithm_options),
            orientation_mode=self.algorithm.orientation_mode)
        if self.algorithm.requires_recorded_joints:
            stored = np.array(record['joints'], dtype=float)
            if stored.shape != (7,) or not np.all(np.isfinite(stored)):
                raise ValueError('关节回放基线的录制关节数据无效')
            request = JointReplayRequest(**inputs, goal_joints=stored.copy())
            self.say('关节回放基线：直接用录制的目标关节；这个结果不算 IK 解算能力')
        else:
            request = PlanningRequest(**inputs)
            self.say('位姿 IK 测试：只给当前实测关节 + 目标 TCP 位姿，不给录制的目标关节')
        settings = MotionSettings(self.args.velocity, self.args.acceleration,
                                  self.args.jerk, self.args.max_travel_deg)
        planned = plan_motion(self.algorithm, request, self.velocity_limits, settings)
        actual = self.ready()
        if np.max(np.abs(actual-q0)) > 0.01:
            raise ValueError('计算期间起点变了；再按一次目标键')
        other = [j for j in snapshot if j not in self.arm_joints]
        if time.monotonic()-self.joints_at > 1 or any(abs(self.joints.get(j, math.inf)-snapshot[j]) > 0.005 for j in other):
            raise ValueError('其它关节状态变了/过期；请重新计算')
        if time.monotonic() >= deadline:
            raise ValueError('超出计算预算；不发过期结果')
        goal = FollowJointTrajectory.Goal(trajectory=planned.trajectory)
        self.busy = True
        self.cancel_sent = False
        self.action.send_goal_async(goal).add_done_callback(self.accepted)
        self.say(f'计算用时 {time.monotonic()-start_time:.3f} s；已请求执行 {name}，时长 {planned.duration:.2f} s')

    def accepted(self, future):
        try:
            self.goal = future.result()
            if not self.goal.accepted:
                self.goal = None
                self.busy = False
                self.stop_cycle()
                self.say('JTC 拒绝了目标')
                return
            self.goal.get_result_async().add_done_callback(self.finished)
            if self.cancel_sent:
                self.goal.cancel_goal_async()
        except Exception as error:
            self.busy = False
            self.stop_cycle()
            self.say(f'目标提交失败: {error}')

    def finished(self, future):
        try:
            response = future.result()
            self.say(f'action status={response.status}, error_code={response.result.error_code}, {response.result.error_string}')
            if self.cycling:
                if response.status == 4 and response.result.error_code == 0:
                    self.cycle_target = 'b' if self.cycle_target == 'a' else 'a'
                    self.cycle_due = time.monotonic() + 3.0
                    self.say(f'已到达；停 3 s 后去 {self.cycle_target}')
                else:
                    self.stop_cycle()
        except Exception as error:
            self.stop_cycle()
            self.say(f'获取执行结果失败: {error}')
        finally:
            self.goal = None
            self.busy = False

    def cancel(self):
        self.stop_cycle()
        if self.busy and not self.cancel_sent:
            self.cancel_sent = True
            if self.goal is not None:
                self.goal.cancel_goal_async()
            self.say('已请求取消；等 JTC 返回结果（不是急停）')

    def stop_cycle(self):
        if self.cycling:
            self.say('往返循环已停止；不再安排后续段')
        self.cycling = False
        self.cycle_due = math.inf

    def toggle_cycle(self):
        if self.cycling:
            self.stop_cycle()
            return
        if self.busy:
            raise ValueError('等当前动作走完再开始循环')
        if not all(name in self.records for name in ('a', 'b')):
            raise ValueError('先录 A 和 B')
        self.ready()
        self.cycling = True
        self.cycle_target = 'a'
        self.cycle_due = time.monotonic()
        self.say('往返循环开始：先去 a；再按 c 停止后续循环，空格取消当前动作')

    def cycle_tick(self):
        # Called only by the keyboard loop, never by a ROS timer: move() spins
        # service futures and must not recursively start another planning pass.
        if self.cycling and not self.busy and time.monotonic() >= self.cycle_due:
            self.cycle_due = math.inf
            try:
                self.move(self.cycle_target)
            except Exception as error:
                self.stop_cycle()
                self.say(f'循环已停止；这次没执行: {error}')

    def watch_feedback(self):
        if self.busy and np.max(time.monotonic()-self.received[self.key]) > self.args.stale_after:
            self.cancel()


def main():
    parser = argparse.ArgumentParser(description=HELP)
    algorithms, unavailable = discover()
    for module, error in unavailable.items():
        print(f'算法模块不可用 {module}: {error}', file=sys.stderr)
    parser.add_argument('--arm', choices=['left', 'right'], default='right', help='测哪条手臂（默认 right，同 G1 原版）')
    parser.add_argument('--algorithm', '--mode', dest='algorithm', choices=list(algorithms),
                        help='选一个已注册算法；--mode 是旧别名；不给就交互选择')
    parser.add_argument('--list-algorithms', action='store_true', help='列出算法后退出，不连 ROS')
    parser.add_argument('--algorithm-options', default='{}', help='算法专属参数，JSON 对象')
    parser.add_argument('--records', default='~/.ros/g2_motion/{arm}_ab.json')
    parser.add_argument('--max-travel-deg', type=float, default=90.0,
                        help='单关节行程提示参考，度；只打印不拒绝')
    parser.add_argument('--velocity', type=float, default=float('inf'),
                        help='rad/s，关节速度上限（默认 inf = 直接用 URDF 限位 3.3~5.3）')
    parser.add_argument('--acceleration', type=float, default=0.3, help='rad/s²')
    parser.add_argument('--jerk', type=float, default=1.0, help='rad/s³')
    parser.add_argument('--plan-timeout', type=float, default=1.0)
    parser.add_argument('--stale-after', type=float, default=1.0)
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])
    if args.list_algorithms:
        for name, spec in algorithms.items():
            print(f'{name}: {spec.description}')
        if not algorithms:
            parser.exit(1, '没有可用算法\n')
        return
    if not algorithms:
        parser.error('没有可用算法；看上面的模块加载错误')
    try:
        args.algorithm_options = json.loads(args.algorithm_options)
    except json.JSONDecodeError as error:
        parser.error(f'--algorithm-options 不是合法 JSON: {error}')
    if not isinstance(args.algorithm_options, dict):
        parser.error('--algorithm-options 必须是 JSON 对象')
    for field in ['max_travel_deg', 'velocity', 'acceleration', 'jerk', 'plan_timeout', 'stale_after']:
        if not math.isfinite(getattr(args, field)) or getattr(args, field) <= 0:
            parser.error(f'{field} 必须是正的有限数')
    if not sys.stdin.isatty():
        parser.error('请在交互终端里运行')
    if args.algorithm is None:
        names = list(algorithms)
        default = DEFAULT_ALGORITHM if DEFAULT_ALGORITHM in algorithms else names[0]
        print('可用算法：', flush=True)
        for index, name in enumerate(names, 1):
            print(f'  {index}. {name}: {algorithms[name].description}', flush=True)
        try:
            selected = input(f'输入编号或名字，回车用 {default}: ').strip()
        except (KeyboardInterrupt, EOFError):
            return
        if not selected:
            args.algorithm = default
        elif selected.isdecimal() and 1 <= int(selected) <= len(names):
            args.algorithm = names[int(selected)-1]
        elif selected in algorithms:
            args.algorithm = selected
        else:
            parser.error('算法选择无效')
    try:
        algorithm = algorithms[args.algorithm].factory()
    except Exception as error:
        parser.error(f'算法初始化失败: {error}')
    rclpy.init()
    node = PoseAB(args, algorithm)
    fd = sys.stdin.fileno()
    previous = termios.tcgetattr(fd)
    print(HELP, flush=True)
    try:
        tty.setcbreak(fd)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            node.cycle_tick()
            if not select.select([fd], [], [], 0)[0]:
                continue
            key = os.read(fd, 1).decode(errors='ignore')
            try:
                if key in ['A', 'B']:
                    if node.cycling:
                        raise ValueError('先按 c 停止循环再录制')
                    node.record(key.lower())
                elif key in ['a', 'b']:
                    if node.cycling:
                        raise ValueError('循环中不接受手动目标；先按 c 停止循环')
                    node.move(key)
                elif key == 'c':
                    node.toggle_cycle()
                elif key == 'p':
                    node.say(json.dumps(node.records, indent=2))
                elif key == ' ':
                    node.cancel()
                elif key == 'q':
                    break
            except Exception as error:
                node.say(f'这次没执行: {error}')
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, previous)
        node.cancel()
        until = time.monotonic()+2.0
        while node.busy and rclpy.ok() and time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.05)
        if node.busy:
            node.say('取消还没确认；请在硬件侧检查状态')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
