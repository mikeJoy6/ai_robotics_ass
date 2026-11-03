"""G2 dual-arm Cartesian keyboard jog, executed by existing JTC controllers.

Ported from g1_pointcloud_grasping genie_motion/differential_ik.py. Changes for G2 only:
  · chains arm_base_link -> arm_{l,r}_end_link, joints idx2x/idx6x, controllers {left,right}_arm_group_controller
  · feedback from /joint_states (joint_state_broadcaster, fed by the G2 hardware plugin from /gdk/joint_state)
    instead of G1's /hal/{left,right}_arm_data
Everything else (per-press local IK solve_pose, target-vs-measured lag guard, --frame modes) is unchanged.

    source ~/agi@g2_robot/env_g2.sh
    ros2 run genie_robot_agibot_g2_motion differential_ik
"""

import argparse
import math
import os
import select
import sys
import termios
import time
import tty

import numpy as np
import PyKDL as kdl
import rclpy
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import ListControllers
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .ik_lab.kinematics import ChainKin, rotation_about, rotation_vector, solve_pose


# TCP = arm_{l,r}_end_link (G2 URDF arm tip). Chain roots are arm-only (arm_base_link sits on the
# waist); --frame selects the Cartesian command axes.
ARMS = {
    '1': ('left', 'arm_base_link', 'arm_l_end_link', 'left_arm_group_controller',
          [f'idx2{i}_arm_l_joint{i}' for i in range(1, 8)]),
    '2': ('right', 'arm_base_link', 'arm_r_end_link', 'right_arm_group_controller',
          [f'idx6{i}_arm_r_joint{i}' for i in range(1, 8)]),
}
KEYS = {'w': (0, 1), 's': (0, -1), 'a': (1, 1), 'd': (1, -1),
        'r': (2, 1), 'f': (2, -1), 'u': (3, 1), 'o': (3, -1),
        'i': (4, 1), 'k': (4, -1), 'j': (5, 1), 'l': (5, -1)}
HELP = '''
1 / 2: 选左 / 右臂；再按一次同一个数字 = 从实测位置重新同步（选臂不会动）
w/s: +X/-X    a/d: +Y/-Y    r/f: +Z/-Z
u/o: 绕 +X/-X    i/k: 绕 +Y/-Y    j/l: 绕 +Z/-Z
空格: 重发当前目标    p: 显示目标与实测 TCP    q: 退出
默认每按一次：平移 1 cm，旋转 0.05 rad；JTC 在 0.4 s 内到达。
默认 mixed：平移沿 base_link 轴（x前 y左 z上），旋转绕目标 TCP 自身轴、TCP 位置不动。
TCP = arm_l_end_link / arm_r_end_link；--frame 可改成统一的轴系。
退出只是停止读键；已经发出的轨迹 JTC 仍会执行完。
'''


class DifferentialIk(Node):
    def __init__(self, args, node_name='g2_differential_ik'):
        super().__init__(node_name)
        self.args = args
        self.xml = None
        self.kin = {}
        self.measured = {key: np.full(7, np.nan) for key in ARMS}
        self.received = {key: np.full(7, -np.inf) for key in ARMS}
        self.velocity = {key: np.full(7, np.inf) for key in ARMS}
        self.joints = None              # full /joint_states snapshot (pose_ab collision checks)
        self.joints_at = -np.inf
        self.target = {}
        self.published_at = {key: -np.inf for key in ARMS}
        self.mode = None
        self.active = set()
        self.controllers_at = -np.inf
        self.pending = None
        self.pending_at = 0.0
        self.pubs = {key: self.create_publisher(
            JointTrajectory, f'/{cfg[3]}/joint_trajectory', 1) for key, cfg in ARMS.items()}
        # BEST_EFFORT：env_g2.sh 加载的 GDK FastDDS 配置把默认发布者设成 BEST_EFFORT，joint_state_broadcaster 也
        #   跟着变 BEST_EFFORT；RELIABLE 订阅收不到它。BEST_EFFORT 订阅两种发布者都能收。
        self.create_subscription(JointState, '/joint_states', self.on_joint_states, qos_profile_sensor_data)
        self.description_subscription = self.create_subscription(
            String, '/robot_description', self.on_description,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.controller_client = self.create_client(
            ListControllers, '/controller_manager/list_controllers')
        self.create_timer(0.5, self.poll_controllers)

    def say(self, text):
        print(text, flush=True)

    def on_description(self, message):
        if self.xml is None:
            self.xml = message.data
            self.say('已收到 /robot_description；按 1 或 2 选臂。')

    def on_joint_states(self, message):
        now = time.monotonic()
        index = {name: i for i, name in enumerate(message.name)}
        for key, cfg in ARMS.items():
            for j, name in enumerate(cfg[4]):
                i = index.get(name)
                if i is None or i >= len(message.position) or not math.isfinite(message.position[i]):
                    continue
                self.measured[key][j] = message.position[i]
                self.received[key][j] = now
                if i < len(message.velocity):
                    self.velocity[key][j] = message.velocity[i]
        if len(message.name) == len(message.position) and all(map(math.isfinite, message.position)):
            self.joints = dict(zip(message.name, message.position))
            self.joints_at = now

    def poll_controllers(self):
        if self.pending is not None:
            if self.pending.done():
                try:
                    self.active = {c.name for c in self.pending.result().controller if c.state == 'active'}
                    self.controllers_at = time.monotonic()
                except Exception as error:
                    self.active.clear()
                    self.get_logger().warning(f'查询控制器失败: {error}')
                self.pending = None
            elif time.monotonic() - self.pending_at > 2.0:
                self.controller_client.remove_pending_request(self.pending)
                self.pending = None
                self.active.clear()
            return
        if self.controller_client.service_is_ready():
            self.pending = self.controller_client.call_async(ListControllers.Request())
            self.pending_at = time.monotonic()

    def require_ready(self, key):
        if self.xml is None:
            raise ValueError('还没收到 /robot_description；先启动 hardware.launch.py')
        if (time.monotonic() - self.controllers_at > 2.0 or ARMS[key][3] not in self.active):
            raise ValueError(f'{ARMS[key][3]} 还没确认 active；稍等或检查 hardware launch')
        age = time.monotonic() - self.received[key]
        stale = [name for name, seconds in zip(ARMS[key][4], age) if seconds > self.args.stale_after]
        if stale:
            raise ValueError(f'/joint_states 缺失/过期: {", ".join(stale)}；不发新命令')

    def select_arm(self, key):
        self.require_ready(key)
        if key not in self.kin:
            _, root, tip, _, joints = ARMS[key]
            kin = ChainKin(self.xml, root, tip)
            if kin.joint_names != joints:
                raise ValueError(f'URDF 链关节顺序不符: {kin.joint_names} != {joints}')
            self.kin[key] = kin
        self.mode = key
        self.target[key] = self.measured[key].copy()
        axes = '平移 base_link / 旋转绕目标 TCP 自身轴' if self.args.frame == 'mixed' else self.args.frame
        self.say(f'已选{"左" if ARMS[key][0] == "left" else "右"}臂，目标已同步到实测位置；轴系: {axes}')
        self.describe()

    def describe(self):
        if self.mode is None:
            return
        key = self.mode
        self.require_ready(key)
        kin = self.kin[key]
        p, _ = kin.fk(self.measured[key])
        target, _ = kin.fk(self.target[key])
        self.say(f'{ARMS[key][2]} @ {ARMS[key][1]}: 实测 {np.round(p, 4)} m；目标 {np.round(target, 4)} m')

    def frame_rotation(self, key, rot_target):
        """Return R_root_frame: requested axes expressed in the Jacobian's root."""
        root = ARMS[key][1]
        frame_name = 'base_link' if self.args.frame == 'mixed' else self.args.frame
        if frame_name in ('root', root):
            return np.eye(3)
        if frame_name == 'tool':
            return rot_target
        try:
            transform = self.tf_buffer.lookup_transform(root, frame_name, Time())
        except TransformException as error:
            raise ValueError(f'拿不到 TF {root} <- {frame_name}: {error}') from error
        stamp = transform.header.stamp
        if stamp.sec or stamp.nanosec:  # A static transform has zero time.
            age = (self.get_clock().now() - Time.from_msg(stamp)).nanoseconds / 1e9
            if age > self.args.stale_after or age < -0.1:
                raise ValueError(f'TF 过期或时钟不同步: age={age:.3f}s')
        q = transform.transform.rotation
        rot = kdl.Rotation.Quaternion(q.x, q.y, q.z, q.w)
        return np.array([[rot[r, c] for c in range(3)] for r in range(3)])

    def jog(self, axis, sign):
        if self.mode is None:
            raise ValueError('先按 1 或 2 选臂')
        key = self.mode
        self.require_ready(key)
        q0 = self.target[key]
        lag = float(np.max(np.abs(q0 - self.measured[key])))
        if lag > self.args.max_joint_step:
            if time.monotonic() - self.published_at[key] < self.args.move_time + 0.3:
                raise ValueError(f'还没跟上上一个目标（差 {lag:.3f} rad）；等当前动作走完')
            self.target[key] = q0 = self.measured[key].copy()
            self.say('上一个目标没到位；已从实测位置重新同步。')
        position, rotation = self.kin[key].fk(q0)
        position_target = position.copy()
        rotation_target = rotation.copy()
        if axis < 3:
            frame = self.frame_rotation(key, rotation)
            delta = np.zeros(3)
            delta[axis] = sign * self.args.step
            position_target += frame @ delta
        else:
            delta = rotation_about(axis - 3, sign * self.args.rotation_step)
            if self.args.frame == 'mixed':
                # Body-axis increment; position_target stays at the TCP pivot.
                rotation_target = rotation @ delta
            else:
                frame = self.frame_rotation(key, rotation)
                rotation_target = frame @ delta @ frame.T @ rotation
        result = solve_pose(self.kin[key], q0, position_target, rotation_target,
                            damping=self.args.damping)
        if not result.converged:
            raise ValueError(f'局部 IK 没收敛：位置差 {result.position_error*1000:.2f} mm，'
                             f'姿态差 {result.rotation_error:.4f} rad；不发')
        largest = float(np.max(np.abs(result.q - q0)))
        if largest > self.args.max_joint_step:
            raise ValueError(f'关节变化 {largest:.3f} rad 超过 {self.args.max_joint_step:.3f}；不发')
        self.publish_target(key, result.q)
        self.target[key] = result.q
        self.say(f'已发{"左" if ARMS[key][0] == "left" else "右"}臂：IK {result.iterations} 轮，最大关节增量 {largest:.4f} rad；'
                 f'末端位移 {np.round(position_target-position, 4)} m，'
                 f'旋转 {np.round(rotation_vector(rotation, rotation_target), 4)} rad')

    def publish_target(self, key, q):
        self.require_ready(key)  # Recheck after IK, including feedback freshness.
        if not np.all(np.isfinite(q)):
            raise ValueError('目标含非有限值')
        ahead = float(np.max(np.abs(q - self.measured[key])))
        if ahead > self.args.max_joint_step:
            raise ValueError(f'新目标领先实测 {ahead:.3f} rad；等机器人跟上，这次按键不累计')
        if self.pubs[key].get_subscription_count() == 0:
            raise ValueError('JTC 命令话题没有订阅者')
        nanoseconds = round(self.args.move_time * 1e9)
        point = JointTrajectoryPoint(
            positions=[float(v) for v in q],
            time_from_start=Duration(sec=nanoseconds // 10**9, nanosec=nanoseconds % 10**9))
        trajectory = JointTrajectory(joint_names=ARMS[key][4], points=[point])
        self.pubs[key].publish(trajectory)
        self.published_at[key] = time.monotonic()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--frame', default='mixed',
                        help='默认 mixed：平移沿 base_link、旋转绕目标 TCP 自身轴；也可 base_link、root、tool 或任意 TF 名')
    parser.add_argument('--step', type=float, default=0.01, help='每按一次平移量, m')
    parser.add_argument('--rotation-step', type=float, default=0.05, help='每按一次旋转量, rad')
    parser.add_argument('--move-time', type=float, default=0.4, help='每个目标的执行时间, s')
    parser.add_argument('--max-joint-step', type=float, default=0.15, help='每按一次/领先实测 的最大关节差, rad')
    parser.add_argument('--stale-after', type=float, default=1.0, help='反馈/TF 过期阈值, s')
    parser.add_argument('--damping', type=float, default=0.05, help='DLS 阻尼 lambda')
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])
    for name in ('step', 'rotation_step', 'move_time', 'max_joint_step', 'stale_after', 'damping'):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0:
            parser.error(f'--{name.replace("_", "-")} 必须是正的有限数')
    if not sys.stdin.isatty():
        parser.error('请在交互终端里用 ros2 run 运行；键盘节点需要 TTY')
    rclpy.init()
    node = DifferentialIk(args)
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    print(HELP, flush=True)
    try:
        tty.setcbreak(fd)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            if not select.select([fd], [], [], 0)[0]:
                continue
            keys = os.read(fd, 256).decode(errors='ignore')
            if 'q' in keys:
                break
            for key in keys:
                rclpy.spin_once(node, timeout_sec=0.0)
                try:
                    if key in ARMS:
                        node.select_arm(key)
                    elif key in KEYS:
                        node.jog(*KEYS[key])
                    elif key == 'p':
                        node.describe()
                    elif key == ' ' and node.mode is not None:
                        node.publish_target(node.mode, node.target[node.mode])
                        node.say('已重发当前目标。')
                except (ValueError, np.linalg.LinAlgError) as error:
                    node.say(str(error))
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
