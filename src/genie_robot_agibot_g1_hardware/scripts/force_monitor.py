#!/usr/bin/env python3
# force_monitor.py —— 六维力实时监视小工具（G1 HAL 层诊断，不参与任何技能流程）
#
# 背景（DEVLOG「六维力开发」）：`ros2 topic echo /hal/left_arm_data` 每条要刷 7 个电机的
#   全部字段，六维力那 6 个数被淹在几十行里，根本没法看"手推一下变多少"。这个脚本只订
#   force_data，原地刷新成 6 行条形图。
#
# ★为什么显示百分比而不是 N / Nm★：官方给的量程是 力 0~200 N、力矩 0~7 Nm——这两个数
#   量纲不同、数量级差 30 倍，直接看裸值没法横向比"哪一路先顶满"。换算成各自量程的百分比
#   后 6 路就在同一把尺子上了：谁先逼近 100%，谁就是先饱和的通道。
#   （这不是学术洁癖：传感器量程隐含的力臂是 7÷200 = 3.5 cm，而本机 TCP 在 Link7 外 14.3 cm，
#     指尖受力时 7 Nm ÷ 0.143 m ≈ 49 N 力矩就顶满了，力通道那时才用掉 24%。
#     看裸值会误以为"才 49N，离 200N 远得很"，看百分比一眼就能看见力矩条已经撞墙。）
#   允许超过 100%：超量程不截断，数字照常往上涨（300 N 就显示 150%）。
#
# ★条形图另有一把刻度（BAR_FULL_PCT，默认 25%）★：数字列是真·占量程，但条形图满格只对应
#   量程的 25%。因为实测真机日常数据只占量程 2~3%，按量程画条子永远是一条平线，白给。
#   200 N / 7 Nm 是"撞坏机器"级别的量程，正常操作压根够不着。条子冲出显示范围时画 « / »，
#   那只表示"超出当前显示刻度"，不是超量程 —— 真超量程看数字列。刻度可用 --full 现场改。
#
# 采样 vs 刷新：话题 100 Hz，终端没必要跟着重绘 100 次。这里回调只做"记录 + 更新峰值"，
#   重绘交给 20 Hz 定时器——峰值在回调里逐帧更新，所以两次重绘之间的瞬间尖峰不会被漏掉。
#
# 用法：
#   ros2 run genie_robot_agibot_g1_hardware force_monitor.py            # 默认左臂
#   ros2 run genie_robot_agibot_g1_hardware force_monitor.py --arm right
#   ros2 run genie_robot_agibot_g1_hardware force_monitor.py --topic /hal/left_arm_data
#   Ctrl-C 退出。

import argparse
import math
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.utilities import remove_ros_args

from genie_msgs.msg import ArmState


# ── 官方量程（GDK v1.5.0 使用指南）───────────────────────────────
FORCE_RANGE_N   = 200.0   # x/y/z 轴力
TORQUE_RANGE_NM = 7.0     # rx/ry/rz 轴力矩

# force_data 的 6 个槽位：(显示名, 该路的满量程)
AXES = (
    ("Fx", FORCE_RANGE_N),
    ("Fy", FORCE_RANGE_N),
    ("Fz", FORCE_RANGE_N),
    ("Mx", TORQUE_RANGE_NM),
    ("My", TORQUE_RANGE_NM),
    ("Mz", TORQUE_RANGE_NM),
)

# force_coordinate 查表（DEVLOG 记的当前值是 0）
COORD_NAMES = {0: "传感器系", 1: "工具系", 2: "基座系"}

# arm_state 查表（全表见 genie_msgs/msg/arm/ArmState.msg 的注释）。
#   ★为什么顺手加这个★：手臂"撞限位就断电保护"到底是哪一类故障，机器人自己会报——
#   与其靠回忆猜是哪个关节、猜该收到多少度（DEVLOG 里那次改 URDF 限位反而让 IK 换了构型、
#   手臂翻到头顶），不如让它直接说。ArmState 跟 force_data 在同一条消息里，白拿。
#   这里只列跟"限位/碰撞/掉使能"有关的；其余码原样显示十六进制。
ARM_STATE_NAMES = {
    0x0000: "正常(控制链路未建立)",
    0x0001: "正常(控制链路已建立)",
    0x1002: "★目标角度超过限位★",
    0x1003: "奇异点不可达",
    0x1007: "关节超速",
    0x100D: "★机械臂发生碰撞★",
    0x1010: "★关节掉使能★",        # 多半就是俗称的"断电保护"
    0x1012: "★自碰撞★",
    0x1014: "★超关节软限位★",
    0x2003: "六维力模块异常",
}

HALF = 22          # 条形图单侧格数（总宽 2*HALF，中间一根 | 是零位）
REDRAW_HZ = 20.0   # 终端重绘频率
LINES = 12         # 一屏占几行（重绘时要往上退这么多行，改版式记得同步改）

# ★条形图满格 = 量程的百分之几★（数字列仍然是真·占量程百分比，只有条子放大）
#   实测真机静置/手推的数据只占量程 2~3%（DEVLOG「六维力测试脚本」那屏），条子几乎是平的、
#   一点变化都看不出来 —— 200 N / 7 Nm 是"撞坏机器"级别的量程，日常操作根本用不到那个量级。
#   所以把条形图单独放大：满格对应量程的 BAR_FULL_PCT%，超过就画 « / »（那只是"超出显示范围"，
#   不是超量程，真超量程看数字列）。可用 --full 现场改，不用重编。
BAR_FULL_PCT = 25.0


def color_of(pct):
    """上色阈值跟着【条形图刻度】走：<半格 绿、<满格 黄、冲出显示范围 亮红。
    ★为什么不按量程上色★：按量程的话（<50%绿）实测值永远是绿的，这一列就废了；
    绑到条形图刻度后，颜色和条长说的是同一件事，红=冲出当前显示范围、该调 --full 了。"""
    a = abs(pct)
    if a >= BAR_FULL_PCT:
        return "\033[1;31m"
    if a >= BAR_FULL_PCT / 2.0:
        return "\033[33m"
    return "\033[32m"


def bar(pct):
    """中心零位的双向条：负值向左长、正值向右长。冲出显示范围时最外一格换成 « / »。"""
    n = int(round(abs(pct) / BAR_FULL_PCT * HALF))
    over = n > HALF
    n = min(n, HALF)
    fill = "█" * n
    if pct < 0:
        left = " " * (HALF - n) + fill
        right = " " * HALF
        if over:
            left = "«" + left[1:]
    else:
        left = " " * HALF
        right = fill + " " * (HALF - n)
        if over:
            right = right[:-1] + "»"
    return left + "|" + right


class ForceMonitor(Node):
    def __init__(self, topic):
        super().__init__("force_monitor")
        self.topic = topic

        self.force = None       # 最近一帧的 6 个原始数
        self.peak = [0.0] * 6   # 各路 |百分比| 的历史峰值（回调里逐帧更新，不受重绘频率影响）
        self.state = 0          # force_state：0 正常
        self.coord = 0          # force_coordinate
        self.arm_state = 0      # arm_state：手臂故障码（限位/碰撞/掉使能都在这）
        self.arm_worst = 0      # 见过的最严重(非正常)码 —— 故障常常一闪而过，不留一份就看不见了
        self.count = 0          # 收到的总帧数
        self._t0 = time.monotonic()
        self._n0 = 0
        self.hz = 0.0
        self._drawn = False     # 首屏之前不能往上退光标

        # 监视器只要最新值，用 BEST_EFFORT + depth=1：
        #   BEST_EFFORT 的订阅端能匹配 RELIABLE 的发布端（要求 ≤ 提供），反过来不行，所以这样最保险；
        #   depth=1 保证卡顿后拿到的是"现在"而不是补播一串旧帧。
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(ArmState, topic, self._on_arm, qos)
        self.create_timer(1.0 / REDRAW_HZ, self._redraw)

    def _on_arm(self, msg):
        """100 Hz 回调：只存值 + 更新峰值，不打印。"""
        if len(msg.force_data) < 6:
            return
        self.force = list(msg.force_data[:6])
        self.state = msg.force_state
        self.coord = msg.force_coordinate
        self.arm_state = msg.arm_state
        # 故障码常常只闪一帧（报完就被清），20Hz 重绘必然漏掉 —— 所以在 100Hz 回调里锁存，
        #   跟 peak 是同一个道理：要看的东西发生在两次重绘之间。
        if msg.arm_state not in (0x0000, 0x0001):
            self.arm_worst = msg.arm_state
        self.count += 1
        for i, (_, full) in enumerate(AXES):
            p = abs(self.force[i]) / full * 100.0
            if p > self.peak[i]:
                self.peak[i] = p

    def _redraw(self):
        lines = self._render()
        out = []
        if self._drawn:
            out.append("\033[%dA" % LINES)     # 光标退回本屏第一行
        for ln in lines:
            out.append("\033[2K" + ln + "\n")  # 逐行清掉旧内容再写，避免残留
        sys.stdout.write("".join(out))
        sys.stdout.flush()
        self._drawn = True

    @staticmethod
    def _arm_txt(code):
        """arm_state → 带色文字。正常绿，其余一律亮红（限位/碰撞/掉使能都是要立刻看见的东西）。"""
        name = ARM_STATE_NAMES.get(code, "未知码")
        if code in (0x0000, 0x0001):
            return "\033[32m0x%04X %s\033[0m" % (code, name)
        return "\033[1;31m0x%04X %s\033[0m" % (code, name)

    def _arm_latch_txt(self):
        """锁存的历史故障：没出过就留白，出过就一直挂着（故障闪一下就恢复的话，现值已经看不到了）。"""
        if not self.arm_worst:
            return ""
        return "\033[1;33m[本次运行出现过 %s\033[1;33m]\033[0m" % self._arm_txt(self.arm_worst)

    def _render(self):
        # 帧率：拿两次重绘之间的实际增量算，反映的是真实到达率而不是标称 100 Hz
        now = time.monotonic()
        dt = now - self._t0
        if dt >= 1.0:
            self.hz = (self.count - self._n0) / dt
            self._t0, self._n0 = now, self.count

        coord_txt = COORD_NAMES.get(self.coord, "未知%d" % self.coord)
        state_txt = ("\033[32m正常\033[0m" if self.state == 0
                     else "\033[1;31m异常(force_state=%d)\033[0m" % self.state)

        L = []
        L.append("\033[1m六维力实时监视\033[0m  %s   量程: 力 %.0f N / 力矩 %.0f Nm"
                 "   \033[36m条形图满格=量程 %g%%\033[0m"
                 % (self.topic, FORCE_RANGE_N, TORQUE_RANGE_NM, BAR_FULL_PCT))
        L.append("传感器 %s   坐标系 %d(%s)   帧 %d   %.0f Hz"
                 % (state_txt, self.coord, coord_txt, self.count, self.hz))
        L.append("手臂 %s   %s" % (self._arm_txt(self.arm_state), self._arm_latch_txt()))
        # 表头刻度要跟数据行的条形图对齐：数据行前缀 "  %-3s  %+7.2f%%  " 恰好 17 个字符，
        # 所以左标签从第 17 列起，0 落在中线(17+HALF)，右标签右端顶到条尾(17+2*HALF)。
        # 空格数由 HALF 和标签实际长度推出来 —— 改条宽或改 BAR_FULL_PCT（标签会从 "-100%"
        # 变成 "-25%"，少一个字符）都不用手动数格子。
        lo = "-%g%%" % BAR_FULL_PCT
        hi = "+%g%%" % BAR_FULL_PCT
        L.append(" " * 7 + "占量程" + " " * 4
                 + lo + " " * (HALF - len(lo)) + "0" + " " * (HALF - len(hi)) + hi
                 + "     峰值")

        if self.force is None:
            L.append("  \033[33m等待 %s 数据…（确认 HAL 已启动）\033[0m" % self.topic)
            L += [""] * (LINES - len(L))
            return L[:LINES]

        for i, (name, full) in enumerate(AXES):
            pct = self.force[i] / full * 100.0
            c = color_of(pct)
            L.append("  %-3s  %s%+7.2f%%\033[0m  %s%s\033[0m  %6.1f%%"
                     % (name, c, pct, c, bar(pct), self.peak[i]))

        # 合力 / 合力矩：6 路各自不超量程，合起来也可能已经很吃紧，单看某一路会漏判
        fx, fy, fz, mx, my, mz = self.force
        fmag = math.sqrt(fx * fx + fy * fy + fz * fz)
        mmag = math.sqrt(mx * mx + my * my + mz * mz)
        fp = fmag / FORCE_RANGE_N * 100.0
        mp = mmag / TORQUE_RANGE_NM * 100.0
        L.append("  " + "─" * (2 * HALF + 30))
        L.append("  合力 |F| %s%6.2f%%\033[0m      合力矩 |M| %s%6.2f%%\033[0m"
                 % (color_of(fp), fp, color_of(mp), mp))
        return L[:LINES]


def main():
    # bar()/color_of() 都读模块级 BAR_FULL_PCT，--full 改一次全局生效，不用到处传参。
    # global 必须写在函数最前面：下面 add_argument 的 default 就读了它，Python 不允许"先用后声明"。
    global BAR_FULL_PCT

    argv = remove_ros_args(sys.argv)[1:]
    ap = argparse.ArgumentParser(description="六维力实时监视（百分比显示）")
    ap.add_argument("--arm", choices=["left", "right"], default="left",
                    help="监视哪只手臂（默认 left）")
    ap.add_argument("--topic", default=None,
                    help="直接指定话题，给了就覆盖 --arm")
    ap.add_argument("--full", type=float, default=BAR_FULL_PCT,
                    help="条形图满格对应量程的百分之几（默认 %g；数字列不受影响，始终是真·占量程）"
                         % BAR_FULL_PCT)
    args = ap.parse_args(argv)
    topic = args.topic or ("/hal/%s_arm_data" % args.arm)

    if args.full <= 0.0:
        ap.error("--full 必须为正数")
    BAR_FULL_PCT = args.full

    rclpy.init()
    node = ForceMonitor(topic)
    sys.stdout.write("\033[?25l")            # 藏光标，免得在条形图里乱闪
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl-C 走 KeyboardInterrupt；被 kill / launch 关停走 ExternalShutdownException。
        # 两个都得接住，否则退出时会吐一段 traceback，把最后一屏数据冲掉。
        pass
    except Exception:
        # 关停竞态：context 已失效后 spin 还可能再摸一次 wait set，抛 RCLError。
        # 只在确实已经关停时吞掉，rclpy 还活着就照常抛出去，别把真 bug 盖了。
        if rclpy.ok():
            raise
    finally:
        sys.stdout.write("\033[?25h\n")      # 还原光标
        sys.stdout.flush()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
