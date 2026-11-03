#!/usr/bin/env python3
"""head_depth_cloud.py —— G2 头部深度图 → PointCloud2，喂给 MoveIt 的 PointCloudOctomapUpdater 做规划组避障。

G2 的 /gdk/camera/head_depth 只是一张裸 16UC1 深度图：frame_id 为空、没有 camera_info。本节点补齐：
  1. 静态 TF：head_link3 → head_color_optical_frame → head_depth_optical_frame（标定见 config/head_camera.yaml），
     这样点云挂在机器人 TF 树上，弯腰/转头时 MoveIt 用当时的 TF 把点放回 base_link，地图不跟着相机动。
  2. 按内参把深度反投影成点（光学系：x 右 y 下 z 前），每 stride 个像素取 1 个、限频 rate。
  3. 时间戳 = 机器人给图像打的 stamp + 关节数据的"收到−stamp"延迟(中位数，实测约 107ms)。
     /joint_states 由 JSB 按本机收到 /gdk/joint_state 的时刻打戳，TF 也就是"本机收到关节"的时间轴；
     图像的机器人 stamp 加上同一个偏移，就落到这条时间轴上，和拍摄瞬间的手臂/头/腰姿态对齐（时钟偏差相消）。
     以前用"本机收到图像的时刻"，比拍摄晚了约 36ms(图像比关节多走 36ms 网络)，手臂运动时自身过滤会对不上。
     还没收到 /gdk/joint_state 时退回用收到时刻。
  4. 末端 link 附近 ee_filter_radius 球内的点删掉：URDF 没有手/夹爪模型，MoveIt 的自身过滤滤不掉手。
  5. 散斑过滤（全分辨率）：3x3 邻域里深度相近(±speckle_rel)的像素少于 speckle_min_neighbors 个就丢——
     深度相机每帧有零星随机噪点，一个就能在空中凭空点亮一个 octomap 格子。
  6. 超过 max_depth 的像素不丢，沿同一条射线放到 clear_distance（> MoveIt max_range）：MoveIt 对超出
     max_range 的点只清空射线、不记占据。以前直接丢掉 → 看向远处/天花板的方向永远没有射线，
     那个方向上被噪点点亮的格子永远清不掉（2026-11-02 头前 0.47m 浮着的两个格子就是这样，实测像素深度 16~30m）。
     深度 0（没回波）仍然丢：可能太近也可能太远，不能当成"空"。
  7. 悬空小块过滤（在发布网格上做）：比周围 floater_window×floater_window 邻域中位深度还近 floater_ratio 倍、
     且连成的块 < floater_max_points 个采样点 → 整块丢。专治两类：
     · 头部深度相机固定坏点：像素(258~260,110~111) 每帧稳定报 0.381m(σ0.4mm)，彩色图那里是 3m 外的墙
       ——红外窗口上的灰/划痕；它在头前 0.4m 点亮的格子每帧被重新点亮，射线清不掉。
     · 近处偶发的几像素假读数。
     细杆/椅子腿也比周围近，但连成长条，块大于阈值，会保留。近处真实物体边缘误删约 1.4%(实测)。
  8. 背面补全不在这里做，挪到 shadow_cloud.py：它订阅 MoveIt 已经按 URDF 滤掉机器人的 /head_camera/filtered_points
     再补，手臂上的点永远不会被往后延长。以前在这里用"离手臂骨架 15cm"猜哪些是手臂，手臂运动时看到的肩/肘外壳
     超出 15cm 就会被补到手臂背后，那片格子被手臂自己挡着清不掉，手臂一过去 MoveIt 就判撞、执行中止（2026-11-02）。
  9. 裁掉画面四周 border_margin 像素：边缘深度很差（实测有效率 上 4 行 16%、左 4 列 3%、右 4 列 8%，
     时间抖动 上 2.5%、左 4%、右 12.8%，中间只有 0.43%）。边缘偶发的假读数点亮的格子落在视野最边上，
     那个方向有效射线很少，清不掉（2026-11-02 头顶前方两个橙色浮点就是投影在第 1~6 行）。
  10. 悬空块整块删（在生成射线/背面补全之前，所有下游都看不到它）：发布网格上相邻采样点 3D 距离
      < connect_dist + connect_rel·z 就算连着。连成的块满足任一条就整块保留，否则整块丢——空中真飘一个鸡蛋也删：
        · 有一点贴地（base_link z < ground_max_z）；桌上的箱子经桌面→桌腿→地面连通；
        · 碰到裁边后的画面边缘：视野外可能有支撑；
        · 有一点紧挨着比它近的东西（遮挡边）：支撑可能藏在遮挡物后面。快照里右臂挡住了右前方台面的下半截，
          没有这条，整个台面(665 点，离末端 18cm)会被当悬空删掉。
      只有四周看得见都在它后面、看得见它悬在空中的块才删：悬空噪点、镜头坏点、单独一截手臂（肩不在画面里）。
      代价：放在不回波的黑色/镜面桌面上的东西，支撑面测不到(深度 0 不算遮挡)，会被删掉（connect_filter: false 可关）。
  11. 多帧确认（最后一步，背面补全/MoveIt 都只看到确认过的点）：点按 base_link 里 confirm_res(=octomap 3cm)分格，
      一格在最近 confirm_frames 帧里至少 confirm_min 帧都有点才发；这帧没确认的点整个不发（也不拿来清空射线）。
      专治一闪一闪的东西：物体边缘的飞点、偶发假读数、手臂运动时没对上自身过滤的点——它们进了 octomap 后
      若落在手臂/物体背后，射线清不掉，再被背面补全往后延 25cm，几次往返就在箱顶上"增生"出一片（2026-11-02 实测）。
      真实静止物体每帧都在，只是新出现的东西晚 (confirm_min−1)/rate ≈0.4s 才进图。分格在 base_link：转头/弯腰不影响。
  12. 小块直接删（确认之后）：确认过的格在 3D 里按 26 邻域连成块，块小于 cluster_min_voxels 格的整块不发——
      只看大物品，零星的不管真假都删。已经进了 octomap 的零星格由 octomap_sweeper 清。
      连通容许中间空一格（cluster_reach=2）：stride 4 采样在 2m 处点距 ~2.6cm，快赶上 3cm 格，表面会断成碎块。
  13. 上面 10~12 只作用于 occupy_range(= sensors_3d.yaml max_range 2.5m) 以内的点。更远的点 MoveIt 不记占据、只用来
      清空射线；它们稀疏(3m 处点距 4cm > 格)又抖，以前被多帧确认/小块删除当零星点删掉，清空射线跟着没了——
      墙前面的陈旧格再也清不掉，地图越积越脏（2026-11-02 实测：陈旧且相机看得穿的格 892 个里 700 个没有任何射线经过；
      关掉这两道后剩 16 个）。删它们换不来任何好处，只会丢清空。
  14. 深度图上抠掉手臂（最先做，后面所有步骤都看不到）：手臂各 link 碰撞凸包投影成图像轮廓（外扩 self_mask_dilate_px），
      轮廓里测到的深度在 [该 link 最近点 − self_mask_margin, 最远点 + self_mask_far_margin] 的像素一律丢——手臂挡住的
      地方相机不可能看到它后面的东西。远在手臂后面的保留：手臂贴近镜头时几 cm 的模型/标定误差投影出来是几十像素，
      轮廓会罩到其实看得见的远处背景（实测罩住了 2.5m 外的墙），那不可能是手臂造成的。
      轮廓用 拍摄时刻−self_mask_lookback、拍摄时刻、最新 三个时刻的手臂位姿合起来画：运动中轮廓被拉长，盖住时间差。
      起因（2026-11-02 实测）：手臂挡在镜头前 0.24m（深度相机近距盲区），手臂表面被测到它后面 6~15cm（0.30~0.40m），
      离 URDF 凸包 5~20cm、MoveIt 自身过滤(+padding)管不到，再被背面补全往后延 25cm → 手臂后面一大块板，手臂挡着清不掉。
      比手臂明显更近的像素保留（真夹在相机和手臂之间的东西）。URDF 从 /robot_description 话题拿。
  15. 深度跳变处不可信，直接删（全分辨率，和散斑过滤同一步）：每个像素和隔 jump_step 像素的邻居(4 个方向)连成线段，
      线段和视线夹角 < jump_angle、且深度差 > jump_min_rel·深度 的——两点沿视线前后错开，是跳变边缘或中间的飞点——
      两个都删，再外扩 jump_dilate 像素。夹角排除斜看的地面/墙（相邻深度差大但和视线夹角十几~几十度）；
      跳变点在悬空块判断（第 10 条）之后才去掉：先去会切断物体和地面/桌腿的连接，整块表面被当悬空删掉（实测丢了一整块 1m 处的箱面）。
      深度差门限排除远处噪声（4m 处隔 2 像素横向才 2.5cm，深度随机抖几 cm 夹角就很小——第一版只看夹角，删了 47% 像素、整面墙）。
      只删 occupy_range 以内的（同第 13 条：更远的点只清空不记占据，删了只会丢清空射线）。
      手臂贴近镜头时那一片碎的近距假深度、手臂/物体轮廓上的飞点都在这里去掉。
手臂/身体本身由 MoveIt 按 URDF 碰撞模型过滤（sensors_3d.yaml 的 padding_offset），不在这里做。

订阅 BEST_EFFORT（GDK 相机话题就是 BEST_EFFORT）。
"""
from collections import deque

import numpy as np
import rclpy
from scipy import ndimage
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import String
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf2_ros import Buffer, ExtrapolationException, StaticTransformBroadcaster, TransformException, TransformListener
from gdk_msgs.msg import JointState as GdkJointState

from .robot_mask import depth_threshold, link_hulls
from .voxel_util import components, keys_of, quat_to_mat as _quat_to_mat


class HeadDepthCloud(Node):
    def __init__(self):
        super().__init__("head_depth_cloud")
        p = self.declare_parameter
        self.depth_topic = p("depth_topic", "/gdk/camera/head_depth").value
        self.cloud_topic = p("cloud_topic", "/head_camera/depth_points").value
        self.head_link = p("head_link", "head_link3").value
        self.color_frame = p("color_frame", "head_color_optical_frame").value
        self.depth_frame = p("depth_frame", "head_depth_optical_frame").value
        self.fx, self.fy, self.cx, self.cy = p("intrinsics", [309.359, 309.359, 319.0, 202.5]).value
        self.scale = p("depth_scale", 0.001).value
        color_in_head = p("color_in_head", [0.0] * 6 + [1.0]).value
        depth_in_color = p("depth_in_color", [0.0] * 6 + [1.0]).value
        self.stride = int(p("stride", 4).value)
        self.period = 1.0 / float(p("rate", 5.0).value)
        self.min_depth = float(p("min_depth", 0.25).value)
        self.margin = int(p("border_margin", 12).value)
        self.max_depth = float(p("max_depth", 4.0).value)
        self.clear_distance = float(p("clear_distance", 3.0).value)
        self.speckle_rel = float(p("speckle_rel", 0.05).value)
        self.speckle_min = int(p("speckle_min_neighbors", 2).value)
        self.jump_angle = float(p("jump_angle_deg", 10.0).value)
        self.jump_step = int(p("jump_step", 2).value)
        self.jump_min_rel = float(p("jump_min_rel", 0.05).value)
        self.jump_dilate = int(p("jump_dilate", 2).value)
        self.floater_ratio = float(p("floater_ratio", 0.6).value)
        self.floater_window = int(p("floater_window", 5).value)
        self.floater_max = int(p("floater_max_points", 4).value)
        self.connect_filter = bool(p("connect_filter", True).value)
        self.connect_dist = float(p("connect_dist", 0.04).value)
        self.connect_rel = float(p("connect_rel", 0.02).value)
        self.base_frame = p("base_frame", "base_link").value
        self.ground_max_z = float(p("ground_max_z", 0.03).value)
        self.ee_links = list(p("ee_links", ["arm_l_end_link", "arm_r_end_link"]).value)
        self.ee_radius = float(p("ee_filter_radius", 0.15).value)
        self.js_topic = p("joint_state_topic", "/gdk/joint_state").value
        self.self_mask = bool(p("self_mask", True).value)
        self.self_mask_links = list(p("self_mask_links", ["arm_"]).value)
        self.self_mask_margin = float(p("self_mask_margin", 0.05).value)
        self.self_mask_far = float(p("self_mask_far_margin", 0.30).value)
        self.self_mask_dilate = int(p("self_mask_dilate_px", 8).value)
        self.self_mask_lookback = float(p("self_mask_lookback", 0.1).value)
        self._hulls = None                    # {link: 凸包顶点}，收到 /robot_description 后建
        self.confirm_frames = int(p("confirm_frames", 4).value)
        self.confirm_min = int(p("confirm_min", 3).value)
        self.confirm_res = float(p("confirm_res", 0.03).value)
        self.cluster_min = int(p("cluster_min_voxels", 4).value)
        self.cluster_reach = int(p("cluster_reach", 2).value)
        self.occupy_range = float(p("occupy_range", 2.5).value)
        self._hist = deque(maxlen=max(0, self.confirm_frames - 1))   # 之前几帧的占据格（排好序的 int64 键）

        self.static_tf = StaticTransformBroadcaster(self)
        self.static_tf.sendTransform([
            self._tf(self.head_link, self.color_frame, color_in_head),
            self._tf(self.color_frame, self.depth_frame, depth_in_color),
        ])
        self.tf_buffer = Buffer()
        # TF 自己一个线程：图像回调里查 TF 时，TF 消息照样进来（单线程时阻塞等待会把 TF 也堵住，查询 100% 超时失败）
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.pub = self.create_publisher(PointCloud2, self.cloud_topic, 1)
        if self.self_mask:
            self.create_subscription(String, "/robot_description", self._on_urdf,
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Image, self.depth_topic, self._on_depth, qos_profile_sensor_data)
        self._js_delay = deque(maxlen=200)    # /gdk/joint_state 的 (本机收到 − 机器人 stamp)，秒
        # 关节延迟估计必须用"真实收到时刻"：放进独立回调组(配合 MultiThreadedExecutor)，不被图像处理拖晚
        self.create_subscription(GdkJointState, self.js_topic, self._on_js, qos_profile_sensor_data,
                                 callback_group=MutuallyExclusiveCallbackGroup())
        self._last = None
        self._rays = None                 # 缓存每个采样像素的 (x/z, y/z)
        self._warned_tf = set()
        self.get_logger().info(
            f"{self.depth_topic} → {self.cloud_topic}（{self.depth_frame}），stride {self.stride}，≤{1/self.period:.0f}Hz，"
            f"深度 {self.min_depth}~{self.max_depth}m(更远的放到 {self.clear_distance}m 只清空)，"
            f"裁边 {self.margin}px，散斑过滤 邻居≥{self.speckle_min}，悬空小块 <{self.floater_max}点，"
            f"悬空块删除 {'开' if self.connect_filter else '关'}(贴地 {self.base_frame} z<{self.ground_max_z}m 或碰画面边)，末端过滤半径 {self.ee_radius}m，"
            f"多帧确认 {self.confirm_min}/{self.confirm_frames} 帧({self.confirm_res*100:.0f}cm 格)，小于 {self.cluster_min} 格的块删，"
            f"时间戳对齐 {self.js_topic}")

    @staticmethod
    def _tf(parent, child, xyzq):
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = parent, child
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, xyzq[:3])
        q = t.transform.rotation
        q.x, q.y, q.z, q.w = map(float, xyzq[3:])
        return t

    def _ray_table(self, h, w):
        if self._rays is None or self._rays[0].shape != (len(range(0, h, self.stride)), len(range(0, w, self.stride))):
            v, u = np.mgrid[0:h:self.stride, 0:w:self.stride].astype(np.float32)
            self._rays = ((u - self.cx) / self.fx, (v - self.cy) / self.fy)
        return self._rays

    @staticmethod
    def _sec(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def _on_urdf(self, msg):
        try:
            self._hulls = link_hulls(msg.data, self.self_mask_links)
            self.get_logger().info(f"深度图抠手臂：{len(self._hulls)} 个 link（{', '.join(sorted(self._hulls))}）")
        except Exception as e:                               # URDF/网格读不了：不抠，照常发（宁可多留）
            self.get_logger().error(f"手臂碰撞模型加载失败，不做深度图抠手臂：{e}")

    def _arm_threshold(self, grid_shape, stamp):
        """发布网格上每格的手臂深度区间 (lo, hi)（见文件头第 14 条）；没有 URDF 时返回 None。"""
        if not self._hulls:
            return None
        times = [Time(nanoseconds=stamp.nanoseconds - int(self.self_mask_lookback * 1e9), clock_type=stamp.clock_type),
                 stamp, Time()]
        poses = []
        for link in self._hulls:
            for t in times:
                tr = self._lookup(self.depth_frame, link, t)
                if tr is None:
                    continue
                a, q = tr.transform.translation, tr.transform.rotation
                poses.append((link, np.array([a.x, a.y, a.z]), (q.x, q.y, q.z, q.w)))
        return depth_threshold(self._hulls, poses, grid_shape, self.fx, self.fy, self.cx, self.cy, self.stride,
                               self.self_mask_margin, self.self_mask_far, int(np.ceil(self.self_mask_dilate / self.stride)))

    def _on_js(self, msg):
        self._js_delay.append(self.get_clock().now().nanoseconds * 1e-9 - self._sec(msg.header.stamp))

    def _cloud_stamp(self, msg, now):
        """图像机器人 stamp + 关节数据延迟中位数 → 本机 TF 时间轴上的拍摄时刻；没有关节数据时用收到时刻。"""
        if len(self._js_delay) < 10 or self._sec(msg.header.stamp) <= 0:
            return now
        t = self._sec(msg.header.stamp) + float(np.median(self._js_delay))
        t = min(t, now.nanoseconds * 1e-9)                    # 不超过现在（TF 还没有未来的数据）
        return Time(nanoseconds=int(t * 1e9), clock_type=now.clock_type)

    def _lookup(self, target, source, stamp):
        """按拍摄时刻查 target←source；查不到返回 None（每对只警告一次）。"""
        try:
            try:
                return self.tf_buffer.lookup_transform(target, source, stamp)
            except ExtrapolationException:
                # 拍摄时刻比最新 TF 还新几 ms（rsp 还没发到）：退回最新 TF，误差只有这几 ms 的运动
                return self.tf_buffer.lookup_transform(target, source, Time())
        except TransformException as e:
            if (target, source) not in self._warned_tf:
                self.get_logger().warn(f"查不到 {target}←{source}：{e}")
                self._warned_tf.add((target, source))
            return None

    def _link_pos(self, link, stamp):
        """link 原点在深度光学系里的位置。"""
        tr = self._lookup(self.depth_frame, link, stamp)
        if tr is None:
            return None
        t = tr.transform.translation
        return np.array([t.x, t.y, t.z], dtype=np.float32)

    def _edge_mask(self, H, W):
        """采样网格上，裁边后仍有效的区域的最外一圈。"""
        r = np.arange(0, H, self.stride)
        c = np.arange(0, W, self.stride)
        ri, ci = np.nonzero((r >= self.margin) & (r < H - self.margin))[0], np.nonzero((c >= self.margin) & (c < W - self.margin))[0]
        e = np.zeros((len(r), len(c)), bool)
        e[ri[[0, -1]], ci[0]:ci[-1] + 1] = True
        e[ri[0]:ri[-1] + 1, ci[[0, -1]]] = True
        return e

    def _height_fn(self, stamp):
        """深度光学系的点 → base_link 里的 z。查不到 TF 返回 None。"""
        tr = self._lookup(self.base_frame, self.depth_frame, stamp)
        if tr is None:
            return None
        t, q = tr.transform.translation, tr.transform.rotation
        r = _quat_to_mat(q.x, q.y, q.z, q.w)[2]
        return lambda P: P @ r + t.z

    def _unsupported(self, X, valid, height, edge, z_all):
        """发布网格上的悬空块（见文件头第 10 条）。X: (h,w,3) 深度光学系的点，valid: 参与连通的格，
        edge: 裁边后画面最外一圈的格，z_all: 所有通过散斑过滤的深度(含太近/太远，0=无效)，用来找遮挡物。"""
        h, w = valid.shape
        idx = np.full((h, w), -1, np.int64)
        idx[valid] = np.arange(int(valid.sum()))
        n = int(valid.sum())
        if n == 0:
            return np.zeros((h, w), bool)
        z = X[..., 2]
        tol = self.connect_dist + self.connect_rel * z
        rows, cols = [], []
        for dy, dx in ((0, 1), (1, 0), (1, 1), (1, -1)):            # 8 邻域，每条边只建一次
            a = (slice(0, h - dy), slice(max(0, -dx), w - max(0, dx)))
            b = (slice(dy, h), slice(max(0, dx), w - max(0, -dx)))
            link = valid[a] & valid[b] & (np.linalg.norm(X[a] - X[b], axis=-1) < np.minimum(tol[a], tol[b]))
            rows.append(idx[a][link])
            cols.append(idx[b][link])
        r, c = np.concatenate(rows), np.concatenate(cols)
        _, lab = connected_components(coo_matrix((np.ones(len(r), np.int8), (r, c)), shape=(n, n)), directed=False)
        keep = np.zeros(lab.max() + 1, bool)
        keep[lab[height(X[valid]) < self.ground_max_z]] = True    # 贴地
        keep[lab[edge[valid]]] = True                                     # 碰画面边：视野外可能有支撑
        pad = np.pad(z_all, 1)
        behind = np.zeros((h, w), bool)
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                nb = pad[1 + dy:1 + dy + h, 1 + dx:1 + dx + w]
                behind |= (nb > 0) & (nb < z - tol)
        keep[lab[behind[valid]]] = True                                   # 紧挨遮挡物：支撑可能藏在后面
        out = np.zeros((h, w), bool)
        out[valid] = ~keep[lab]
        return out

    def _confirmed(self, P, stamp):
        """多帧确认 + 小块删除（见文件头第 11~13 条）：P 是深度光学系的点，返回保留掩码，并把这帧的占据格记进历史。
        只管 occupy_range 以内的点；更远的原样保留（MoveIt 只拿它们清空射线）。查不到 TF 时这帧不做、全部保留。"""
        keep = np.ones(len(P), bool)
        confirm = self.confirm_min > 1 and self.confirm_frames > 1
        if not confirm and self.cluster_min <= 1:
            return keep
        tr = self._lookup(self.base_frame, self.depth_frame, stamp)
        if tr is None:
            return keep
        near = np.linalg.norm(P, axis=1) <= self.occupy_range
        t, q = tr.transform.translation, tr.transform.rotation
        Pb = P[near] @ _quat_to_mat(q.x, q.y, q.z, q.w).T + np.array([t.x, t.y, t.z], np.float32)
        key = keys_of(Pb, self.confirm_res)
        ok = np.ones(len(key), bool)
        if confirm:
            count = np.ones(len(key), np.int32)                                   # 这帧自己算 1
            for h in self._hist:
                count += np.isin(key, h)
            self._hist.append(np.unique(key))
            ok = count >= self.confirm_min
        if self.cluster_min > 1 and ok.any():
            uk, inv = np.unique(key[ok], return_inverse=True)
            lab, size = components(uk, self.cluster_reach)
            ok[ok] = size[lab][inv] >= self.cluster_min
        keep[near] = ok
        return keep

    def _not_speckle(self, Z):
        """全分辨率：8 邻域里深度与自己相差 < speckle_rel·z 的有效像素 ≥ speckle_min 个才保留。"""
        if self.speckle_min <= 0:
            return Z > 0
        H, W = Z.shape
        pad = np.pad(Z, 1)
        tol = self.speckle_rel * Z
        similar = np.zeros((H, W), np.uint8)
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dy or dx:
                    nb = pad[1 + dy:1 + dy + H, 1 + dx:1 + dx + W]
                    similar += (nb > 0) & (np.abs(nb - Z) < tol)
        return (Z > 0) & (similar >= self.speckle_min)

    def _continuous(self, Z):
        """全分辨率：深度连续的像素才保留（见文件头第 15 条）。Z 里 0 = 无效。"""
        if self.jump_angle <= 0:
            return np.ones(Z.shape, bool)
        H, W = Z.shape
        X = ((np.arange(W, dtype=np.float32) - self.cx) / self.fx)[None, :] * Z
        Y = ((np.arange(H, dtype=np.float32) - self.cy) / self.fy)[:, None] * Z
        P = np.stack([X, Y, Z], axis=-1)
        sin_t = np.sin(np.radians(self.jump_angle))
        bad = np.zeros((H, W), bool)
        k = self.jump_step
        for dy, dx in ((0, k), (k, 0), (k, k), (k, -k)):
            a = (slice(0, H - dy), slice(max(0, -dx), W - max(0, dx)))
            b = (slice(dy, H), slice(max(0, dx), W - max(0, -dx)))
            d = P[b] - P[a]
            m = P[b] + P[a]                                    # 视线方向（两点中点，不用归一）
            dn = np.linalg.norm(d, axis=-1)
            sin = np.linalg.norm(np.cross(m, d), axis=-1) / np.maximum(np.linalg.norm(m, axis=-1) * dn, 1e-12)
            dz = np.abs(Z[b] - Z[a])
            veil = ((Z[a] > 0) & (Z[b] > 0) & (dz > self.jump_min_rel * np.minimum(Z[a], Z[b]))   # 深度差够大才算跳
                    & (sin < sin_t))                                                             # 且几乎顺着视线
            bad[a] |= veil
            bad[b] |= veil
        if self.jump_dilate > 0 and bad.any():
            bad = ndimage.binary_dilation(bad, iterations=self.jump_dilate)
        bad &= Z * np.sqrt(1.0 + (X / np.maximum(Z, 1e-6)) ** 2 + (Y / np.maximum(Z, 1e-6)) ** 2) <= self.occupy_range
        return ~bad                                            # 远点只清空不记占据，不删（同第 13 条）

    def _floaters(self, z):
        """发布网格上的悬空小块：比邻域中位深度近得多、且连成的块很小。无效(0)当成很远。"""
        if self.floater_max <= 0:
            return np.zeros(z.shape, bool)
        med = ndimage.median_filter(np.where(z > 0, z, 10.0), size=self.floater_window)
        cand = (z > 0) & (z < self.floater_ratio * med)
        lab, nl = ndimage.label(cand, structure=np.ones((3, 3)))
        if nl == 0:
            return np.zeros(z.shape, bool)
        sizes = ndimage.sum(cand, lab, index=np.arange(1, nl + 1))
        small = np.zeros(nl + 1, bool)
        small[1:] = sizes < self.floater_max
        return small[lab]

    def _on_depth(self, msg):
        now = self.get_clock().now()
        if self._last is not None and (now - self._last).nanoseconds * 1e-9 < self.period:
            return
        if msg.encoding != "16UC1":
            self.get_logger().error(f"不支持的深度编码 {msg.encoding}（只支持 16UC1）")
            return
        self._last = now
        stamp = self._cloud_stamp(msg, now)
        D = np.frombuffer(bytes(msg.data), np.uint16).reshape(msg.height, msg.step // 2)[:, :msg.width]
        Z = D.astype(np.float32) * self.scale
        if self.margin > 0:                                  # 边缘深度不可靠：当成没测到
            m = self.margin
            Z = Z.copy()
            Z[:m, :] = Z[-m:, :] = Z[:, :m] = Z[:, -m:] = 0.0
        keep = self._not_speckle(Z)[::self.stride, ::self.stride]
        smooth = self._continuous(Z)[::self.stride, ::self.stride]    # 跳变点：最后才去掉，不参与悬空块的连通判断
        z = np.where(keep, Z[::self.stride, ::self.stride], 0.0)
        keep &= ~self._floaters(z)
        if self.self_mask:                                                  # 深度图上抠掉手臂（第 14 条）
            band = self._arm_threshold(z.shape, stamp)
            if band is not None:
                keep &= ~((z > 0) & (z >= band[0]) & (z <= band[1]))
                z = np.where(keep, z, 0.0)
        rx, ry = self._ray_table(msg.height, msg.width)
        hit = keep & (z >= self.min_depth) & (z <= self.max_depth)          # 真实点：占据 + 清空射线
        if self.connect_filter and hit.any():
            height = self._height_fn(stamp)
            if height is not None:                                          # 查不到 TF 这帧不做（宁可多留）
                near = z * np.sqrt(rx * rx + ry * ry + 1.0) <= self.occupy_range   # 远点只清空，不删（见第 13 条）
                hit &= ~(self._unsupported(np.stack([rx * z, ry * z, z], axis=-1), hit, height,
                                           self._edge_mask(msg.height, msg.width), z) & near)
        hit &= smooth            # 跳变点在悬空块判断之后才去：先去会把物体和地面/桌腿的连接切断，整块被当悬空删掉
        far = keep & (z > self.max_depth)                                   # 远处：只用来清空射线
        z_far = np.full(int(far.sum()), self.clear_distance, np.float32)
        P_hit = np.stack([rx[hit] * z[hit], ry[hit] * z[hit], z[hit]], axis=1).astype(np.float32)
        P_far = np.stack([rx[far] * z_far, ry[far] * z_far, z_far], axis=1).astype(np.float32)
        if self.ee_radius > 0 and len(P_hit):
            for link in self.ee_links:
                c = self._link_pos(link, stamp)
                if c is not None:
                    P_hit = P_hit[np.sum((P_hit - c) ** 2, axis=1) > self.ee_radius ** 2]
        if len(P_hit):
            P_hit = P_hit[self._confirmed(P_hit, stamp)]
        pts = np.concatenate([P_hit, P_far]).astype(np.float32)

        cloud = PointCloud2()
        cloud.header.stamp = stamp.to_msg()
        cloud.header.frame_id = self.depth_frame
        cloud.height, cloud.width = 1, len(pts)
        cloud.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                        for i, n in enumerate("xyz")]
        cloud.is_bigendian, cloud.point_step = False, 12
        cloud.row_step, cloud.is_dense = 12 * len(pts), True
        cloud.data = pts.tobytes()
        self.pub.publish(cloud)


def main():
    rclpy.init()
    node = HeadDepthCloud()
    ex = MultiThreadedExecutor(num_threads=2)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
