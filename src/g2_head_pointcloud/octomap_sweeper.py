#!/usr/bin/env python3
"""octomap_sweeper.py —— 把 MoveIt octomap 里的零星格清掉：只看大物品，小块不管真假都删。

删哪些（pick_targets，2026-11-13 改）：octomap 格按 connect_reach 连通成块，下面两种整块清掉——
  · 小块：< min_voxels 格（原来只有这一条）
  · 悬空块：< floating_max_voxels 格，且整块最低格中心高于 support_z（base_link）——没有挨着地面，
    也没有和桌子、人、箱子这些大块连着。空中不会凭空飘着小东西，一定是假读数。
    原来只删 <4 格，可 shadow_cloud 把每个漏进来的点沿射线往后补 25cm（~8 格），一个假点就成了 8 格的“条”，
    永远够不上“零星”，被挡住 / 转头后不在视野里就再也清不掉（2026-11-13 用户反馈“空中飘着零星点云”）。
  floating_max_voxels 是“多大的悬空块算真东西”：举在空中、身体没被看到的人手（带背面补全）大约 50~150 格，
  默认 40 会保留它；调大会把这种也删掉（机器人就看不见它了），调小会漏删大一点的假块。

head_depth_cloud 已经在源头删小块（文件头第 12 条），但早先进来的、或背面补全/多帧叠加凑出来的零星格还会留在图里；
落在射线到不了的地方（被挡住、或在画面裁边区里——2026-11-02 头顶前方那个橙色格子就在深度图第 1 行）就永远清不掉。

做法：每 period 秒从 /get_planning_scene 取 octomap，按上面的规则选出目标格。
对每个目标格，沿"相机 → 该格"方向在 clip_range(> sensors_3d.yaml 的 max_range) 处放一个点，以 burst_rate
发到 sweep_topic（sensors_3d.yaml 的 head_sweep 传感器）：MoveIt 对超出 max_range 的点只沿射线清空、不记占据
（pointcloud_octomap_updater.cpp：CLIP 点缩到 max_range 处，相机→那里的格全部清空；同帧有点的格从清空里剔除），
同一个格子被清几次（~9 次；burst_rate 10Hz 时 1s 左右）就没了。射线不看真实深度图，裁边区、被挡住的、
不在视野里的格子一样清得到。相机动了（转头、弯腰）就按新的相机位置重新算射线接着清（原来是停下等下一轮）。

保护：清空射线要一直延到 clip_range，路上几乎总会擦到桌子/地面。MoveIt 每帧"有点落在里面的格不清空"，
所以同一帧里给射线经过(外扩一圈)的大块占据格各放一个点（它们本来就是占据，再命中一次没有坏处）。
保护点自己也有一条相机→它的射线，那上面的大块格同样要保护 → 迭代到不再增加（这条只取线上经过的格、到保护点之前为止）。
保护点必须在 max_range 以内才算占据：射线附近有 max_range 外的大块格，这个目标就跳过。

清除区域（2026-11-04）：订 region_topic（moveit_msgs/CollisionObject，BOX；ADD = 开 / 更新，REMOVE = 关，按 id）。
  区域里的占据格不管大小都当目标清（也不给它们放保护点）。区域的 frame_id 可以是机器人 link（如 arm_l_end_link）：
  每轮按 TF 换到 octomap 系，跟着手走——task_board_pose 的 attach_board 挂纸板时开一个"纸板外扩一圈"的区域，
  把纸板附近（含相机看不见、普通机制清不掉的）点云格清掉；取下纸板时关。
"""
import numpy as np
import rclpy
from moveit_msgs.msg import CollisionObject, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, TransformException, TransformListener

from .octomap_util import occupied_centers
from .voxel_util import components, dilate, keys_of, quat_to_mat, unpack


def pick_targets(centers, res, min_voxels, floating_max_voxels, support_z, reach):
    """centers：octomap 占据格中心 (N,3)，octomap 系 = base_link（z 向上）。纯计算，可离线测试。
    返回 (uk 排好序的格键, small 每格要不要清, 小块格数, 悬空块格数)。"""
    uk = np.unique(keys_of(centers, res))
    lab, size = components(uk, reach)
    z = (unpack(uk)[:, 2] + 0.5) * res
    lowest = np.full(len(size), np.inf)
    np.minimum.at(lowest, lab, z)                     # 每块最低格的中心高度
    tiny = size[lab] < min_voxels
    floating = (size[lab] < floating_max_voxels) & (lowest[lab] > support_z) & ~tiny
    return uk, tiny | floating, int(tiny.sum()), int(floating.sum())


class OctomapSweeper(Node):
    def __init__(self):
        super().__init__("octomap_sweeper")
        p = self.declare_parameter
        self.sensor_frame = p("sensor_frame", "head_depth_optical_frame").value
        self.topic = p("sweep_topic", "/head_camera/sweep_points").value
        self.min_voxels = int(p("min_voxels", 4).value)
        self.floating_max_voxels = int(p("floating_max_voxels", 40).value)
        self.support_z = float(p("support_z", 0.15).value)
        self.reach = int(p("connect_reach", 2).value)
        self.max_range = float(p("max_range", 2.5).value)
        self.clip_range = float(p("clip_range", 2.6).value)
        period = float(p("period", 1.0).value)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self.pub = self.create_publisher(PointCloud2, self.topic, 1)
        self.cli = self.create_client(GetPlanningScene, "/get_planning_scene")
        self._future = None
        self._scene = None                    # 最近一次 octomap 的 (格键, 要清的掩码, 分辨率, frame)
        self._plan = None                     # (相机原点, 清空端点, 保护点, frame)，都在 octomap 系
        self._last_log = None
        self.regions = {}                     # id → CollisionObject（BOX）
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(CollisionObject, p("region_topic", "/octomap_sweeper/clear_region").value,
                                 self._on_region, qos)
        self.create_timer(period, self._refresh)
        self.create_timer(1.0 / float(p("burst_rate", 10.0).value), self._sweep)
        self.get_logger().info(f"每 {period}s 查 octomap：小于 {self.min_voxels} 格的块、小于 {self.floating_max_voxels} 格"
                               f"且最低处高于 {self.support_z}m 的悬空块，沿相机射线清掉（→ {self.topic}）")

    def _refresh(self):
        if self._future is not None and not self._future.done():
            return
        if not self.cli.service_is_ready():
            return
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.OCTOMAP
        self._future = self.cli.call_async(req)
        self._future.add_done_callback(self._on_scene)

    def _on_region(self, m):
        if m.operation == CollisionObject.REMOVE:
            if self.regions.pop(m.id, None) is not None:
                self.get_logger().info(f"清除区域 '{m.id}' 关闭")
            return
        if not m.primitives or m.primitives[0].type != m.primitives[0].BOX or len(m.primitive_poses) < 1:
            self.get_logger().warn(f"清除区域 '{m.id}' 只支持一个 BOX，忽略")
            return
        if m.id not in self.regions:
            d = m.primitives[0].dimensions
            self.get_logger().info(f"清除区域 '{m.id}' 打开：{m.header.frame_id} 里 {d[0]:.2f}×{d[1]:.2f}×{d[2]:.2f}m")
        self.regions[m.id] = m

    def _in_regions(self, C, frame):
        """octomap 系的格中心 C 落在任一清除区域里的掩码。TF 查不到的区域这轮跳过。"""
        mask = np.zeros(len(C), bool)
        for rid, m in list(self.regions.items()):
            try:
                tr = self.tf_buffer.lookup_transform(m.header.frame_id, frame, Time())
            except TransformException:
                continue
            t, q = tr.transform.translation, tr.transform.rotation
            P = C @ quat_to_mat(q.x, q.y, q.z, q.w).T + [t.x, t.y, t.z]          # octomap 系 → 区域 frame
            ps = m.primitive_poses[0]
            Rb = quat_to_mat(ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w)
            L = (P - [ps.position.x, ps.position.y, ps.position.z]) @ Rb          # → 盒子自身系
            mask |= (np.abs(L) <= np.asarray(m.primitives[0].dimensions) / 2).all(1)
        return mask

    def _camera(self, frame):
        try:
            tr = self.tf_buffer.lookup_transform(frame, self.sensor_frame, Time())
        except TransformException:
            return None
        t, q = tr.transform.translation, tr.transform.rotation
        return tr.header.stamp, np.array([t.x, t.y, t.z]), quat_to_mat(q.x, q.y, q.z, q.w)

    def _on_scene(self, fut):
        self._scene = None
        self._plan = None
        try:
            om = fut.result().scene.world.octomap
        except Exception as e:                       # move_group 重启等
            self.get_logger().warn(f"取 octomap 失败：{e}")
            return
        C = occupied_centers(om.octomap) if om.header.frame_id else np.empty((0, 3))
        if len(C) == 0:
            return
        res, frame = om.octomap.resolution, om.header.frame_id
        uk, small, n_tiny, n_floating = pick_targets(C, res, self.min_voxels, self.floating_max_voxels,
                                                     self.support_z, self.reach)
        region = self._in_regions((unpack(uk) + 0.5) * res, frame) if self.regions else np.zeros(len(uk), bool)
        self._scene = (uk, small | region, res, frame)   # 区域里的格不管大小都清，也不当大块去保护
        cam = self._camera(frame)
        if cam is None:
            return
        n_ends, skipped, n_protect = self._build(cam[1])
        msg = (f"octomap {len(uk)} 格，小块 {n_tiny} 格、悬空块 {n_floating} 格"
               + (f" + 清除区域里 {int(region.sum())} 格" if region.any() else "")
               + f"：清 {n_ends}，跳过 {skipped}，保护点 {n_protect}")
        if msg != self._last_log:
            self.get_logger().info(msg)
            self._last_log = msg

    def _build(self, o):
        """按相机原点 o 算这一轮的清空端点和保护点，存进 self._plan。返回 (端点数, 跳过数, 保护点数)。"""
        self._plan = None
        uk, small, res, frame = self._scene
        big = uk[~small]
        big_c = (unpack(big) + 0.5) * res
        big_far = big[np.linalg.norm(big_c - o, axis=1) > self.max_range - res]   # 保护不了的
        targets = (unpack(uk[small]) + 0.5) * res
        steps = np.arange(0.0, self.clip_range, res / 8)

        def ray_cells(end, stop=0.0):
            """相机→end 这条线经过的格（到 end 前 stop 米为止，排好序的键）。"""
            L = np.linalg.norm(end - o)
            return np.unique(keys_of(o + np.outer(steps[steps <= L - stop], (end - o) / L), res))

        def near_ray(end):                           # 清空射线：外扩一圈，保守。MoveIt 把远点缩到 max_range，只清到那里
            return dilate(ray_cells(end, stop=self.clip_range - self.max_range))

        ends, protect, skipped = [], np.empty(0, np.int64), 0
        for c in targets:
            d = c - o
            L = np.linalg.norm(d)
            if L < 1e-3 or L > self.max_range:
                skipped += 1
                continue
            end = o + d / L * self.clip_range
            near = near_ray(end)
            if np.isin(big_far, near).any():
                skipped += 1
                continue
            ends.append(end)
            protect = np.union1d(protect, big[np.isin(big, near)])
        frontier = protect
        for _ in range(10):                          # 保护点自己的射线上的大块格也要保护
            if len(frontier) == 0:               # （到保护点所在格之前为止、不外扩：外扩会沿表面一格格蔓延）
                break
            near = np.unique(np.concatenate([ray_cells(e, stop=res) for e in (unpack(frontier) + 0.5) * res]))
            new = np.setdiff1d(big[np.isin(big, near)], protect)
            protect, frontier = np.union1d(protect, new), new
        if len(frontier):                            # 没收敛：宁可这轮不清
            self.get_logger().warn("保护点迭代没收敛，这轮不清")
            return 0, skipped, 0
        if ends:
            self._plan = (o, np.array(ends), (unpack(protect) + 0.5) * res, frame)
        return len(ends), skipped, len(protect)

    def _sweep(self):
        if self._scene is None:
            return
        cam = self._camera(self._scene[3])
        if cam is None:
            return
        stamp, o, R = cam
        if self._plan is not None and np.linalg.norm(o - self._plan[0]) > 0.01:
            self._build(o)                           # 相机动了（转头、弯腰）：按新位置重新算射线，接着清
        if self._plan is None:
            return
        _, ends, prot, _ = self._plan
        P = (np.concatenate([ends, prot]) - o) @ R                 # octomap 系 → 传感器系：Rᵀ(p − o)
        cloud = PointCloud2()
        cloud.header.stamp = stamp
        cloud.header.frame_id = self.sensor_frame
        cloud.height, cloud.width = 1, len(P)
        cloud.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                        for i, n in enumerate("xyz")]
        cloud.is_bigendian, cloud.point_step = False, 12
        cloud.row_step, cloud.is_dense = 12 * len(P), True
        cloud.data = P.astype(np.float32).tobytes()
        self.pub.publish(cloud)


def main():
    rclpy.init()
    node = OctomapSweeper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
