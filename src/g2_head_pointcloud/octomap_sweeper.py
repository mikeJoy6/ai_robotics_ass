#!/usr/bin/env python3
"""octomap_sweeper.py —— 把 MoveIt octomap 里的零星格清掉：只看大物品，小于 min_voxels 格的连通块不管真假都删。

head_depth_cloud 已经在源头删小块（文件头第 12 条），但早先进来的、或背面补全/多帧叠加凑出来的零星格还会留在图里；
落在射线到不了的地方（被挡住、或在画面裁边区里——2026-11-02 头顶前方那个橙色格子就在深度图第 1 行）就永远清不掉。

做法：每 period 秒从 /get_planning_scene 取 octomap，26 邻域连通，块小于 min_voxels 格的是目标。
对每个目标格，沿"相机 → 该格"方向在 clip_range(> sensors_3d.yaml 的 max_range) 处放一个点，以 burst_rate
发到 sweep_topic（sensors_3d.yaml 的 head_sweep 传感器）：MoveIt 对超出 max_range 的点只沿射线清空、不记占据
（pointcloud_octomap_updater.cpp：CLIP 点缩到 max_range 处，相机→那里的格全部清空；同帧有点的格从清空里剔除），
同一个格子被清几次（~9 次，2s 左右）就没了。射线不看真实深度图，裁边区、被挡住的格子一样清得到。

保护：清空射线要一直延到 clip_range，路上几乎总会擦到桌子/地面。MoveIt 每帧"有点落在里面的格不清空"，
所以同一帧里给射线经过(外扩一圈)的大块占据格各放一个点（它们本来就是占据，再命中一次没有坏处）。
保护点自己也有一条相机→它的射线，那上面的大块格同样要保护 → 迭代到不再增加（这条只取线上经过的格、到保护点之前为止）。
保护点必须在 max_range 以内才算占据：射线附近有 max_range 外的大块格，这个目标就跳过。
"""
import numpy as np
import rclpy
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, TransformException, TransformListener

from .octomap_util import occupied_centers
from .voxel_util import components, dilate, keys_of, quat_to_mat, unpack


class OctomapSweeper(Node):
    def __init__(self):
        super().__init__("octomap_sweeper")
        p = self.declare_parameter
        self.sensor_frame = p("sensor_frame", "head_depth_optical_frame").value
        self.topic = p("sweep_topic", "/head_camera/sweep_points").value
        self.min_voxels = int(p("min_voxels", 4).value)
        self.max_range = float(p("max_range", 2.5).value)
        self.clip_range = float(p("clip_range", 2.6).value)
        period = float(p("period", 1.0).value)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self.pub = self.create_publisher(PointCloud2, self.topic, 1)
        self.cli = self.create_client(GetPlanningScene, "/get_planning_scene")
        self._future = None
        self._plan = None                     # (相机原点, 清空端点, 保护点)，都在 octomap 系
        self._last_log = None
        self.create_timer(period, self._refresh)
        self.create_timer(1.0 / float(p("burst_rate", 5.0).value), self._sweep)
        self.get_logger().info(f"每 {period}s 查 octomap，小于 {self.min_voxels} 格的块沿相机射线清掉（→ {self.topic}）")

    def _refresh(self):
        if self._future is not None and not self._future.done():
            return
        if not self.cli.service_is_ready():
            return
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.OCTOMAP
        self._future = self.cli.call_async(req)
        self._future.add_done_callback(self._on_scene)

    def _camera(self, frame):
        try:
            tr = self.tf_buffer.lookup_transform(frame, self.sensor_frame, Time())
        except TransformException:
            return None
        t, q = tr.transform.translation, tr.transform.rotation
        return tr.header.stamp, np.array([t.x, t.y, t.z]), quat_to_mat(q.x, q.y, q.z, q.w)

    def _on_scene(self, fut):
        self._plan = None
        try:
            om = fut.result().scene.world.octomap
        except Exception as e:                       # move_group 重启等
            self.get_logger().warn(f"取 octomap 失败：{e}")
            return
        C = occupied_centers(om.octomap) if om.header.frame_id else np.empty((0, 3))
        cam = self._camera(om.header.frame_id) if len(C) else None
        if cam is None:
            return
        res, o = om.octomap.resolution, cam[1]
        uk = np.unique(keys_of(C, res))
        lab, size = components(uk)
        small = size[lab] < self.min_voxels
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
            return
        if ends:
            self._plan = (o, np.array(ends), (unpack(protect) + 0.5) * res, om.header.frame_id)
        msg = f"octomap {len(uk)} 格，零星格 {int(small.sum())} 个：清 {len(ends)}，跳过 {skipped}，保护点 {len(protect)}"
        if msg != self._last_log:
            self.get_logger().info(msg)
            self._last_log = msg

    def _sweep(self):
        if self._plan is None:
            return
        o0, ends, prot, frame = self._plan
        cam = self._camera(frame)
        if cam is None:
            return
        stamp, o, R = cam
        if np.linalg.norm(o - o0) > 0.01:            # 相机动了：射线和规划时不一样，等下一轮
            return
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
