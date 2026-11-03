#!/usr/bin/env python3
"""shadow_cloud.py —— 背面补全：相机只看到纸箱朝向它的那一面，后面是未知的，把它补成有厚度的实体。

输入是 MoveIt 已经按 URDF 碰撞模型(+padding)滤掉机器人自身的点云 /head_camera/filtered_points
（sensors_3d.yaml 里 head_depth 传感器的 filtered_cloud_topic），所以手臂上的点永远不会被往后延长。
  —— 以前在 head_depth_cloud 里用"离手臂骨架 15cm"猜哪些是手臂：手臂运动时看到的肩/肘外壳超出 15cm 就被补到
     手臂背后，那片格子被手臂自己挡着、射线清不掉，手臂一往那边走 MoveIt 就判"剩余路径撞了"中止执行，
     之后手臂泡在格子里再也规划不出来（2026-11-02 截图）。

手臂够得到的范围内（点离 reach_link ≤ shadow_reach），每个点沿"相机→点"方向往后每 shadow_step 补一个点、
补到 shadow_length。输出 = 这些表面点 + 补出来的点，同帧同 stamp，发给 sensors_3d.yaml 里第二个传感器 head_shadow：
  · 同一帧里占据格不会被射线清空 → 表面格和补出来的格互不清除，只清表面前方；
  · 补出来的点再经 MoveIt 自身过滤一次（落进机器人模型的不记），多一道保险；
  · 物体拿走后，head_depth 的射线能穿过去，正常清掉。
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, ExtrapolationException, TransformException, TransformListener


class ShadowCloud(Node):
    def __init__(self):
        super().__init__("shadow_cloud")
        p = self.declare_parameter
        self.in_topic = p("input_topic", "/head_camera/filtered_points").value
        self.out_topic = p("output_topic", "/head_camera/shadow_points").value
        self.length = float(p("shadow_length", 0.25).value)
        self.step = float(p("shadow_step", 0.03).value)
        self.reach = float(p("shadow_reach", 1.0).value)
        self.reach_link = p("reach_link", "arm_base_link").value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self.pub = self.create_publisher(PointCloud2, self.out_topic, 1)
        self.create_subscription(PointCloud2, self.in_topic, self._on_cloud, qos_profile_sensor_data)
        self._warned = False
        self.steps = np.arange(self.step, self.length + 1e-6, self.step, dtype=np.float32)
        self.get_logger().info(f"{self.in_topic} → {self.out_topic}：离 {self.reach_link} ≤{self.reach}m 的点往后补 "
                               f"{self.length}m（每 {self.step}m 一个）")

    @staticmethod
    def _xyz(msg):
        off = {f.name: f.offset for f in msg.fields}
        if not all(k in off for k in "xyz"):
            return np.empty((0, 3), np.float32)
        n = msg.width * msg.height
        raw = np.frombuffer(bytes(msg.data), np.uint8).reshape(n, msg.point_step)
        cols = [raw[:, off[k]:off[k] + 4].copy().view(np.float32).reshape(-1) for k in "xyz"]
        P = np.stack(cols, axis=1)
        return P[np.all(np.isfinite(P), axis=1)]

    def _on_cloud(self, msg):
        P = self._xyz(msg)
        try:
            try:
                tr = self.tf_buffer.lookup_transform(msg.header.frame_id, self.reach_link,
                                                     rclpy.time.Time.from_msg(msg.header.stamp))
            except ExtrapolationException:
                tr = self.tf_buffer.lookup_transform(msg.header.frame_id, self.reach_link, rclpy.time.Time())
        except TransformException as e:
            if not self._warned:
                self.get_logger().warn(f"查不到 {msg.header.frame_id}←{self.reach_link}，这帧不补：{e}")
                self._warned = True
            return
        t = tr.transform.translation
        center = np.array([t.x, t.y, t.z], dtype=np.float32)
        Q = P[np.linalg.norm(P - center, axis=1) <= self.reach] if len(P) else P
        if len(Q):
            unit = Q / np.linalg.norm(Q, axis=1, keepdims=True)
            S = (Q[None, :, :] + self.steps[:, None, None] * unit[None, :, :]).reshape(-1, 3)
            pts = np.concatenate([Q, S]).astype(np.float32)
        else:
            pts = np.empty((0, 3), np.float32)
        out = PointCloud2()
        out.header = msg.header
        out.height, out.width = 1, len(pts)
        out.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate("xyz")]
        out.is_bigendian, out.point_step = False, 12
        out.row_step, out.is_dense = 12 * len(pts), True
        out.data = pts.tobytes()
        self.pub.publish(out)


def main():
    rclpy.init()
    node = ShadowCloud()
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
