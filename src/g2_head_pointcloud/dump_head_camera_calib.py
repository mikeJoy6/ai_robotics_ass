#!/usr/bin/env python3
"""dump_head_camera_calib.py —— 从真机 GDK 重新读头部深度相机内参/外参，打印成 config/head_camera.yaml 的写法。

需要 GDK 库路径（不切模式）：
    source ~/.cache/agibot/app/env.sh
    source ~/agi@g2_robot/env_g2.sh
    ros2 run genie_robot_agibot_g2_perception dump_head_camera_calib
只读，不动机器人。相机重新标定/换头后跑一次，把输出贴回 config/head_camera.yaml。
"""
import time


def main():
    try:
        import agibot_gdk as g
    except ImportError:
        raise SystemExit("import agibot_gdk 失败：先 source ~/.cache/agibot/app/env.sh")
    if g.gdk_init() != g.GDKRes.kSuccess:
        raise SystemExit("gdk_init 失败：网线/机器人在线吗？")
    cam, tf = g.Camera(), g.TF()
    time.sleep(2.0)
    k = cam.get_camera_intrinsic(g.CameraType.kHeadDepth)
    e = tf.get_tf_from_sensor(g.SensorExtrinsicType.kHeadRGBDToHeadLink3)
    d = tf.get_tf_from_sensor(g.SensorExtrinsicType.kHeadDepthToHeadColor)

    def row(t):
        return [t.translation.x, t.translation.y, t.translation.z,
                t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]

    fmt = lambda v: "[" + ", ".join(f"{x:.6g}" for x in v) + "]"
    print(f"    intrinsics: {fmt(k.intrinsic[:4])}")
    print(f"    # distortion: {fmt(k.distortion)}")
    print(f"    color_in_head: {fmt(row(e))}")
    print(f"    depth_in_color: {fmt(row(d))}")


if __name__ == "__main__":
    main()
