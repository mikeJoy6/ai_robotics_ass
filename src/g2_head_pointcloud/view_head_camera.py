#!/usr/bin/env python3
"""订阅头部相机 CompressedImage(jpeg) 话题，解码后用 OpenCV 窗口显示。

用法:
    python3 view_head_camera.py                         # 默认 /gdk/camera/head_color
    python3 view_head_camera.py /gdk/camera/hand_left_color
    python3 view_head_camera.py --save out.jpg          # 只保存一帧后退出(无显示器时用)
按 q 或 ESC 退出，按 s 保存当前帧。
"""
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


class HeadCameraViewer(Node):
    def __init__(self, topic, save_path=None):
        super().__init__('head_camera_viewer')
        self.topic = topic
        self.save_path = save_path
        self.frame = None
        self.create_subscription(CompressedImage, topic, self.callback, qos_profile_sensor_data)
        self.get_logger().info(f'subscribed to {topic}')

    def callback(self, msg):
        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            self.get_logger().warn(f'decode failed, format={msg.format}')
            return
        self.frame = img


def main():
    args = sys.argv[1:]
    save_path = None
    if '--save' in args:
        i = args.index('--save')
        save_path = args[i + 1]
        del args[i:i + 2]
    topic = args[0] if args else '/gdk/camera/head_color'

    rclpy.init()
    node = HeadCameraViewer(topic, save_path)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            if node.frame is None:
                continue
            if save_path:
                cv2.imwrite(save_path, node.frame)
                print(f'saved {node.frame.shape} -> {save_path}')
                break
            cv2.imshow(topic, node.frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                break
            if key == ord('s'):
                name = f'frame_{time.strftime("%Y%m%d_%H%M%S")}.jpg'
                cv2.imwrite(name, node.frame)
                print(f'saved {name}')
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
