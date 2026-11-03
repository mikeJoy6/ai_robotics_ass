#!/usr/bin/env python3
"""attach_board.py —— 运行中给 MoveIt 挂/取纸板：作为附着碰撞体(AttachedCollisionObject)挂在末端吸盘(end_link 长方体)前面。

不改 URDF：/apply_planning_scene 发一次 diff 就生效，move_group 不用重启。挂上后纸板跟着末端走，
MoveIt 整机碰撞检查把它当手臂的一部分（对身体、另一条臂、octomap、虚拟碰撞体都查）；octomap 更新时也会把它从点云里滤掉。

几何（end_link 系：z = 吸盘伸出方向；料框里的工作姿态 x ≈ 竖直朝上、y ≈ 水平）：
  吸盘 = URDF end_link 长方体，z∈[0, 0.128]（hardware.launch.py end_effector:=true 才有）。
  纸板贴在吸盘端面上：z∈[0.128, 0.128+厚度]；长边沿 end x、宽边沿 end y；
  吸附点在纸板中心偏下 → 纸板中心相对吸盘轴线往 +x(上) 偏 --offset。
  厚度默认 2cm（真纸板只有几 mm）：碰撞检查是离散采样的（关节每 2° 查一次，末端外 ~1m 处一步最多走 ~3.8cm），
  octomap 体素 3cm：2cm + 3cm > 3.8cm，采样之间不会整块穿过一个体素；再薄就可能两次采样之间"跳过"障碍。

  ros2 run genie_robot_agibot_g2_hardware attach_board.py                       # 右臂挂默认纸板
  ros2 run genie_robot_agibot_g2_hardware attach_board.py --offset 0.15         # 改吸附点（同 id 重发 = 替换）
  ros2 run genie_robot_agibot_g2_hardware attach_board.py --remove              # 取下并从场景删除
"""
import argparse
import sys

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from shape_msgs.msg import SolidPrimitive

CUP_LENGTH = 0.128           # 吸盘(end_link 长方体)沿 end z 的长度，同 G2_t2_crsB_ee.urdf


def call(node, cli, req, timeout=10.0):
    if not cli.wait_for_service(timeout_sec=timeout):
        return None
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    return fut.result() if fut.done() else None


def main():
    ap = argparse.ArgumentParser(description="运行中给 MoveIt 挂/取纸板附着碰撞体")
    ap.add_argument("--arm", choices=["left", "right"], default="right")
    ap.add_argument("--length", type=float, default=0.59, help="纸板长边(m)，沿 end x（工作姿态竖直）")
    ap.add_argument("--width", type=float, default=0.405, help="纸板宽边(m)，沿 end y（工作姿态水平）")
    ap.add_argument("--thickness", type=float, default=0.02, help="碰撞模型厚度(m)，沿 end z")
    ap.add_argument("--offset", type=float, default=0.10,
                    help="吸附点在纸板中心下方多远(m)：纸板中心沿 end +x 偏这么多")
    ap.add_argument("--id", default="board")
    ap.add_argument("--remove", action="store_true", help="取下并从场景删除")
    args = ap.parse_args(remove_ros_args(sys.argv)[1:])

    side = "l" if args.arm == "left" else "r"
    link = f"arm_{side}_end_link"
    rclpy.init()
    node = Node("attach_board")
    log = node.get_logger()

    aco = AttachedCollisionObject(link_name=link)
    aco.object.id = args.id
    aco.object.header.frame_id = link
    scene = PlanningScene(is_diff=True)
    scene.robot_state.is_diff = True
    if args.remove:
        aco.object.operation = CollisionObject.REMOVE      # 先从手上取下（MoveIt 会把它放回 world）
        world = CollisionObject(id=args.id, operation=CollisionObject.REMOVE)
        world.header.frame_id = link
        scene.world.collision_objects.append(world)        # 再从 world 删掉
    else:
        aco.touch_links = [link, f"arm_{side}_link7"]      # 贴着吸盘/手腕不算碰撞
        aco.object.operation = CollisionObject.ADD         # 同 id 再发 = 替换
        aco.object.primitives.append(SolidPrimitive(
            type=SolidPrimitive.BOX, dimensions=[args.length, args.width, args.thickness]))
        aco.object.pose.orientation.w = 1.0
        p = Pose()
        p.orientation.w = 1.0
        p.position.x = args.offset
        p.position.z = CUP_LENGTH + args.thickness / 2
        aco.object.primitive_poses.append(p)
    scene.robot_state.attached_collision_objects.append(aco)

    res = call(node, node.create_client(ApplyPlanningScene, "/apply_planning_scene"),
               ApplyPlanningScene.Request(scene=scene))
    if res is None or not res.success:
        log.error("/apply_planning_scene 失败或不可用（move_group 在跑吗？）")
        rclpy.shutdown()
        return 1

    # 回读确认：手上挂着哪些
    req = GetPlanningScene.Request()
    req.components.components = PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
    got = call(node, node.create_client(GetPlanningScene, "/get_planning_scene"), req)
    attached = [(a.object.id, a.link_name) for a in got.scene.robot_state.attached_collision_objects] if got else "?"
    if args.remove:
        log.info(f"已取下 {args.id}；当前附着物 {attached}")
    else:
        log.info(f"已挂 {args.id} → {link}：{args.length}×{args.width}×{args.thickness}m，"
                 f"吸附点在中心下方 {args.offset}m，贴吸盘端面 z={CUP_LENGTH}；当前附着物 {attached}")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
