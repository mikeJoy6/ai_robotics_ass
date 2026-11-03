# g2_head_pointcloud —— 头部深度图 → MoveIt octomap 避障点云，以及怎么把垃圾点删干净（AgiBot G2）

> 原样复制自 `~/agi@g2_robot/src/genie_robot_agibot_g2_perception/`（+ `genie_moveit_config/config/sensors_3d.yaml`、仓库根 `view_head_camera.py`），2026-11-03。
> 包名在源码里是 `genie_robot_agibot_g2_perception`，没改。依赖 `gdk_msgs`（G2 关节反馈）、`ros-humble-moveit-ros-perception`。

## 总体思路

不自己写 SLAM：MoveIt 自带的 octomap（`PointCloudOctomapUpdater`）正好是要的语义——看到点就占据、相机到点的射线清空、被挡住的不动、
按 URDF 碰撞模型 + padding 滤掉机器人自身、地图固定在 base_link（弯腰转头不跟着动）。本目录只做两件事：
**把 G2 裸深度图变成 MoveIt 能用的点云**，以及**在点进 octomap 之前把垃圾删掉**。

octomap 的致命点：**一个格子一旦被点亮，只有射线穿过它才会被清掉**。落在手臂背后、物体背后、视野边缘、没有射线的方向上的假点，
会永远留着，手臂一过去就判撞、执行中止、之后再也规划不出来。所以这里花了大量功夫在"源头不让垃圾进去"。

## 数据流

```
/gdk/camera/head_depth (16UC1, frame_id 空, 无 camera_info)
  └─ head_depth_cloud ── 静态 TF(head_link3→彩色→深度光学系)、反投影、全部过滤 ──→ /head_camera/depth_points
        └─ MoveIt head_depth 传感器：自身过滤(padding 0.10) → octomap；过滤后的点 → /head_camera/filtered_points
              └─ shadow_cloud：背面补全(沿射线往后 25cm) ──→ /head_camera/shadow_points → MoveIt head_shadow 传感器
  octomap_sweeper：每 1s 取 octomap，<4 格的零星块沿相机射线清掉 ──→ /head_camera/sweep_points → MoveIt head_sweep 传感器
```

## 垃圾点处理清单（`head_depth_cloud.py` 文件头 1~15 条有完整原因和实测数据）

按执行顺序：

| 步骤 | 治什么 | 关键做法 |
|---|---|---|
| 深度图抠手臂（第 14 条） | 手臂贴在镜头前（近距盲区）测出的假深度、运动时间差 | 手臂碰撞凸包投影成轮廓（拍摄时刻−0.1s/拍摄时刻/最新 三个位姿合起来），轮廓里深度在 [link 最近点−5cm, 最远点+30cm] 的丢；更远的是看得见的背景，留 |
| 散斑（第 5 条） | 单像素随机噪点 | 全分辨率 3×3 邻域相近像素太少就丢 |
| 深度跳变（第 15 条） | 物体/手臂轮廓上的飞点 | 隔 2 像素的邻居连线与视线夹角 <10° 且深度差 >5%·深度 → 两点都删再外扩 2px；只删 2.5m 内 |
| 裁边（第 9 条） | 画面四边深度很差，假点落在射线稀少的视野边缘 | 四周裁 12px |
| 悬空小块（第 7 条） | 深度相机固定坏点（红外窗口灰，每帧稳定报 0.381m） | 比邻域中位深度近 40% 且块 <4 点 → 丢；必须在发布网格上做 |
| 悬空块整块删（第 10 条） | 空中漂浮的噪点、单独一截手臂 | 网格连通块：贴地 / 碰画面边 / 紧挨更近的遮挡物 三条都不满足才删 |
| 跳变点最后才去掉 | 先去会切断物体与地面的连接 → 整块箱面被当悬空删 | 跳变在悬空块判断之后应用 |
| 远点只清空（第 6 条） | 看向远处的方向没有射线，被点亮的格子永远清不掉 | >4m 的像素沿射线放到 3m（> MoveIt max_range 2.5m）：MoveIt 只清空不记占据；深度 0 仍丢 |
| 多帧确认（第 11 条） | 一闪一闪的点在箱顶"增生" | base_link 3cm 分格，最近 4 帧至少 3 帧有点才发 |
| 小块删除（第 12 条） | 只看大物品 | 3D 连通（容许空一格），<4 格整块不发 |
| 只在 2.5m 内做 10~12（第 13 条） | 远处稀疏点被当零星点删 → 清空射线没了、地图越积越脏 | 远点不过这几道 |
| 时间戳（第 3 条） | 手臂运动时自身过滤对不上 | 图像机器人 stamp + 关节"收到−stamp"中位延迟，落到 TF 时间轴 |
| MoveIt padding 0.10（sensors_3d.yaml） | 手臂静止周围一圈格子 | ConvexMesh 外扩沿"中心→顶点"推，细长 link 侧面只外扩一半；0.05 不够 |
| 背面补全挪到 MoveIt 之后（shadow_cloud） | 手臂外壳被往后延长、手臂自己挡着清不掉 | 只补 MoveIt 已滤掉机器人的 filtered_points |
| octomap_sweeper | 已经进图的零星格 | 发 2.6m 处的清空射线点 + 路上大块格的"保护点"（同帧有点的格不清空） |

## 文件

```
g2_head_pointcloud/
├── head_depth_cloud.py      ★ 主节点：TF、反投影、上表全部过滤；文件头 15 条是每一道的原因和实测
├── robot_mask.py            URDF 碰撞凸包 → 深度图上每像素的"手臂深度区间"（cv2.fillConvexPoly + min/max 滤波）
├── voxel_util.py            体素键打包/解包、26 邻域连通(可跨空格)、膨胀、四元数→矩阵
├── octomap_util.py          解码 MoveIt 发的完整 OcTree（每节点 float32 log-odds + uint8 子节点位，不是 2bit 二进制格式）→ 占据体素中心
├── shadow_cloud.py          背面补全：表面点沿相机射线往后每 3cm 补到 25cm（手臂够得到的范围内）
├── octomap_sweeper.py       零星格清扫
├── dump_head_camera_calib.py 从 GDK Python API 重读头部相机内外参
├── view_head_camera.py      看头部相机画面（诊断）
├── config/                  head_camera.yaml（标定+全部过滤参数）、sensors_3d.yaml（MoveIt 3 个传感器），见 config/README.md
└── launch/                  head_depth_cloud.launch.py，见 launch/README.md
```

## 踩过的坑

- **`filtered_cloud_topic` 在 MoveIt Humble 是必填**：缺了整个传感器加载失败，只打一行 `Failed to configure updater of type PointCloudUpdater`，补全点从没进过 octomap。
- GDK 相机话题、`/joint_states`、`/rosout` 都是 BEST_EFFORT，RELIABLE 订阅收不到。
- 单线程执行器里带 timeout 的 TF lookup 会阻塞 TF 回调 → 100% 失败：`TransformListener(spin_thread=True)` + 关节订阅独立回调组 + MultiThreadedExecutor。
- 外参约定用**地面拟合**判定（四种候选里只有"RGBD=彩色，深度→彩色→head_link3"地面倾角 0.7°）；地面在 base_link z≈−6cm。
- 地图在 base_link：底盘一动地图整体跟车走，只适合原地操作。
- 旧格子不会自己消失：`ros2 service call /clear_octomap std_srvs/srv/Empty` 或重启。改了 sensors_3d.yaml 必须重启 move_group。
- 效果验证用"旁路对比"：同一帧深度图起第二个实例（不同节点名/输出话题、`"/**"` 键的参数文件），比较稳定格丢失数，不要只看截图。
- 还没做：robot_mask 只认 URDF link，手上附着的纸板不会被抠掉。
