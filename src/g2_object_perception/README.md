# g2_object_perception —— 头部 RGBD 找物体:HSV 颜色 / SAM 3 文字分割,掩码 + 深度 → base_link 坐标,转头对准(AgiBot G2)

> 原样复制自 `~/robot_harness/harness/perception/`、`harness/backends/color_object.py`、`scripts/`,以及
> `~/sam3_service/server.py`(2026-11-12,未提交)。**没改过 import**:`from .calib`、`from ..motion.kin`
> (`kin.py` 在 `../g2_arm_ik/`)、`from harness.perception.sam3_client` 按原仓库结构写。
> `rgbd_source.py`、`scene3d.py`、`calib.py`、`sensor_node.py` 最早来自 `~/agi@g2_robot/src/task_board_pose`(提交 `ecdfb5d`)。

## 结论:用哪个

| 场景 | 用 | 说明 |
|---|---|---|
| **按名字找任意物体(首选)** | **SAM 3**:`sam3_client.Sam3Client` → `sam3_objects.objects_from_masks` | 开放词汇,一张图所有实例的掩码;要 GPU 和单独服务 |
| 颜色鲜明、不想开 GPU | HSV:`color_object.find_objects` + `merge_frames`(5 帧中位数) | 依赖光照;新颜色用 `scripts/hsv_pick.py` 点选 |
| 让物体在画面中央 | `head_aim.HeadAim.solve` | URDF 正运动学 + 相机外参,只改头 yaw/pitch |
| 实时看分割效果 | `scripts/sam3_live.py` | 快输入慢处理:最新帧格子 + 零拷贝 |
| 离线素材 | `scripts/capture_head.py` | 彩色 + 深度 + 头部位姿,格式同 `DatasetSource` |

SAM 3 实测(RTX 4070 Laptop 8GB,bf16,640×400):一张图 1 个短语 ≈ **320 ms**,每多一个短语 +45 ms;显存常驻 3.4 GB、峰值 5.1 GB。
积木(70~270 像素)、笔、黑包、手机都能分;`phone` 和 `mobile phone` 结果相同。
问题:相近颜色串(红积木也匹配 `orange block` 0.66)、泛称(`box`)不稳、偶有 0.5 左右的误检。

## SAM 3 为什么是单独的服务

SAM 3 要 Python ≥ 3.12,ROS 2 Humble 是 3.10,不能在一个进程里 import。分界线按进程画:

- `sam3_service/server.py`(3.12,uv 建的 `.venv`):只管“图 + 英文短语 → 掩码”,不知道 ROS。
- 其余全部(客户端、3D 换算、实时画面)跑在 ROS 的 3.10 里。

接口 `POST /segment`:JSON(`image_jpg_b64`)或 `Content-Type: image/jpeg` 原始字节 + 头 `X-Sam3-Request: {"prompts": [...]}`(实时用,不 base64)。
安装和权重见原仓库 `~/robot_harness/docs/sam3_service.md`。

## 掩码 → 3D(`sam3_objects.objects_from_masks`)

掩码 ∩ 有效深度 → 取深度中位数 ±3cm 那层表面(去边缘飞点)→ `Rgbd.points_base` 的 base_link 点取中位数。
按离相机距离排序编号。每次分割存 `traces/sam3/<时间>_<短语>/`:`color.jpg`、`overlay.jpg`、`mask_<编号>.png`、`depth.png`、`result.json`。
离线核对:3D 点反投影回画面,和掩码中心差 < 1 像素;桌面物体 z ≈ 0.74~0.78 m。

## 文件

```
g2_object_perception/
├── rgbd_source.py     RosSource:/gdk/camera/head_color(JPEG)+ head_depth 按 stamp 配对,/gdk/tf 取 base_link→head_link3;DatasetSource 回放
├── scene3d.py         Rgbd.points_base:深度 → base_link 3D 点(深度和彩色已逐像素对齐,不用再配准)
├── calib.py           头部相机内参、外参(2026-11-03 GDK 读出;能 import agibot_gdk 时重读)
├── sensor_node.py     用时才订阅、用完退订的传感器节点(图像流量大)
├── color_object.py    HSV 分割 + 连通域 + 尺寸过滤;merge_frames 多帧选离参考点最近 + 稳定性检查
├── colors.yaml        预置颜色的 HSV 范围
├── sam3_client.py     ★ Sam3Client.segment(bgr) / segment_jpeg(内存原样发);draw 画掩码
├── sam3_objects.py    ★ objects_from_masks、save_segmentation(日志)、slug
├── head_aim.py        ★ HeadAim:base_link→head_link3 链(5 腰 + 3 头)+ 外参,2×2 高斯-牛顿解 yaw/pitch,夹限位
├── sam3_service/server.py  SAM 3 HTTP 服务(启动时先查本机权重缓存,完全不联网)
└── scripts/
    ├── sam3_live.py       实时画面 + 终端随时换物体名(4 线程:ROS 回调 / 分割 / 输入 / 显示)
    ├── sam3_test_image.py 单张图测服务
    ├── capture_head.py    录帧
    └── hsv_pick.py        在画面上点选新颜色
```

## G2 专有事实 / 坑

- 头部相机 640×400;深度 `16UC1` 毫米,0 = 无效;彩色是 `CompressedImage` JPEG。
- 零位时相机在 base_link (0.20, 0.03, 1.68),光轴朝下约 25°。头 yaw 限 ±44.7°,pitch(正 = 低头)−1°~30°:桌上很近的东西低头不够。
- `scene3d` 的外参约定:`color_in_head_link3` 直接当深度光学系用;曾多乘 `depth_in_color` 导致横向偏 3cm(文件头有记录)。
- SAM 3 服务坑:克隆目录不能叫 `sam3/`(会挡住包);`hf` 不认 `all_proxy=socks://`;Xet 下载走代理会卡死,用 `HF_HUB_DISABLE_XET=1`;
  `HF_HUB_OFFLINE` 对 huggingface_hub 2.4 的 `hf_hub_download` 无效,所以 server 先 `try_to_load_from_cache` 再把路径传给模型。
- 实时程序防旧图堆积:ROS QoS `KEEP_LAST depth=1` + 程序里只留最新一帧(`LatestSlot`,`threading.Condition`);格子里只存消息引用,有人要才解码。
