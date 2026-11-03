# bench —— planner_bench 的真机快照

| 文件 | 内容 |
|---|---|
| `octomap_0428_base_link.npy` | 2026-11-02 04:28 真机 MoveIt octomap 占据体素中心 (N,3)，base_link 系，3cm 分辨率 |
| `snapshot_joints.json` | 同一时刻全身关节角：腰/头/左臂固定在这个姿态，右臂做规划 |

`planner_bench` 的场景 `real` 用它；另有合成场景 `box`（桌上箱子挡在 A/B 中间）、`wall`（悬空竖板）、`free`（无障碍）。
