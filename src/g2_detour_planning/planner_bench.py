"""planner_bench —— 离线对比 planners/ 里所有绕障规划器（不需要机器人、ROS 节点、MoveIt）。

碰撞用 sim_world.SimWorld（URDF 凸包 + SRDF 自碰撞豁免 + octomap 体素，和真机 MoveIt 同一模型，单次 ~1ms ≈ 真机服务）。
场景（右臂，腰/头/左臂固定在 2026-11-02 04:28 真机快照的关节值）：
  real   04:28 真机 octomap（去掉快照里粘在右臂上的格子），A/B 随机取，被真实障碍挡住的算绕障题
  box    合成：桌面 + 桌上一个挡在 A/B 正中间的箱子（高度/宽度随机）
  wall   合成：A/B 中间一块悬空竖板（只能从上/下/侧面绕）
  free   没有障碍（看直线情况下有没有多余开销）
每题 A=起点关节角，B=目标末端位姿（只给位姿，IK 由规划器自己解）。题目由固定随机种子生成，可复现。

指标：成功率、规划耗时、碰撞查询次数、执行时长(arm_motion.cart_profile 同一套变速)、末端路程/直线距离、
关节累计转角、末端最远偏离起终点连线（抡圈程度）、手臂离障碍最近距离。
每条结果路径都在 SimWorld 里逐 2° 复查一遍，复查撞了算失败。

    source /opt/ros/humble/setup.bash && source ~/agi@g2_robot/install/setup.bash
    ros2 run genie_robot_agibot_g2_motion planner_bench                # 全部规划器、全部场景
    ros2 run genie_robot_agibot_g2_motion planner_bench --planners via_points astar_w25 --cases 10
    ros2 run genie_robot_agibot_g2_motion planner_bench --edge relax_orient=0 clearance=0.06   # 改"一段"的参数对比
"""
import argparse
import json
import math
import multiprocessing as mp
import os

for _v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):   # 多进程并行：每个进程单线程，
    os.environ.setdefault(_v, "1")                                         # 否则 BLAS 线程互抢，规划耗时失真
import time
from pathlib import Path

import numpy as np

def _paths():
    from ament_index_python.packages import get_package_share_directory
    desc = get_package_share_directory("genie_robot_description")
    cfg = get_package_share_directory("genie_moveit_config")
    urdf = os.path.join(desc, "urdf", "G2_t2_crsB", "G2_t2_crsB.urdf")
    return urdf, os.path.join(cfg, "config", "genie.srdf"), os.path.dirname(desc)


def _data_dir():
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory("genie_robot_agibot_g2_motion")) / "bench"


# ── 世界 / 题目 ──────────────────────────────────────────────────────
_W = {}


def world(scene):
    """每个进程每个场景建一次。"""
    if scene in _W:
        return _W[scene]
    from .algorithms import ArmSolver
    from .sim_world import SimWorld
    urdf, srdf, mesh_root = _paths()
    data = _data_dir()
    meta = json.loads((data / "snapshot_joints.json").read_text())
    W = SimWorld(open(urdf).read(), open(srdf).read(), "right", meta["joints"], mesh_root)
    W.solver = ArmSolver(W.kin)
    W.q_snap = np.array([meta["joints"][n] for n in W.kin.joint_names])
    W.vel = np.array([W.robot.joint_map[j].limit.velocity for j in W.kin.joint_names])
    if scene == "real":
        O = W.base_to_root(np.load(data / "octomap_0428_base_link.npy"))
        # 快照里自身过滤漏掉、粘在右臂上的格子去掉（那正是当时要分析的 bug，不是环境）
        H = W.link_hulls(W.q_snap)
        S = np.concatenate([h.S for hs in H.values() for h in hs])
        from scipy.spatial import cKDTree
        O = O[cKDTree(S).query(O)[0] > 0.06]
        W.base_obs = O
    else:
        W.base_obs = _table(W) if scene == "box" else np.empty((0, 3))
    W.set_obstacles(W.base_obs, 0.03)
    _W[scene] = W
    return W


def _voxels(lo, hi, res=0.03):
    g = [np.arange(a + res / 2, b, res) for a, b in zip(lo, hi)]
    return np.stack(np.meshgrid(*g, indexing="ij"), -1).reshape(-1, 3)


def _shell(lo, hi, res=0.03):
    """长方体表面一层体素（octomap 看到的就是表面 + 背面补全，内部是未知；这里取表面层足够判撞）。"""
    V = _voxels(lo, hi, res)
    lo, hi = np.asarray(lo), np.asarray(hi)
    on = np.any((V - lo < res) | (hi - V < res), axis=1)
    return V[on]


TABLE_Z = 0.72


def _table(W):
    """base_link 里：桌面 z=0.72，x 0.40~1.10，y -0.80~0.40（右臂前方），厚 3cm。转到根系。"""
    return W.base_to_root(_voxels([0.40, -0.80, TABLE_Z - 0.03], [1.10, 0.40, TABLE_Z]))


def _oriented_box(center, u, half):
    """base_link 里：中心 center，水平朝向 u（板厚方向），half=(厚/2, 宽/2, 高/2)。返回表面体素。"""
    u = np.array([u[0], u[1], 0.0]); u /= np.linalg.norm(u)
    v = np.array([-u[1], u[0], 0.0])
    B = _shell(-np.asarray(half), np.asarray(half))
    return center + B[:, 0:1] * u + B[:, 1:2] * v + B[:, 2:3] * np.array([0, 0, 1.0])


def make_cases(n_per_scene, seed=0):
    rng = np.random.default_rng(seed)
    cases = []
    for scene in ("real", "box", "wall", "free"):
        W = world(scene)
        n = n_per_scene if scene != "free" else max(3, n_per_scene // 3)
        tries = 0
        while sum(c["scene"] == scene for c in cases) < n and tries < 4000:
            tries += 1
            c = _sample_case(W, scene, rng)
            if c is not None:
                c["id"] = f"{scene}{sum(x['scene'] == scene for x in cases):02d}"
                cases.append(c)
    return cases


def _sample_q(W, rng):
    q = W.q_snap + rng.normal(0, 0.6, W.kin.n)
    q = np.clip(q, W.kin.lower, W.kin.upper)
    p_base = W.root_in_base[:3, :3] @ W.kin.fk(q)[0] + W.root_in_base[:3, 3]
    # 右臂常用工作区：身前、右侧、桌面以上
    if not (0.30 < p_base[0] < 0.85 and -0.65 < p_base[1] < 0.15 and TABLE_Z + 0.10 < p_base[2] < 1.35):
        return None, None
    return q, p_base


def _sample_case(W, scene, rng):
    qa, pa = _sample_q(W, rng)
    if qa is None:
        return None
    qb, pb = _sample_q(W, rng)
    if qb is None:
        return None
    L = float(np.linalg.norm(pb - pa))
    if not 0.20 < L < 0.55:
        return None
    extra = np.empty((0, 3))
    if scene in ("box", "wall"):
        mid = 0.5 * (pa + pb)
        u = pb - pa
        if np.linalg.norm(u[:2]) < 0.10:           # 主要是上下移动的不适合放竖板
            return None
        if scene == "box":                          # 桌上箱子，顶面高过连线 3~15cm
            top = mid[2] + rng.uniform(0.03, 0.15)
            h = (top - TABLE_Z) / 2
            box = _oriented_box(np.array([mid[0], mid[1], TABLE_Z + h]), u,
                                (rng.uniform(0.03, 0.06), rng.uniform(0.06, 0.15), h))
        else:                                       # 悬空竖板
            box = _oriented_box(mid, u, (0.02, rng.uniform(0.08, 0.18), rng.uniform(0.08, 0.16)))
        extra = W.base_to_root(box)
    obs = np.concatenate([W.base_obs, extra])
    W.set_obstacles(obs, 0.03)
    try:
        if not (W.valid(qa) and W.valid(qb)):
            return None
        ca, cb = W.clearance(qa), W.clearance(qb)
        if min(ca[0], cb[0]) < 0.02:                # 起终点本身贴着障碍的不要（真机也规划不了）
            return None
        # 关节直线是否被挡 → 区分"直线题"和"绕障题"（real 场景按实际情况分）
        n = int(np.ceil(np.max(np.abs(qb - qa)) / math.radians(4)))
        blocked = not all(W.valid(qa + s * (qb - qa)) for s in np.linspace(0, 1, n + 1))
        if scene in ("box", "wall") and not blocked:
            return None
    finally:
        W.set_obstacles(W.base_obs, 0.03)
    pr, Rr = W.kin.fk(qb)
    return dict(scene=scene, q0=qa.tolist(), p_goal=pr.tolist(), R_goal=Rr.tolist(), q_goal_ref=qb.tolist(),
                extra=extra.tolist(), blocked=bool(blocked), dist=L)


def profile_peaks(*prof):
    """按 _cart_traj 同样取点(cart_samples)，再按 JTC 的位置+速度三次样条求 (时长, 关节速度峰值, 关节加速度峰值)。"""
    from .arm_motion import cart_samples, spline_peaks
    tt, P, V = cart_samples(*prof)
    vel, acc = spline_peaks(tt, P, V)
    return float(tt[-1]), float(vel.max()), float(acc.max())


# ── 运行一题 ─────────────────────────────────────────────────────────
def run_one(args):
    name, case, time_limit = args
    from .arm_motion import cart_profile, cart_samples
    from .planners import PlanRequest, discover
    W = world(case["scene"])
    extra = np.asarray(case["extra"]).reshape(-1, 3)
    obs = np.concatenate([W.base_obs, extra])
    W.set_obstacles(obs, 0.03)
    planners, _ = discover()
    req = PlanRequest(solver=W.solver, valid=W.valid, q0=np.array(case["q0"]), p_goal=np.array(case["p_goal"]),
                      R_goal=np.array(case["R_goal"]), obstacles=obs, up=W.up_in_root(), time_limit=time_limit)
    t0 = time.monotonic()
    try:
        res = planners[name].factory().plan(req)
        err = None
    except Exception as e:                          # 规划器 bug 也记下来，不让整个 bench 挂
        res, err = None, f"{type(e).__name__}: {e}"
    dt = time.monotonic() - t0
    out = dict(planner=name, case=case["id"], scene=case["scene"], blocked=case["blocked"], dist=case["dist"],
               ok=False, t_plan=dt, note=err or (res.note if res else ""),
               valid_calls=(res.stats.get("valid_calls", 0) if res else 0))
    if res is None or not res.ok:
        W.set_obstacles(W.base_obs, 0.03)
        return out
    from .algorithms import STEP_JOINT_MAX, polish, rotvec
    jp = list(res.joint_path)
    q_fine = polish(W.kin, jp[-1], req.p_goal, req.R_goal, 30)       # 同 arm_motion.plan_detour 的终点精修
    if q_fine is not None and float(np.max(np.abs(q_fine - jp[-1]))) <= STEP_JOINT_MAX and W.valid(q_fine):
        jp.append(q_fine)
    jp = np.asarray(jp)
    # 复查：到位 + 全程不撞（逐 2°）
    p_end, R_end = W.kin.fk(jp[-1])
    pos_err = float(np.linalg.norm(p_end - req.p_goal))
    rot_err = float(np.linalg.norm(rotvec(req.R_goal @ R_end.T)))
    def path_clear(path):
        for qa, qb in zip(path[:-1], path[1:]):
            n = max(1, int(np.ceil(np.max(np.abs(qb - qa)) / math.radians(2))))
            if not all(W.valid(qa + s * (qb - qa)) for s in np.linspace(0, 1, n + 1)[1:]):
                return False
        return True
    clear = path_clear(jp)
    ee = np.array([W.kin.fk(q)[0] for q in jp])
    a, b = ee[0], np.asarray(req.p_goal)
    ab = (b - a) / max(np.linalg.norm(b - a), 1e-9)
    dev = float(np.max(np.linalg.norm((ee - a) - np.outer((ee - a) @ ab, ab), axis=1)))
    cl = [W.clearance(q) for q in jp[::max(1, len(jp) // 40)]] + [W.clearance(jp[-1])]
    prof = cart_profile(W.kin, jp, W.vel)
    T, qd_pk, qdd_pk, unsmoothed = None, 0.0, 0.0, False
    if prof is not None and clear and not path_clear(cart_samples(*prof)[1]):   # 同 _cart_traj：倒圆角会撞就退回
        prof, unsmoothed = cart_profile(W.kin, jp, W.vel, smooth=0.0), True
    if prof is not None:
        T, qd_pk, qdd_pk = profile_peaks(*prof)
    out.update(ok=bool(clear and pos_err < 0.005 and rot_err < math.radians(1)), recheck_clear=clear,
               pos_err=pos_err, rot_err=rot_err, n_points=len(jp),
               ee_len=float(np.sum(np.linalg.norm(np.diff(ee, axis=0), axis=1))),
               joint_travel=float(np.sum(np.abs(np.diff(jp, axis=0)))),
               max_dev=dev, exec_time=float(T or 0.0), qd_peak=qd_pk, qdd_peak=qdd_pk, unsmoothed=unsmoothed,
               min_clear_arm=float(min(c[0] for c in cl)), min_clear_hand=float(min(c[1] for c in cl)),
               segments=len(res.waypoints) - 1 if res.waypoints else None)
    if not clear:
        out["note"] += "  ★复查撞了"
    W.set_obstacles(W.base_obs, 0.03)
    return out


# ── 汇总 ─────────────────────────────────────────────────────────────
def summarize(rows, names, cases):
    by = {}
    for r in rows:
        by.setdefault(r["planner"], []).append(r)
    detour_ids = {c["id"] for c in cases if c["blocked"]}
    # 公平比较：绕障成功率 ≥70% 的规划器都解出的绕障题，在同一批题上比路程/时长；其余规划器(标 *)只能在自己解出的题上算
    rate = {n: np.mean([r["ok"] for r in by[n] if r["case"] in detour_ids]) if detour_ids else 1.0 for n in names}
    good = [n for n in names if rate[n] >= 0.7]
    common = [c for c in detour_ids if all(any(r["case"] == c and r["ok"] for r in by[n]) for n in good)]
    lines = [f"题目 {len(cases)} 道（绕障 {len(detour_ids)}；绕障成功率≥70% 的 {len(good)} 个规划器都解出 {len(common)} 道，用于公平比较）", ""]
    hdr = (f"{'规划器':16s} {'成功(全部)':>10s} {'成功(绕障)':>10s} {'规划中位':>8s} {'规划P90':>8s} {'查询中位':>8s} | "
           f"{'执行中位':>8s} {'规划+执行':>9s} {'路程/直线':>9s} {'关节转角':>8s} {'最远偏离':>8s} {'臂离障':>7s}")
    lines.append("绕障题（公平子集）上的路线质量：")
    lines.append(hdr)
    stats = {}
    for n in names:
        rs = by[n]
        ok_all = sum(r["ok"] for r in rs)
        rd = [r for r in rs if r["case"] in detour_ids]
        ok_d = sum(r["ok"] for r in rd)
        cs = [r for r in rs if (r["case"] in common if n in good else r["case"] in detour_ids) and r["ok"]]
        tp = [r["t_plan"] for r in rd if r["ok"]]
        f = lambda k, fn=np.median: float(fn([r[k] for r in cs])) if cs else float("nan")
        ratio = float(np.median([r["ee_len"] / r["dist"] for r in cs])) if cs else float("nan")
        st = dict(ok_all=ok_all, n_all=len(rs), ok_detour=ok_d, n_detour=len(rd),
                  t_plan_med=float(np.median(tp)) if tp else float("nan"),
                  t_plan_p90=float(np.percentile(tp, 90)) if tp else float("nan"),
                  calls_med=float(np.median([r["valid_calls"] for r in rd if r["ok"]])) if tp else float("nan"),
                  exec_med=f("exec_time"), total_med=float(np.median([r["t_plan"] + r["exec_time"] for r in cs])) if cs else float("nan"),
                  len_ratio=ratio, travel_med=f("joint_travel"), dev_med=f("max_dev"),
                  clear_arm_min=f("min_clear_arm", np.min), clear_hand_min=f("min_clear_hand", np.min),
                  clear_arm_med=f("min_clear_arm"), clear_hand_med=f("min_clear_hand"))
        stats[n] = st
        lines.append(f"{n + ('' if n in good else ' *'):16s} {ok_all:>4d}/{len(rs):<5d} {ok_d:>4d}/{len(rd):<5d} {st['t_plan_med']:>7.2f}s {st['t_plan_p90']:>7.2f}s "
                     f"{st['calls_med']:>8.0f} | {st['exec_med']:>7.1f}s {st['total_med']:>8.1f}s {ratio:>9.2f} "
                     f"{st['travel_med']:>7.2f}r {st['dev_med']*100:>6.1f}cm {st['clear_arm_med']*100:>5.1f}cm")
    lines.append("")
    lines.append("（路线质量列 = 公平子集上的中位数；臂离障 = 每条路径上手臂表面离障碍的最小距离再取中位）")
    return "\n".join(lines), stats


def main():
    from .planners import discover
    planners, errors = discover()
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--planners", nargs="*", default=list(planners), choices=list(planners))
    ap.add_argument("--cases", type=int, default=15, help="每个场景几道题（free 是它的 1/3）")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--time-limit", type=float, default=8.0)
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) - 2))
    ap.add_argument("--edge", nargs="*", default=[], metavar="KEY=VALUE",
                    help="覆盖 planners/_edges.EDGE_DEFAULTS（所有末端规划器共用的一段验证参数），如 clearance=0.06 relax_orient=0")
    ap.add_argument("--out", default="~/.ros/g2_motion/planner_bench.json")
    args = ap.parse_args()
    from .planners._edges import EDGE_DEFAULTS
    for kv in args.edge:                                   # fork 出来的工作进程继承改过的默认值
        k, v = kv.split("=", 1)
        if k not in EDGE_DEFAULTS:
            ap.error(f"--edge 没有 {k}（可选：{', '.join(EDGE_DEFAULTS)}）")
        EDGE_DEFAULTS[k] = type(EDGE_DEFAULTS[k])(float(v)) if not isinstance(EDGE_DEFAULTS[k], bool) else v not in ("0", "false", "False")
    if args.edge:
        print("一段验证参数：" + "，".join(f"{k}={v}" for k, v in EDGE_DEFAULTS.items()))
    for k, v in errors.items():
        print(f"规划器 {k} 加载失败：{v}")
    t0 = time.monotonic()
    cases = make_cases(args.cases, args.seed)
    print(f"生成 {len(cases)} 道题（{time.monotonic()-t0:.1f}s）："
          + "，".join(f"{s} {sum(c['scene'] == s for c in cases)}(绕障 {sum(c['scene'] == s and c['blocked'] for c in cases)})"
                     for s in ("real", "box", "wall", "free")), flush=True)
    jobs = [(n, c, args.time_limit) for c in cases for n in args.planners]
    rows = []
    with mp.get_context("fork").Pool(args.jobs) as pool:
        for i, r in enumerate(pool.imap_unordered(run_one, jobs), 1):
            rows.append(r)
            if i % 20 == 0 or i == len(jobs):
                print(f"  {i}/{len(jobs)}  {time.monotonic()-t0:.0f}s", flush=True)
    text, stats = summarize(rows, args.planners, cases)
    print("\n" + text)
    bad = [r for r in rows if r.get("recheck_clear") is False]
    if bad:
        print(f"\n★ 复查撞了的路径 {len(bad)} 条：" + "，".join(f"{r['planner']}/{r['case']}" for r in bad))
    out = Path(args.out).expanduser()
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(dict(args=vars(args), cases=cases, rows=rows, stats=stats), ensure_ascii=False, indent=1))
    print(f"\n明细：{out}")


if __name__ == "__main__":
    main()
