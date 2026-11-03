"""末端空间 3D 格子上的惰性 A* 家族（astar_* / theta_star 共用）。

以起点为原点铺 res 格子，26 邻域。邻居入队时不验证，出队才验证"父节点关节角 → 这一格"一段（大部分格子永远不验证）。
  g = 末端路程 + joint_weight·关节转动(rad) + clear_weight·贴近障碍的罚分
  f = g + hw·到目标直线距离（hw>1 = 加权 A*，更贪心、快，路线稍长）
  any_angle=True(Theta*)：出队时先试从"父节点的父节点"直连这一格，连得上就跳过父节点 → 搜出来就接近折线最短。
出队的格子离目标 ≤ 1.5·res 时试连目标，连上就结束；最后统一捷径平滑。
"""
import heapq
import itertools

import numpy as np

from ._edges import DIRS26, EdgeChecker
from ._types import PlanResult


def grid_search(req, hw=2.5, res=0.05, joint_weight=0.05, clear_weight=0.0, clear_dist=0.20,
                any_angle=False, margin=0.35, kind="网格绕行", **edge_kw):
    E = EdgeChecker(req, res=res, **edge_kw)
    why = E.goal_check()
    if why:
        return PlanResult(None, None, why, E.stats)
    direct = E.direct()
    if direct is not None:
        return PlanResult(direct, [E.p_start, E.p_goal], f"直线可达（{E.ms()}）", E.stats)

    lo = np.minimum(E.p_start, E.p_goal) - margin
    hi = np.maximum(E.p_start, E.p_goal) + margin
    pos_of = lambda k: E.p_start + np.array(k, dtype=float) * res
    start = (0, 0, 0)
    closed = {start: (E.q0, None, 0.0)}           # key -> (q, parent_key, g)
    point = lambda k: E.p_goal if k == "goal" else (E.p_start if k == start else pos_of(k))
    counter = itertools.count()
    heap = []

    def penalty(p, step):
        if clear_weight <= 0:
            return 0.0
        d = E.clearance_at(p)
        return clear_weight * step * max(0.0, (clear_dist - d) / clear_dist)

    def push_neighbors(k, g):
        for d in DIRS26:
            nk = (k[0] + int(d[0]), k[1] + int(d[1]), k[2] + int(d[2]))
            if nk in closed:
                continue
            np_ = pos_of(nk)
            if np.any(np_ < lo) or np.any(np_ > hi):
                continue
            step = float(np.linalg.norm(d)) * res
            g2 = g + step + penalty(np_, step)
            heapq.heappush(heap, (g2 + hw * float(np.linalg.norm(np_ - E.p_goal)), next(counter), g2, nk, k))

    def attach(k, parent, seg, pb):
        q_par, _, g_par = closed[parent]
        travel = float(np.sum(np.abs(np.diff(np.asarray(seg), axis=0))))
        step = float(np.linalg.norm(pb - point(parent)))
        g = g_par + step + joint_weight * travel + penalty(pb, step)
        closed[k] = (seg[-1], parent, g)
        return g

    push_neighbors(start, 0.0)
    found = False
    while heap and not E.out_of_budget():
        _, _, _, k, parent = heapq.heappop(heap)
        if k in closed:
            continue
        pb = pos_of(k)
        seg = None
        if any_angle and closed[parent][1] is not None:          # Theta*：先试跳过父节点
            gp = closed[parent][1]
            seg = E.edge(closed[gp][0], point(gp), pb, E.R_at(pb))
            if seg is not None:
                parent = gp
        if seg is None:
            seg = E.edge(closed[parent][0], point(parent), pb, E.R_at(pb))
        if seg is None:
            continue
        g = attach(k, parent, seg, pb)
        if np.linalg.norm(pb - E.p_goal) <= 1.5 * res:
            last = E.edge(seg[-1], pb, E.p_goal, E.R_goal)
            if last is not None:
                closed["goal"] = (last[-1], k, g)
                found = True
                break
        push_neighbors(k, g)

    if not found:
        why = "超出边数/时间上限" if heap else "可走的格子都试完了"
        return PlanResult(None, None, f"{kind}失败（{why}；{E.stat_text()}，{E.ms()}）", E.stats)
    chain, k = [], "goal"
    while k is not None:
        chain.append((closed[k][0], point(k)))
        k = closed[k][1]
    chain.reverse()
    jp, wps = E.shortcut(chain)
    if jp is None:
        return PlanResult(None, None, "平滑时相邻路点连不上（状态变了？）", E.stats)
    return PlanResult(jp, wps, E.done_text(kind, wps, len(chain)), E.stats)
