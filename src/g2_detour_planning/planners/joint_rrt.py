"""关节空间 RRT-Connect + 捷径平滑 —— OMPL 默认思路的复刻，只作对照（量化"抡大圈"）。

目标关节角 = 多种子 IK 里离 q0 最近的不撞解；关节空间双向树，步长 0.3rad，边每 2° 查碰撞。
不约束末端走直线，末端路线是什么样就是什么样。"""
import numpy as np

from ..algorithms import interp
from ._edges import EdgeChecker
from ._registry import register
from ._types import Planner, PlanResult

STEP = 0.3


@register("joint_rrt", "关节空间 RRT-Connect+捷径（OMPL 式，对照组）")
class JointRRT(Planner):
    def plan(self, req):
        E = EdgeChecker(req)
        q_goal = E.S.ik(E.p_goal, "full", E.R_goal, None, E.q0, n=60, valid=E.valid)
        if q_goal is None:
            return PlanResult(None, None, f"目标没有不撞的 IK 解（{E.ms()}）", E.stats)
        lo, hi = E.kin.lower, E.kin.upper
        lo = np.where(np.isfinite(lo), lo, -np.pi); hi = np.where(np.isfinite(hi), hi, np.pi)
        ok = lambda a, b: (E.stats.__setitem__("costly", E.stats["costly"] + 1) or
                           E.joint_path_clear([a, b]))
        if ok(E.q0, q_goal):
            return self._done(E, [E.q0, q_goal], "关节直线可达")
        rng = np.random.default_rng(0)
        trees = [([E.q0], [-1]), ([q_goal], [-1])]

        def extend(tree, x):
            V, par = tree
            i = int(np.argmin(np.linalg.norm(np.asarray(V) - x, axis=1)))
            d = x - V[i]
            n = float(np.linalg.norm(d))
            qn = V[i] + d * min(1.0, STEP / max(n, 1e-9))
            if not ok(V[i], qn):
                return None
            V.append(qn); par.append(i)
            return len(V) - 1

        def connect(tree, x):
            while True:
                j = extend(tree, x)
                if j is None:
                    return None
                if np.linalg.norm(tree[0][j] - x) < 1e-6:
                    return j

        a, b = 0, 1
        while not E.out_of_budget():
            x = rng.uniform(lo, hi)
            j = extend(trees[a], x)
            if j is not None:
                k = connect(trees[b], trees[a][0][j])
                if k is not None:
                    path = self._trace(trees[a], j)[::-1] + self._trace(trees[b], k)[1:]
                    if a == 1:
                        path = path[::-1]
                    return self._done(E, self._shortcut(E, path, ok, rng), "关节RRT-Connect")
            a, b = b, a
        return PlanResult(None, None, f"关节 RRT 失败（{E.stat_text()}，{E.ms()}）", E.stats)

    @staticmethod
    def _trace(tree, i):
        V, par = tree
        out = []
        while i >= 0:
            out.append(V[i]); i = par[i]
        return out

    @staticmethod
    def _shortcut(E, path, ok, rng, tries=60):
        path = list(path)
        for _ in range(tries):
            if len(path) < 3:
                break
            i, j = sorted(rng.choice(len(path), 2, replace=False))
            if j - i > 1 and ok(path[i], path[j]):
                path = path[:i + 1] + path[j:]
        return path

    @staticmethod
    def _done(E, path, kind):
        jp = [path[0]]
        for a, b in zip(path[:-1], path[1:]):
            jp += list(interp(a, b, 0.1)[1:])
        wps = [E.kin.fk(q)[0] for q in path]
        return PlanResult(jp, wps, E.done_text(kind, wps), E.stats)
