"""混合：先试绕行模板（最多 40 个通过粗筛的，绝大多数 U 型绕行几十毫秒出结果），不行再加权 A* 兜底。"""
from ._edges import EdgeChecker
from ._grid import grid_search
from ._registry import register
from ._types import Planner, PlanResult
from .via_points import try_templates


@register("via_then_astar", "先绕行模板(≤40 个)，不通再加权 A*(×2.5) 兜底")
class ViaThenAStar(Planner):
    def plan(self, req):
        E = EdgeChecker(req)
        why = E.goal_check()
        if why:
            return PlanResult(None, None, why, E.stats)
        direct = E.direct()
        if direct is not None:
            return PlanResult(direct, [E.p_start, E.p_goal], f"直线可达（{E.ms()}）", E.stats)
        jp, wps, name = try_templates(E, req, max_templates=40)
        if jp is not None:
            return PlanResult(jp, wps, E.done_text(f"{name} 型绕行", wps), E.stats)
        res = grid_search(req, hw=2.5, kind="模板不通→加权A*")
        for k, v in E.stats.items():
            res.stats[k] = res.stats.get(k, 0) + v
        return res
