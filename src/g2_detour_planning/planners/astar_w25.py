"""加权惰性 A*（启发 ×2.5）+ 捷径平滑 —— 原 ee_search.EESearch，搬过来不改参数，作为基线。"""
from ._grid import grid_search
from ._registry import register
from ._types import Planner


@register("astar_w25", "末端 5cm 格子加权惰性 A*(启发×2.5)+捷径平滑（原 ee_search 基线）")
class AStarW25(Planner):
    def plan(self, req):
        return grid_search(req, hw=2.5, kind="加权A*")
