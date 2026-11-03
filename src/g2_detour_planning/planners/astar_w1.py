"""标准惰性 A*（启发 ×1，可采纳）：格子图上路程最优，代价是展开更多格子、更慢。"""
from ._grid import grid_search
from ._registry import register
from ._types import Planner


@register("astar_w1", "末端 5cm 格子标准惰性 A*(启发×1，格子图最优)+捷径平滑")
class AStarW1(Planner):
    def plan(self, req):
        return grid_search(req, hw=1.0, kind="A*")
