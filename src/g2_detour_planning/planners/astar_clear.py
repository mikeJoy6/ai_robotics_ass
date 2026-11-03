"""离障加权 A*：离 octomap 点 < 20cm 的格子路程加罚（越近罚越多），路线主动离障碍远一点，安全余量更大。"""
from ._grid import grid_search
from ._registry import register
from ._types import Planner


@register("astar_clear", "加权 A*(×2.5)+离障罚分(<20cm 线性加罚)：路线离障碍更远")
class AStarClear(Planner):
    def plan(self, req):
        return grid_search(req, hw=2.5, clear_weight=1.5, clear_dist=0.20, kind="离障A*")
