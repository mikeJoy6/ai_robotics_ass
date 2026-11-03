"""惰性 Theta*（任意角度 A*）：出队时先试从祖父节点直连，搜出来的就是少折点的折线，平滑前就接近最短。"""
from ._grid import grid_search
from ._registry import register
from ._types import Planner


@register("theta_star", "末端格子惰性 Theta*(任意角度，×2.5)+捷径平滑")
class ThetaStar(Planner):
    def plan(self, req):
        return grid_search(req, hw=2.5, any_angle=True, kind="Theta*")
