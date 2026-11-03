"""Keep the baseline pipeline, replacing only the single-seed numerical solver."""
from . import register
from .multi_ik_direct import MultiIkDirect
from ._adaptive_ik import solve_adaptive


@register('multi_ik_adaptive', 'Full-error adaptive LM/backtracking IK; multi-seed direct paths; minimum total joint travel')
class AdaptiveMultiIk(MultiIkDirect):
    algorithm_name = 'multi_ik_adaptive'

    def solve_seed(self, request, seed, ik_deadline, max_iters):
        return solve_adaptive(request.kin, seed, request.goal_position, request.goal_rotation,
                              max_iters, min(ik_deadline, request.deadline), request.orientation_mode)
