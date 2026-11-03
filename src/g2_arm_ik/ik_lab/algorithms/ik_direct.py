"""Pose-only IK using current/model-derived seeds, then straight joint connection."""
import numpy as np

from . import Algorithm, CandidatePath, register
from ..kinematics import solve_pose
from ._seeds import build_seeds


@register('ik', 'Target-pose-only IK: current/model seeds + direct path, no recorded answers')
class IkDirect(Algorithm):
    def plan(self, request):
        if set(request.options)-{'max_iters', 'seed_count'}:
            raise ValueError('ik algorithm only supports max_iters and seed_count parameters')
        max_iters = request.options.get('max_iters', 100)
        if type(max_iters) is not int or max_iters <= 0:
            raise ValueError('max_iters must be a positive integer')
        seed_count = request.options.get('seed_count', 3)
        if type(seed_count) is not int or seed_count <= 0:
            raise ValueError('seed_count must be a positive integer')
        candidates = []
        for label, seed in build_seeds(request)[:seed_count]:
            request.check_deadline()
            result = solve_pose(request.kin, seed, request.goal_position,
                                request.goal_rotation, max_iters=max_iters)
            if result.converged:
                candidates.append(CandidatePath(label, np.array([request.start, result.q])))
            else:
                request.log(f'{label} did not converge: position error {result.position_error:.5f} m')
        return candidates
