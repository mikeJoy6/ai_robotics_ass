"""G2 arm_motion IK (sanofi lineage) registered in the genie_motion test framework, for A/B comparison.

Same pose-only contract as the other algorithms here: only the measured start joints and the target
TCP pose go in, no recorded target joints. Pipeline = algorithms.ArmSolver.ik (q0 + 60 random seeds,
KDL LMA in full mode, nearest-to-q0 sort, null-space pull toward q0, nearest collision-free endpoint)
then a short genie adaptive-LM polish from that answer, because ArmSolver accepts at 3 mm / 0.02 rad
while this framework's common layer accepts at 0.1 mm / 0.001 rad.
"""
import time

import numpy as np

from . import Algorithm, CandidatePath, register
from ._adaptive_ik import solve_adaptive
from ...algorithms import ArmSolver

POLISH_ITERS = 30


@register('g2_seeded_lm', 'G2 arm_motion IK: q0 + 60 random seeds, nearest + null-space pull, collision-free endpoint; LM polish')
class G2SeededLm(Algorithm):
    def plan(self, request):
        if set(request.options) - {'seeds'}:
            raise ValueError('g2_seeded_lm only supports the seeds parameter')
        n = request.options.get('seeds', 60)
        if type(n) is not int or n < 0:
            raise ValueError('seeds must be a non-negative integer')
        request.check_deadline()
        started = time.monotonic()
        solver = ArmSolver(request.kin)
        q = solver.ik(request.goal_position, 'full', request.goal_rotation, None, request.start, n=n,
                      valid=request.check_state)
        coarse = time.monotonic() - started
        if q is None:
            request.log(f'g2_seeded_lm: no collision-free IK ({coarse*1000:.1f}ms)')
            return []
        result = solve_adaptive(request.kin, q, request.goal_position, request.goal_rotation,
                                POLISH_ITERS, request.deadline)
        request.log(f'g2_seeded_lm: coarse IK {coarse*1000:.1f}ms, polish {result.reason} '
                    f'({result.iterations} it, {result.position_error*1000:.4f}mm), '
                    f'joint distance to start {np.degrees(np.linalg.norm(result.q-request.start)):.1f}°')
        if not result.converged:
            return []
        return [CandidatePath('g2_seeded_lm', np.array([request.start, result.q]))]
