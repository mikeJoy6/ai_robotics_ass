"""Multiple distinct IK goals, validate every straight path, minimize total travel.

Step-by-step construction guide: multi_ik_direct.md.
"""
import math
import time

import numpy as np

from . import Algorithm, CandidatePath, register
from ..kinematics import IkResult, rotation_vector, solve_pose
from ._seeds import build_seeds
from ..pose_task import orientation_distance, travel_weights


@register('multi_ik', 'Up to 5 distinct IK for the same TCP; check all direct paths; pick the smallest total joint travel')
class MultiIkDirect(Algorithm):
    algorithm_name = 'multi_ik'

    def seed_search(self, request, seeds):
        return seeds

    def solve_seed(self, request, seed, ik_deadline, max_iters):
        """Old fixed-clipping DLS, kept as a baseline with precise diagnostics."""
        q, used = seed.copy(), 0
        result = None
        while used < max_iters and time.monotonic() < ik_deadline:
            request.check_deadline()
            result = solve_pose(request.kin, q, request.goal_position, request.goal_rotation,
                                max_iters=min(10, max_iters-used))
            used += result.iterations
            q = result.q
            if result.converged:
                break
        p, rot = request.kin.fk(q)
        reason = ('converged' if result is not None and result.converged else
                  'ik_deadline' if time.monotonic() >= ik_deadline else 'iteration_limit')
        return IkResult(q, reason == 'converged', float(np.linalg.norm(p-request.goal_position)),
                        float(np.linalg.norm(rotation_vector(rot, request.goal_rotation))), used, reason)

    def plan(self, request):
        # 1. Read this algorithm's settings, not ROS/execution settings.
        options = request.options
        unknown = set(options)-{'candidate_count', 'max_attempts', 'max_iters',
                                'distinct_deg', 'ik_fraction'}
        if unknown:
            raise ValueError(f'{self.algorithm_name} does not support parameters: {sorted(unknown)}')
        count = options.get('candidate_count', 5)
        attempts = options.get('max_attempts', 24)
        max_iters = options.get('max_iters', 200)
        for name, value in [('candidate_count', count), ('max_attempts', attempts), ('max_iters', max_iters)]:
            if type(value) is not int or value <= 0:
                raise ValueError(f'{name} must be a positive integer')
        if not 3 <= count <= 5:
            raise ValueError('candidate_count must be between 3 and 5')
        distinct = options.get('distinct_deg', 3.0)
        fraction = options.get('ik_fraction', 0.5)
        for name, value in [('distinct_deg', distinct), ('ik_fraction', fraction)]:
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f'{name} must be finite')
        if distinct <= 0 or not 0 < fraction < 1:
            raise ValueError('distinct_deg must be greater than 0; ik_fraction must be between 0 and 1')
        request.check_deadline()
        ik_deadline = time.monotonic() + fraction*(request.deadline-time.monotonic())
        request.log(f'[{self.algorithm_name} 1/6] goal: collect {count} distinct IK solutions, trying at most {attempts} seeds')

        # 2. Proposals use ONLY measured start/model and target FK residual.
        # No recorded goal joints exist in this algorithm's PlanningRequest.
        kin = request.kin
        seeds = self.seed_search(request, build_seeds(request))
        weights = travel_weights(kin, request.orientation_mode)
        request.log(f'Travel cost weights (in current joint order)={weights.tolist()}')
        distinct_solutions, tried_seeds = [], []
        request.log(f'[{self.algorithm_name} 2/6] current/model seeds: no target-joint answers received')
        for label, seed in seeds:
            if len(tried_seeds) >= attempts or len(distinct_solutions) >= count:
                break
            if time.monotonic() >= ik_deadline:
                request.log('IK stage budget exhausted; keeping time to check paths for existing candidates')
                break
            request.check_deadline()
            seed = np.clip(seed, kin.lower, kin.upper)
            if any(np.max(np.abs(seed-old)) < 1e-8 for old in tried_seeds):
                continue
            tried_seeds.append(seed.copy())

            # 3. The subclass may replace only the single-seed solver.
            # 5D can augment seeds after each verified solution; all modes keep
            # the endpoint tolerances and real full-joint path checks.
            started = time.monotonic()
            result = self.solve_seed(request, seed, ik_deadline, max_iters)
            request.log(f'[{self.algorithm_name} 3/6] {label}: {result.reason}, '
                        f'iterations={result.iterations}, position error={result.position_error*1000:.4f}mm, '
                        f'orientation error={result.rotation_error:.6f}rad/'
                        f'{math.degrees(result.rotation_error):.4f}°, '
                        f'time={(time.monotonic()-started)*1000:.2f}ms {result.details}')
            if not result.converged:
                continue
            q = result.q
            # Independently verify the actual endpoint, not just solver status.
            p, rot = kin.fk(q)
            if (not np.all(np.isfinite(q)) or not np.all(np.isfinite(p))
                    or not np.all(np.isfinite(rot)) or np.any(q < kin.lower) or np.any(q > kin.upper)
                    or np.linalg.norm(p-request.goal_position) > 1e-4
                    or orientation_distance(rot, request.goal_rotation, request.orientation_mode) > 1e-3):
                request.log(f'{label}: endpoint re-check failed')
                continue
            if any(np.max(np.abs(q-old)) < math.radians(distinct)
                   for _, old in distinct_solutions):
                request.log(f'{label}: duplicate of an existing solution, not counted as a candidate')
                continue
            distinct_solutions.append((label, q.copy()))
            request.log(f'[{self.algorithm_name} 3/6] accepted IK {len(distinct_solutions)}/{count}: {label}, iterations {result.iterations}')
            if hasattr(seeds, 'accept') and len(distinct_solutions) < count:
                seeds.accept(label, q)
        request.log(f'Got {len(distinct_solutions)} distinct IK solutions; tried {len(tried_seeds)} seeds')
        if len(distinct_solutions) < 3:
            request.log('Fewer than 3 distinct solutions; continuing with the actual candidates; not duplicating a solution to pad the count')

        # 4. For every distinct IK endpoint, sample the entire straight joint edge
        # using the existing state-validity callback. Do not stop at first success.
        legal = []
        for label, q in distinct_solutions:
            request.check_deadline()
            travel = np.abs(q-request.start)
            total = float(np.sum(travel*weights))
            samples = max(1, math.ceil(float(np.max(travel))/math.radians(2)))
            request.log(f'[{self.algorithm_name} 4/6] checking {label}: joint-term scored travel {math.degrees(total):.2f}°, '
                        f'real total travel {math.degrees(float(np.sum(travel))):.2f}°, {samples+1} samples')
            valid = True
            for index, s in enumerate(np.linspace(0, 1, samples+1)):
                request.check_deadline()
                if not request.check_state(request.start+s*(q-request.start)):
                    request.log(f'{label}: state check failed at sample {index}/{samples}, s={s:.3f}')
                    valid = False
                    break
            if valid:
                legal.append((total, label, q))
                request.log(f'[{self.algorithm_name} 5/6] {label}: direct path check passed')
        if not legal:
            raise ValueError(f'{self.algorithm_name} has no legal direct path: check the IK count and each path check result')

        # 5. Weighted joint travel plus Link6 spin in tool_z mode.
        # Return ONLY the winner so the public layer cannot re-rank the set with
        # a different objective. It still independently validates the winner.
        total, label, q = min(legal, key=lambda item: item[0])
        request.check_deadline()
        request.log(f'[{self.algorithm_name} 6/6] selected {label} out of {len(legal)} legal direct paths, minimum scored travel {math.degrees(total):.2f}°')
        return [CandidatePath(f'{self.algorithm_name}:{label}', np.array([request.start, q]))]
