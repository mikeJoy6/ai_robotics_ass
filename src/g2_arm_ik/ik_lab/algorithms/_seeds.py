"""Pose-only seed proposals: current joints/model geometry, no target answers."""
import numpy as np

from ..pose_task import orientation_distance, task_nullspace, travel_weights


def build_seeds(request):
    kin = request.kin
    start = np.clip(request.start, kin.lower, kin.upper)
    middle = start.copy()
    finite = np.isfinite(kin.lower) & np.isfinite(kin.upper)
    middle[finite] = 0.5*(kin.lower[finite]+kin.upper[finite])
    proposals = [('model-middle', middle)]
    basis, rank = task_nullspace(kin, start, request.goal_rotation, request.orientation_mode)
    request.log(f'Start task Jacobian rank={rank}, null-space dim={basis.shape[1]}')
    for axis, direction in enumerate(basis.T):
        for offset in (0.35, -0.35, 0.7, -0.7, 1.0, -1.0):
            proposals.append((f'current-null{axis+1}{offset:+.2f}', start+offset*direction))
    for origin_name, origin in [('current', start), ('middle', middle)]:
        for offset in (0.45, -0.45, 0.9, -0.9):
            for j in range(kin.n):
                q = origin.copy()
                q[j] += offset
                proposals.append((f'{origin_name}-j{j+1}{offset:+.2f}', q))
    seen, ranked = [start], []
    for label, q in proposals:
        request.check_deadline()
        q = np.clip(q, kin.lower, kin.upper)
        if any(np.max(np.abs(q-old)) < 1e-8 for old in seen):
            continue
        seen.append(q.copy())
        p, rotation = kin.fk(q)
        # Cheap heuristic, not an IK solution or reachability proof. Units are
        # balanced by explicit heuristic weights; exact endpoint checks follow IK.
        cost = (np.linalg.norm(p-request.goal_position)
                + 0.2*orientation_distance(rotation, request.goal_rotation, request.orientation_mode)
                + 0.02*np.linalg.norm((q-start)*travel_weights(kin, request.orientation_mode)))
        ranked.append((float(cost), label, q))
    ranked.sort(key=lambda item: item[0])
    request.log('Seed sources: current measured joints, model midpoint and perturbations; heuristically ranked by target FK error, no recorded answers')
    return [('current', start)]+[(label, q) for _, label, q in ranked]
