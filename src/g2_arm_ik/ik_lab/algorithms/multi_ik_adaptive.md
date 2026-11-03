# From fixed small-step DLS to full-error adaptive IK

[New algorithm entry](multi_ik_lm.py) · [Numeric solver](_adaptive_ik.py) · [Old algorithm and common flow](multi_ik_direct.py) · [Algorithm directory](README.md)

## What this real-robot result tells us

The user's round-trip log from the old multi_ik:

| Direction | Seeds tried | Distinct IK | Passed full direct-path check | Winning total joint travel | Planning time | Execution time |
| --- | --- | --- | --- | --- | --- | --- |
| To b | 24 | 1 | 1 | 296.70° | 0.493 s | 15.51 s |
| To a | 24 | 3 | 1 | 303.40° | 0.477 s | 14.89 s |

Both current seeds used all 200 iterations without reporting success; the successful seeds used 182, 186 and 195. The first action clearly SUCCEEDED; the log for the second stops at sending the execution request, so it can't be recorded as completed ahead of time. Planning time here was already under about half a second, not over 1 second; the current priority is single-solve convergence and diagnostics, not relaxing the time cap first.

3 IK solutions passing endpoint precision and limit re-checks doesn't mean all 3 direct paths are executable: the other two paths to a failed state validity checks at s≈0.234 and 0.203. "IK solved" can't be used to skip path checks. The invalid state returned by the service doesn't itself identify the specific colliding link; contact information is needed to explain further.

The fixed 2 mm error component clipping very likely affects progress on large errors, but "40 cm / 2 mm ≈ 200" is only an order-of-magnitude analysis, not a strict iteration lower bound: DLS position/orientation coupling, damping, limits and nonlinearity all affect the actual FK change. The old logs didn't print the final error on failure, so we can't yet prove every failure was just short of the last bit, nor determine the single reason a seed got stuck.

## Step 1: make the diagnostics accurate first

IkResult gains reason/details with defaults; old calls with five positional arguments remain compatible. The original solve_pose math update didn't change; DIK keeps using the old solver.

MultiIkDirect extracts "single-seed solving" into solve_seed(); the old algorithm still calls the old DLS in segments. The common flow prints for each seed:

```text
stop reason, cumulative iterations, final position error mm, final orientation error rad/deg, time ms
```

| reason | Meaning |
| --- | --- |
| converged | Meets the original position/orientation precision |
| iteration_limit | Iterations exhausted, still not precise enough |
| ik_deadline | Solve stage deadline reached, not finished |
| stalled | Adaptive version: several rounds of step shrinking/added damping still gave no real error decrease |
| constrained_stationary | First-order improvement along the current free directions is tiny, or all joints are frozen at bounds |
| linear_solve_failed / nonfinite_step | Numeric solve error |

For example, iteration_limit with a position error of 0.12 mm and with an error still at 200 mm are two different failures; now you can tell directly. Neither is quietly turned into a success.

## Step 2: keep the experiment shell, replace only one numeric solver interface

```python
@register('multi_ik_adaptive', 'Full-error adaptive LM/backtracking IK; multi-seed direct paths; minimum total joint travel')
class AdaptiveMultiIk(MultiIkDirect):
    algorithm_name = 'multi_ik_adaptive'

    def solve_seed(self, request, seed, ik_deadline, max_iters):
        return solve_adaptive(request.kin, seed, request.goal_position,
                              request.goal_rotation, max_iters,
                              min(ik_deadline, request.deadline))
```

The parent class remains responsible for current/model seeds, result deduplication, endpoint re-checks, per-candidate direct paths, collision sampling and minimum total joint travel. No target-joint answers, pose_ab untouched, the validated jogging DIK unchanged, and no new ROS package introduced. The algorithm name comes from register; the file name is multi_ik_lm.py.

## Step 3: use the full error instead of the fixed-clipped 2 mm

```text
e_p = p_target − p(q)
e_R = Log(R_target R(q)ᵀ)
e = [e_p; w e_R], w=0.2 m/rad
F(q) = 0.5 × eᵀe
```

w makes the relative scale of position and angle in the cost explicit; it's this version's initial value, not a calibrated one. Endpoint acceptance still checks real position/angle errors separately; a small F alone can't count as success.

Uses KDL's geometric Jacobian, and corrects the local linear relation of the Log orientation error via the inverse SO(3) right Jacobian to build A so that `e(q+Δq) ≈ e(q) − A Δq`. Here the target-relative-to-current error is `R_target R_currentᵀ`; where the rotation increment is applied and the multiplication order can't be swapped arbitrarily.

Damped update over the free joints:

```text
(A_freeᵀ A_free + λ² I) Δq_free = A_freeᵀ e
```

At limits, if the current cost-descent direction points outside the bounds, that joint is frozen first and the other joints' update is re-solved; trial values are still projected onto the real lower/upper limits. This local bound handling doesn't guarantee all coupled bound problems are solved.

## Step 4: large errors can try large steps, but every step must be verified

The initial max per-joint numeric step is 0.25 rad, which can later grow to 0.6 rad depending on performance. It isn't a new total travel rejection threshold, much less a 0.6 rad jog sent to the real robot.

1. Compute the damped update Δq.
2. If the largest per-joint update exceeds the current radius, scale the whole update down.
3. Compute the real FK and F at q+Δq, rather than trusting only the linear model.
4. If F doesn't decrease, try 1/2, 1/4… of the step in turn, up to 8 times.
5. Accept a decreasing trial; adjust damping and step radius by the ratio of actual to predicted decrease.
6. If a whole round fails, increase damping and shrink the radius; after 6 consecutive rounds with no improvement report stalled.

So "larger steps" isn't blindly increasing the clipping value; it has real error acceptance and fallback. Iterations can still hit local minima, stall near singularities, or run into branch problems of the orientation Log near π; current isn't guaranteed to succeed. This implementation is the project's own simplified adaptive damping/backtracking scheme, not equivalent to SciPy's full TRF algorithm; see [SciPy trust-region bound handling source](https://github.com/scipy/scipy/blob/main/scipy/optimize/_lsq/trf.py) for how a more complete method differs; this version doesn't call or add a SciPy dependency.

## Step 5: precision and path acceptance stay strict

Position error must still be under 0.1 mm and orientation error under 0.001 rad. Success conditions weren't relaxed on the grounds of "very close". Logs will show near-target misses, and then we'll decide whether to add a separate precision config that fits task needs.

Solved IK is still re-checked and deduplicated independently; each candidate makes a joint direct path from the measured start and calls the same state check; the smallest total travel is chosen only among the legal set found and fully checked in this run. Three endpoint solutions may still have only one legal direct path; this can't be replaced with "solve succeeded, so execute".

In the new algorithm, ordinary solve steps don't run collision checks; numeric iteration may pass through any virtual state; the real execution connection is checked by the later path layer. The q sequence inside the solver can't be used as a real-robot trajectory.

## How to compare

Keep the same A/B, scene, start, budget, seeds and iteration cap:

```bash
ros2 run genie_motion pose_ab --algorithm multi_ik --plan-timeout 1
ros2 run genie_motion pose_ab --algorithm multi_ik_adaptive --plan-timeout 1
```

Only one entry point controls the right arm at a time. Focus on recording current's reason and error, each successful solution's iteration count, the number of distinct solutions, the number of legal direct paths, total compute time and final travel. Don't claim a higher grasp success rate just because the new algorithm finds more endpoints.

This version still defaults to max_iters=200, at most 24 seeds, a target of 5 distinct solutions, and IK taking 50% of the remaining budget. Adaptive solving may run several FKs per iteration, so fewer iterations doesn't automatically mean less time. Since error scaling, damping strategy, orientation linearization and backtracking all changed at once, any improvement shouldn't be attributed solely to removing the 2 mm clipping.

This change ran no offline IK/path computation and no real-robot motion; only the old algorithm's logs are actual results provided by the user. The new algorithm passed syntax, registration discovery and build checks; its numeric effect is still to be verified in practice.
