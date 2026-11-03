# Building multi_ik from scratch: same TCP, multiple solutions, per-candidate direct paths, pick the smallest total travel

[Algorithm source](multi_ik_direct.py) · [Algorithm directory docs](README.md) · [Package docs](../../README.md)

This is a staged build tutorial you can reproduce by following the source, recording each step's goal, existing interfaces, inputs/outputs, gaps and next step. The snippets below are teaching stages; don't save each one as a separately registered module. What actually runs is the single complete algorithm in multi_ik_direct.py.

Input correction: an earlier version of this tutorial used the recorded target joints and their perturbations as seeds, which leaks the answer; that has been removed. Earlier successes of that kind can't count as passing a real IK test. The text below is in sync with the current code.

## What we want in the end

```text
current joints q_start + one target TCP pose
    ↓ generate multiple seeds, each solving the same target
up to 5 non-duplicate, precise IK joint solutions
    ↓ each solution gets a joint direct path from the same measured start
call the existing check_state on each one, checking the whole direct path
    ↓ discard direct paths that fail the checks
from the remaining paths pick the one with the smallest Σ|q_goal − q_start|
    ↓ common layer re-verifies, adds timing, entry point hands it to JTC
```

Here 3–5 is the number of distinct solutions we hope to collect, not a guarantee that any pose has 5 solutions. The default target is 5; in practice there may be only 0, 1 or 2. If they can't be found, report it honestly; don't copy one solution 5 times, and don't treat having fewer than 3 candidates as proof the endpoint is unreachable.

All candidates target the same position + rotation, not just the position. The current request is provided by the right-arm A/B entry point; the pose is in the same frame as kin's chain root, i.e. base_link_r, so there's no need to convert base_link again inside the algorithm.

## Step 1: get the entry point to recognize me, no solving yet

**Do now:** make sure the new file can be discovered and the interface fits. Don't touch ROS nodes, JTC or the keyboard yet.

The existing [__init__.py](__init__.py) already exports Algorithm, CandidatePath and register; no need to write another registration mechanism.

```python
from . import Algorithm, CandidatePath, register

@register('multi_ik', 'Pick the shortest legal direct path among multiple IK candidates')
class MultiIkDirect(Algorithm):
    def plan(self, request):
        return []
```

The input is the existing PlanningRequest; the output is an empty list for now. After building, `--list-algorithms` shows multi_ik, which only proves registration succeeded, not that the algorithm can plan. If this stage is actually called, the common layer should report no complete candidate and must not send motion.

**Stage check:** Does the name appear? Does the class implement plan? No Publisher created?

**Still missing:** no endpoint joints at all, let alone a path. Next, use the known recorded joints to get the data structures working.

## Step 2: be explicit about inputs; target-joint answers must not come in

The plain PlanningRequest only has the current joints start, the target goal_position/goal_rotation, the model, callbacks and budget; there's no goal_joints. Test targets come from recorded TCP poses, but the 7 joints at recording time must not be passed to IK.

So you can't return `[q_start, recorded q_goal]` first and pass it off as IK, nor use the recorded q_goal as a high-quality seed. Both can still stay in the data file for comparison, but only the explicit joint replay baseline may access target joints.

**Stage check:** Does the request structure really have no target-joint field? Can every seed be explained as coming from the current state or the model?

**Still missing:** only a target pose, no endpoint joints yet. Next, reference IK and use the current joints as the first seed.

## Step 3: reference existing IK; don't rewrite FK and the Jacobian

The existing solve_pose in [kinematics.py](../kinematics.py) takes a seed and a target pose and returns an IkResult:

```python
from ..kinematics import solve_pose

result = solve_pose(request.kin, seed,
                    request.goal_position, request.goal_rotation,
                    max_iters=200)
if result.converged:
    q_goal = result.q
```

| Input/output | Contents |
| --- | --- |
| kin | Kinematic model, FK, Jacobian and limits |
| seed | 7-D initial joint values, not a TCP position |
| goal_position / goal_rotation | The single fixed target TCP pose |
| result.q | The solved joint values; may exist even when not converged, only for continuing numeric iteration |
| result.converged | Whether the solver's precision and limit conditions are met |
| result.iterations | Number of iterations used in this call |

Only when result.converged is true is q treated as an endpoint candidate. A non-converged result.q can't be sent to the robot as an executable path on the grounds that "it already moved forward".

**Stage check:** previously only joint replay; now one seed can solve the target TCP; record the error and whether it converged.

**Still missing:** a single seed may fail, or always yield the same configuration. Next, add structured seeds.

## Step 4: not solving five different targets, but solving the same target from multiple initial guesses

Seeds are currently generated by [_seeds.py](_seeds.py); the current measured pose comes first, and extra seeds are sorted by an FK heuristic cost:

1. Current measured joints.
2. Joint midpoints computed from the URDF limits.
3. ± perturbations from the current joints along local redundancy directions.
4. ± single-joint perturbations of the current/midpoint poses; extra seeds sorted by a heuristic cost of target FK residual and distance from the start.

All seeds solve the same goal_position/goal_rotation. What's perturbed is the numeric solver's initial value; the user's target isn't changed, and the real robot isn't driven to these seed poses first.

The local redundancy direction reuses the existing `kin.jacobian(q)`:

```python
_, _, vt = np.linalg.svd(kin.jacobian(request.start), full_matrices=True)
direction = vt[-1]
seed = request.start + offset * direction
```

When the current 6×7 Jacobian has its usual full row rank, there is one local null-space direction. The last right singular vector of the SVD provides such local seed guidance; after a finite-magnitude move IK is still needed again, with no guarantee the TCP stayed fixed, nor that both C and Z configurations are obtained. At singularities the null space may have more than one dimension; this version only uses the last vector as a heuristic.

Joint perturbations provide extra starts; all seeds are first clipped to the model bounds, and duplicate seeds aren't solved again. The whole process neither reads the recorded target joints nor computes the Jacobian at the target joints. This is a finite deterministic attempt, not a full enumeration of the solution space.

**Still missing:** different seeds may well converge to the same solution. Next, results must be deduplicated, rather than counting calls.

## Step 5: precision re-check and deduplication give the real candidate count

After the solver reports success, still re-check that the joints are finite and within limits, and the FK of that q: position error at most 1e-4 m, orientation error at most 1e-3 rad.

Then compare it with the already accepted joint solutions:

```python
duplicate = any(
    np.max(np.abs(q - old)) < math.radians(distinct_deg)
    for _, old in distinct_solutions)
```

Default distinct_deg=3: if every joint differs from some existing solution by less than 3°, it's a duplicate; at least one joint must differ by 3° to become a new candidate. Joints with limits aren't taken modulo ±π.

This definition only means the joint vectors differ; it doesn't guarantee they belong to completely different elbow configuration families. Logs show each seed's converged, duplicate or accepted result, and the final actual count.

**Still missing:** if every seed computes forever, the test program becomes unresponsive. Next, handle time and count caps.

## Step 6: reserve time separately for solving and path checking

The request already has an overall deadline; this version defaults to at most 24 seed attempts, at most 200 IK iterations each, stopping once 5 distinct solutions are collected. The first half of the remaining time goes to IK and the rest is reserved for direct-path checks.

The old solve_pose has no internal deadline parameter. To avoid having no deadline check at all during a 200-iteration run, this algorithm splits iterations of the same seed into chunks of at most 10:

```python
result = solve_pose(kin, q, target_position, target_rotation, max_iters=chunk)
q = result.q  # if not converged, continue from the previous chunk's numeric result
```

The deadline is checked between chunks, and the cumulative used count never exceeds max_iters. The model and target don't change, nor do damping and other parameters; the intermediate q is always just an in-memory numeric solve state and is never sent to JTC.

This check can't forcibly interrupt a running KDL call or a service call. When the IK stage expires, continue with the results collected so far; when the overall request expires, this plan fails and no stale result is published.

**Still missing:** finding a legal endpoint doesn't mean the segment from start to end is legal. Next, generate and check paths.

## Step 7: generate a joint direct path for every candidate and call the existing collision check

For candidate q_goal:

```python
q(s) = q_start + s * (q_goal - q_start)  # 0 <= s <= 1
```

To keep the max per-joint sampling interval at most 2°, compute the sample count and call the existing callback at each point:

```python
for s in np.linspace(0, 1, samples + 1):
    request.check_deadline()
    if not request.check_state(request.start + s * (q - request.start)):
        valid = False
        break
```

request.check_state has already been wired by pose_ab to the existing `/check_state_validity`, which checks after replacing the right-arm joints in the current whole-body snapshot. This algorithm doesn't need to create another MoveIt client. On failure it prints the specific sample index and s; legal paths are added to legal.

**You must keep checking the next candidate**; you can't return as soon as the first legal path is found, since a later one may be a shorter legal path. Once a path has a failed sample you can stop checking the rest of it; it's already illegal, so no need to waste service calls.

2° is discrete sampling, not a continuous collision proof; the service uses the robot and environment models already in the scene. It doesn't guarantee finding small collisions between two samples, and doesn't mean obstacles are avoided automatically.

**Still missing:** we have several legal paths, but which is best? Next, write "total travel" as an explicit objective function.

## Step 8: select strictly by total joint travel, not by first success or largest joint change

For this version's monotonic joint direct paths:

```text
L = Σ_j |q_goal[j] − q_start[j]|
```

L is computed in rad and converted to degrees in the logs. It's the sum of each joint's travel, not the TCP's length in meters. When ranking, every joint has the same weight, with no extra max-joint penalty.

```python
total, label, q = min(legal, key=lambda item: item[0])
return [CandidatePath(f'multi_ik:{label}', np.array([request.start, q]))]
```

For example, if legal candidate A has a total travel of 200°, B 180° and C 220°, pick B; whether the largest single joint angle exceeds 90° is only the existing warning and isn't a hard rejection in this algorithm.

This is only the minimum **within the candidate set found and fully checked this time**, not the global shortest solution. If the overall deadline is hit, the request is aborted even if a legal path has already been checked, because the comparison hasn't been completed as agreed, and unchecked candidates can't be quietly treated as failures.

The original score in the common planning.py was `sum(travel)+0.5*max(travel)`. If all results were handed back to it directly, it might pick a different one. So this algorithm checks and compares all candidates itself and returns only the single winning path. The common layer still independently re-verifies that path, prints its own common score and does time parameterization, but by then there are no other paths for it to re-rank.

## Step 9: why nothing has actually moved after returning

The algorithm doesn't execute on the robot. The subsequent order is:

1. planning.py re-checks the winning path; the along-path collision sampling may be repeated, which is currently kept to keep acceptance uniform.
2. Generate a JointTrajectory with positions, velocities, accelerations and times.
3. pose_ab re-checks the measured start, other joint states and the deadline.
4. Send via the right-arm JTC action and wait for success, failure or cancellation.

This lets the algorithm file focus on solving without rewriting the feedback thread, keyboard, A/B file and action management. Repeated common checks also take time; this version doesn't yet cache validity for the same request/scene.

## Usage and log order

After building, multi_ik shows up automatically without changing pose_ab:

```bash
source /opt/ros/humble/setup.bash
source ~/g1_pointcloud_grasping/install/setup.bash
ros2 run genie_motion pose_ab --list-algorithms
ros2 run genie_motion pose_ab --algorithm multi_ik
```

The existing entry point's default total budget is 1 second; this version's multiple IK runs and per-point ROS service calls haven't been verified to finish within 1 second. When troubleshooting while learning you can explicitly relax the budget, without treating that as having met the performance goal:

```bash
ros2 run genie_motion pose_ab --algorithm multi_ik --plan-timeout 5 \
  --algorithm-options '{"candidate_count":5,"max_attempts":24,"max_iters":200,"distinct_deg":3,"ik_fraction":0.5}'
```

| Parameter | Default | Meaning |
| --- | --- | --- |
| candidate_count | 5 | Number of distinct IK solutions wanted, only 3–5 allowed |
| max_attempts | 24 | Max number of distinct seeds to try |
| max_iters | 200 | Max cumulative iterations per seed |
| distinct_deg | 3 | Result dedup threshold defined by max single-joint difference |
| ik_fraction | 0.5 | Fraction of the remaining time allocated to the IK stage |

Log numbers 1/6–6/6 correspond to config, seeds, solving/dedup, path checks, legal path confirmation and minimum travel selection. Without actual run data, don't pre-fill "got five solutions", "all collision-free" or specific timings.

## After this version, which gaps to upgrade next

- If there's only one distinct solution: look at seed coverage and dedup results, then change seeds/the solver; don't add repeated calls to pad the count.
- If many seeds don't converge: this still reuses the jogging DLS; the next version will consider solve steps, damping and bound handling better suited to large errors.
- If endpoints all solve but every direct path fails: path search must be added; this version has no intermediate obstacle-avoidance points, and renaming won't fix it.
- If service checks take most of the time: consider same-request state caching or local collision checking, optimizing performance only while keeping the scene consistent.
- If you want to judge physical detours: currently only total joint travel is minimized; elbow/TCP spatial paths, singularity and dynamics aren't evaluated yet.

This change only did syntax, discovery entry and build checks; no offline motion tests or real-robot execution. The stage checks above are things to observe while learning, not a substitute for completed motion verification.
