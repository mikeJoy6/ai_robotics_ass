"""Planner contract: no ROS node, publisher, or execution ownership."""
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
import time
from typing import Any, Callable, Iterable, Mapping

import numpy as np


@dataclass(frozen=True)
class PlanningRequest:
    start: np.ndarray
    goal_position: np.ndarray
    goal_rotation: np.ndarray
    kin: Any
    deadline: float
    check_state: Callable[[np.ndarray], bool]
    log: Callable[[str], None]
    options: Mapping[str, Any]
    orientation_mode: str = field(default='full', kw_only=True)

    def __post_init__(self):
        if self.orientation_mode != 'full':
            raise ValueError(f'Unknown orientation constraint mode: {self.orientation_mode}')

    def check_deadline(self):
        if time.monotonic() >= self.deadline:
            raise ValueError('Planning compute budget exhausted')


@dataclass(frozen=True)
class JointReplayRequest(PlanningRequest):
    """Explicit joint-replay baseline only; never sent to IK algorithms."""
    goal_joints: np.ndarray


@dataclass(frozen=True)
class CandidatePath:
    label: str
    # Rows are joint waypoints, including the actual start and goal.
    waypoints: np.ndarray


class Algorithm(ABC):
    requires_recorded_joints = False
    orientation_mode = 'full'
    # Optional scoring semantics, independent of endpoint orientation acceptance.
    cost_mode = None

    @abstractmethod
    def plan(self, request: PlanningRequest) -> Iterable[CandidatePath]:
        """Return candidate geometric paths; never send robot commands."""
