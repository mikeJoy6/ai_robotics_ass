"""Add a module with @register; no host/central algorithm list edit required."""
from ._registry import discover, register
from ._types import Algorithm, CandidatePath, PlanningRequest, JointReplayRequest

__all__ = ['Algorithm', 'CandidatePath', 'PlanningRequest', 'JointReplayRequest', 'discover', 'register']
