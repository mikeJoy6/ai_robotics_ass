"""Existing exact recorded-joint replay baseline."""
import numpy as np

from . import Algorithm, CandidatePath, register


@register('joint', 'Known target joint replay baseline, for comparison only, not an IK test')
class JointDirect(Algorithm):
    requires_recorded_joints = True

    def plan(self, request):
        if request.options:
            raise ValueError('joint algorithm has no specific parameters')
        request.check_deadline()
        return [CandidatePath('recorded', np.array([request.start, request.goal_joints]))]
