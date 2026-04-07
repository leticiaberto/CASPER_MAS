"""
Shared data contract between TiagoPickPlacePlanner and TiagoAdapter.

Extends the basic MOVE/gripper steps with a NAVIGATE step for the mobile base,
which is executed via the Nav2 NavigateToPose action.

A PickPlanResult contains an ordered list of PickPlaceStep objects.
Each step is one atomic action: navigate base, move arm, open/close gripper.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List, Optional, Tuple

from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory

# (x, y, theta) in the world/map frame
NavGoal = Tuple[float, float, float]


class StepKind(Enum):
    MOVE          = auto()   # Move the arm along a planned trajectory
    OPEN_GRIPPER  = auto()   # Open the PAL gripper fully
    CLOSE_GRIPPER = auto()   # Close the PAL gripper (grasp)
    NAVIGATE      = auto()   # Drive the mobile base via Nav2 NavigateToPose


@dataclass
class PickPlaceStep:
    """
    One atomic step in a pick-and-place plan.

    Fields
    ------
    kind : StepKind
        What this step does.
    label : str
        Human-readable name for logging (e.g. "approach_pick").

    -- MOVE steps --
    target_pose : Optional[Pose]
        The Cartesian goal pose in the arm-base frame.
        Stored for reference / future real-robot use.
    joint_trajectory : Optional[JointTrajectory]
        Pre-computed joint trajectory from IK (for simulation).

    -- Gripper steps --
    gripper_width : Optional[float]
        Target per-finger width in metres (informational).

    -- NAVIGATE steps --
    nav_goal : Optional[NavGoal]
        (x, y, theta) in the world/map frame to send to Nav2.
    """
    kind:              StepKind
    label:             str
    target_pose:       Optional[Pose]            = None
    joint_trajectory:  Optional[JointTrajectory] = None
    gripper_width:     Optional[float]           = None
    nav_goal:          Optional[NavGoal]         = None


@dataclass
class PickPlanResult:
    """
    The complete plan for a pick-and-place task.

    Produced by TiagoPickPlacePlanner, consumed by TiagoAdapter.
    Adapters iterate over `steps` in order and execute each one.
    """
    steps:   List[PickPlaceStep] = field(default_factory=list)
    success: bool                = True
    message: str                 = "Plan ready."

    @classmethod
    def failure(cls, reason: str) -> "PickPlanResult":
        """Convenience constructor for a failed plan."""
        return cls(steps=[], success=False, message=reason)
