"""
Shared data contract between the PickPlacePlanner and robot adapters.

This is the ONLY object that crosses the boundary between planning and execution.
Keeping it here (not inside the planner or adapter) means you can swap either
side without touching the other.

A PickPlanResult contains an ordered list of PickPlaceStep objects.
Each step is one atomic motion: move arm, open gripper, or close gripper.

When using Franky on the real robot, the adapter will read the Cartesian
poses from each MOVE step and build CartesianWaypointMotion objects directly
— no re-planning needed.

When using the simulation adapter, the adapter reads the pre-computed
JointTrajectory from each MOVE step and sends it via FollowJointTrajectory.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List, Optional

from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import Pose


class StepKind(Enum):
    """What this step does."""
    MOVE          = auto()   # Move the arm along a planned trajectory
    OPEN_GRIPPER  = auto()   # Open the gripper fully
    CLOSE_GRIPPER = auto()   # Close the gripper (grasp)


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
    
    -- Populated for MOVE steps only --
    target_pose : Optional[Pose]
        The Cartesian goal pose in the world frame.
        Used by the Franky adapter (real robot) to build CartesianWaypointMotion.
    joint_trajectory : Optional[JointTrajectory]
        Pre-computed joint trajectory from IK.
        Used by the simulation adapter (FollowJointTrajectory action).

    -- Populated for gripper steps only --
    gripper_width : Optional[float]
        Target gripper width in metres (informational — adapters may ignore).
    """
    kind:              StepKind
    label:             str
    target_pose:       Optional[Pose]            = None
    joint_trajectory:  Optional[JointTrajectory] = None
    gripper_width:     Optional[float]           = None


@dataclass
class PickPlanResult:
    """
    The complete plan for a pick-and-place task.

    Produced by PickPlacePlanner, consumed by robot adapters.
    Adapters iterate over `steps` in order and execute each one.
    """
    steps:   List[PickPlaceStep] = field(default_factory=list)
    success: bool                = True
    message: str                 = "Plan ready."

    @classmethod
    def failure(cls, reason: str) -> "PickPlanResult":
        """Convenience constructor for a failed plan."""
        return cls(steps=[], success=False, message=reason)
