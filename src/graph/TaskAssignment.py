from enum import Enum
from dataclasses import dataclass, field
from typing import List, Optional

class TaskStatus(Enum):
    PENDING = "pending"
    ASSIGNED = "assigned"
    READY = "ready"
    RUNNING = "running"
    DONE = "done"
    NOTASSIGNED = "notassigned"

@dataclass
class TaskAssignment:
    task_id: str
    selected_agent: Optional[object]
    selected_score: Optional[float]
    top_candidates: List[object] = field(default_factory=list)
    status: TaskStatus = TaskStatus.PENDING

    def start(self):
        if self.selected_agent:
            self.status = TaskStatus.RUNNING

    def complete(self):
        self.status = TaskStatus.DONE

    def assign(self):
        if self.selected_agent:
            self.status = TaskStatus.ASSIGNED

    def __repr__(self):
        return (
            f"TaskAssignment("
            f"selected={self.selected_agent if self.selected_agent else None}, "
            f"score={self.selected_score}, "
            f"status={self.status.value})"
        )
    
    def serialize(self):
        """
        dict for sending over the network
        """
        return {
            "task_id": self.task_id,
            "selected_agent": self.selected_agent,
            "selected_score": self.selected_score,
            "top_candidates": self.top_candidates,
            "status": self.status.name  # convert enum to string
        }
    
    @classmethod
    def deserialize(cls, data):
        """
        Reconstruct a TaskAssignment object from a serialized dict.
        """
        return cls(
            task_id=data["task_id"],
            selected_agent=data["selected_agent"],
            selected_score=data["selected_score"],
            top_candidates=data["top_candidates"],
            status=TaskStatus[data["status"]]  # string → enum
        )