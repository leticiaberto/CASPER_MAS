from enum import Enum
from dataclasses import dataclass, field
from typing import List, Optional

class TaskStatus(Enum):
    PENDING = "pending"
    ASSIGNED = "assigned"
    RUNNING = "running"
    COMPLETED = "completed"

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
        self.status = TaskStatus.COMPLETED

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