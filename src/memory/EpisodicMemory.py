import json
from pathlib import Path
from collections import defaultdict
'''
Example usage:
{"id": "exp_0042", "timestamp": "2026-10-26T14:03:00Z", "context": "warehouse routing task", "agents": [{"name": "planner", "skills": ["decomposition", "scheduling"]}, {"name": "executor", "skills": ["navigation"]}], "goal_tree": {"goal": "deliver all packages", "subgoals": [{"goal": "plan routes", "subgoals": []}, {"goal": "execute routes", "subgoals": []}]}, "outcome": "success"}
'''

class EpisodicMemory:
    def __init__(self, path="memory.jsonl"):
        self.path = Path(path)
        self.records = []                    # all records, in order
        self.by_id = {}                      # id -> record
        self.by_agent = defaultdict(list)    # agent name -> [records]
        self.by_skill = defaultdict(list)    # skill -> [records]
        self._load()

    def _load(self):
        if not self.path.exists():
            self.path.touch()
            return
        with open(self.path, encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    self._index(json.loads(line))
                except json.JSONDecodeError:
                    continue  # skip a corrupted (e.g. half-written) line

    def _index(self, rec):
        self.records.append(rec)
        self.by_id[rec["id"]] = rec
        for agent in rec.get("agents", []):
            self.by_agent[agent["name"]].append(rec)
            for skill in agent.get("skills", []):
                self.by_skill[skill].append(rec)

    # --- write ---
    def add(self, rec):
        with open(self.path, "a", encoding="utf-8") as f:
            f.write(json.dumps(rec, ensure_ascii=False) + "\n")
        self._index(rec)  # keep in-memory view up to date

    # --- read ---
    def get(self, exp_id):
        return self.by_id.get(exp_id)

    def with_agent(self, name):
        return self.by_agent.get(name, [])

    def with_skill(self, skill):
        return self.by_skill.get(skill, [])

    def search(self, predicate):
        return [r for r in self.records if predicate(r)]

    def recent(self, n=10):
        return self.records[-n:]

    def print_all(self, pretty=True):
        if not self.records:
            print("Memory is empty.")
            return
        print(f"{len(self.records)} records in memory\n")
        for rec in self.records:
            if pretty:
                print(json.dumps(rec, indent=2, ensure_ascii=False))
                print("-" * 40)
            else:
                print(rec)