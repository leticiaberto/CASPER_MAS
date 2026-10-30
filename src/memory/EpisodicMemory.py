import json
from pathlib import Path
from collections import defaultdict

'''
Every (update, value) is flattened into one "belief" entry:
{"interaction_id", "update_id", "timestamp", "partner_id", "event_type", "evidence_source", "task_id",
 "context", "skill", "skill_value", "preference", "trust", "outcome" (0/1/None), "is_evidence"}
 
is_evidence is False for partner_self_report (what the partner says about itself) and True for anything
the agent learned from a task (observation, outcome feedback, report). Trust and skill values are stored,
so the agent restores them instead of recomputing them.
'''

SELF_REPORT = "partner_self_report"
OUTCOME_VALUES = {"success": 1.0, "failure": 0.0}

def to_outcome(value):
    if isinstance(value, str):
        return OUTCOME_VALUES.get(value.strip().lower())
    try:
        return float(value)
    except (TypeError, ValueError):
        return None
 
def append_jsonl(path, record):
    """Append one JSON line. Creates the folder, and repairs a missing trailing newline first."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists() and path.stat().st_size > 0:
        with open(path, "rb") as f:
            f.seek(-1, 2)
            last = f.read(1)
        if last != b"\n":
            with open(path, "a", encoding="utf-8") as f:
                f.write("\n")
    with open(path, "a", encoding="utf-8") as f:
        f.write(json.dumps(record, ensure_ascii=False) + "\n")

class EpisodicMemory:
    def __init__(self, path="memory.jsonl"):
        self.path = Path(path)
        self.interactions = []                                  # raw lines, in order
        self.by_id = {}                                         # interaction_id -> raw interaction
        self.by_partner = defaultdict(list)                     # partner_id -> [entries], chronological
        self.by_pair = defaultdict(lambda: defaultdict(list))   # (skill, context) -> partner_id -> [entries]
        self._load()
 
    # --- loading ---
    def _load(self):
        if not self.path.exists():
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self.path.touch()
            return
        with open(self.path, encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    rec = json.loads(line)
                except json.JSONDecodeError:
                    continue    # skip a corrupted (half-written) line
                self._index(rec)
 
    def _flatten(self, rec):
        iid = rec.get("interaction_id")
        outcomes = {o.get("task_id"): to_outcome(o.get("known_outcome"))
                    for o in rec.get("observations", [])}
        out = []
        for partner_id, model in rec.get("partner_models", {}).items():
            for upd in model.get("updates", []):
                task_id = upd.get("task_id")
                for v in upd.get("values", []):
                    if v.get("skill") is None or v.get("context") is None:
                        continue
                    out.append({
                        "interaction_id": iid,
                        "update_id": upd.get("update_id"),
                        "timestamp": upd.get("timestamp") or rec.get("timestamp"),
                        "partner_id": partner_id,
                        "event_type": upd.get("event_type"),
                        "evidence_source": upd.get("evidence_source"),
                        "task_id": task_id,
                        "context": v["context"],
                        "skill": v["skill"],
                        "skill_value": v.get("skill_value"),
                        "preference": v.get("preference"),
                        "trust": v.get("trust"),
                        "outcome": outcomes.get(task_id) if task_id else None,
                        "is_evidence": upd.get("event_type") != SELF_REPORT,
                    })
        out.sort(key=lambda e: e["timestamp"] or "")    # stable: keeps file order on ties
        return out
 
    def _index(self, rec):
        self.interactions.append(rec)
        if rec.get("interaction_id") is not None:
            self.by_id[rec["interaction_id"]] = rec
        entries = self._flatten(rec)
        for e in entries:
            self.by_partner[e["partner_id"]].append(e)
            self.by_pair[(e["skill"], e["context"])][e["partner_id"]].append(e)
        return entries
 
    # --- write --- 
    def add(self, interaction):
        """Append one raw interaction line. Returns its new belief entries."""
        append_jsonl(self.path, interaction)
        return self._index(interaction)
 
    # --- read ---
    def get(self, interaction_id):
        return self.by_id.get(interaction_id)
 
    def with_partner(self, partner_id):
        return self.by_partner.get(partner_id, [])
 
    def for_pair(self, skill, context):
        """partner_id -> [entries] for one (skill, context)."""
        return self.by_pair.get((skill, context), {})
 
    def pair_summary(self, skill, context):
        """
        For borrowing: partner_id -> {"level", "trust", "count"} using the agent's latest belief
        about that partner in (skill, context). Partners with no real evidence (only self-report)
        are left out, there is nothing learned to borrow.
        """
        out = {}
        for pid, entries in self.for_pair(skill, context).items():
            count = sum(1 for e in entries if e["is_evidence"])
            last = entries[-1]
            if count == 0 or last["skill_value"] is None or last["trust"] is None:
                continue
            out[pid] = {"level": last["skill_value"], "trust": last["trust"], "count": count}
        return out
 
    def search(self, predicate):
        return [r for r in self.interactions if predicate(r)]
 
    def recent(self, n=10):
        return self.interactions[-n:]
 
    def print_all(self, pretty=True):
        if not self.interactions:
            print("Memory is empty.")
            return
        print(f"{len(self.interactions)} interactions in memory\n")
        for rec in self.interactions:
            if pretty:
                print(json.dumps(rec, indent=2, ensure_ascii=False))
                print("-" * 40)
            else:
                print(rec)