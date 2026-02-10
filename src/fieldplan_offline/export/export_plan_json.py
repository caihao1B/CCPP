from __future__ import annotations
import json
from pathlib import Path
from fieldplan_offline.export.plan_schema import Plan

def save_plan(plan: Plan, out_path: str | Path) -> Path:
    p = Path(out_path)
    p.write_text(json.dumps(plan.to_dict(), ensure_ascii=False, indent=2), encoding="utf-8")
    return p
