#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Mark generated workcell bundle approved")
    parser.add_argument("--workcell", required=True)
    parser.add_argument("--approved-by", required=True)
    parser.add_argument("--notes", default="")
    args = parser.parse_args(argv)

    summary_path = Path(args.workcell) / "generated" / "generated_workcell_summary.json"
    if not summary_path.exists():
        print(f"FAIL: missing summary: {summary_path}")
        return 1
    data = json.loads(summary_path.read_text(encoding="utf-8"))
    data["approval"] = {
        "status": "approved",
        "approved_by": args.approved_by,
        "approved_at": datetime.now(timezone.utc).isoformat(),
        "notes": args.notes,
    }
    summary_path.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    report_path = Path(args.workcell) / "generated" / "approval_report.json"
    report_path.write_text(json.dumps({"workcell": args.workcell, "approval": data["approval"]}, indent=2, sort_keys=True)+"\n", encoding="utf-8")
    print(f"PASS: updated {summary_path}")
    print(f"PASS: wrote {report_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
