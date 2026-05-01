#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

try:
    from scripts.studio_lite import StudioLiteApp
    from scripts.workcell_roles import ROLE_OPERATOR
except ModuleNotFoundError:
    from studio_lite import StudioLiteApp
    from workcell_roles import ROLE_OPERATOR


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Operator Runtime (no motion)")
    parser.add_argument("--workcell", required=True, help="Path to generated workcell bundle")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    workcell = Path(args.workcell)
    if not workcell.exists():
        print(f"FAIL: workcell path not found: {workcell}")
        return 1
    try:
        import tkinter as tk
        from tkinter import filedialog, ttk
    except Exception as exc:  # noqa: BLE001
        print(f"FAIL: tkinter/display unavailable: {exc}")
        return 1

    try:
        root = tk.Tk()
    except Exception as exc:  # noqa: BLE001
        print(f"FAIL: tkinter/display unavailable: {exc}")
        return 1
    root.title("Operator Runtime")
    app = StudioLiteApp(root, tk, ttk, filedialog, role=ROLE_OPERATOR)
    app.bundle_path.set(str(workcell))
    app._load_bundle()
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
