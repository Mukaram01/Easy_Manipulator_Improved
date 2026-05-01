#!/usr/bin/env python3
from __future__ import annotations

import json
import os
import queue
import subprocess
import sys
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Any

NO_MOTION_BANNER = "NO ROBOT MOTION - Studio Lite is offline preflight only"
SAFE_FOR_ROBOT_MOTION_TEXT = "safe_for_robot_motion: false"
STATUS_NOT_RUN = "NOT RUN"
STATUS_PASS = "PASS"
STATUS_WARN = "WARN"
STATUS_FAIL = "FAIL"


def build_validate_cell_definition_command(cell_definition: str) -> list[str]:
    script = Path(__file__).resolve().parent / "validate_cell_definition.py"
    return [sys.executable, str(script), "--cell-definition", cell_definition, "--json"]


def build_generate_workcell_command(cell_definition: str, output_dir: str, package_name: str) -> list[str]:
    script = Path(__file__).resolve().parent / "generate_workcell_from_cell_definition.py"
    return [
        sys.executable,
        str(script),
        "--cell-definition",
        cell_definition,
        "--output-dir",
        output_dir,
        "--package-name",
        package_name,
        "--json",
    ]


def build_preview_workcell_command(workcell_path: str) -> list[str]:
    script = Path(__file__).resolve().parent / "preview_generated_workcell_bundle.py"
    return [sys.executable, str(script), "--workcell", workcell_path, "--json"]


def build_publish_preview_markers_command(workcell_path: str) -> list[str]:
    cmd = build_preview_workcell_command(workcell_path)
    cmd.append("--publish-markers")
    return cmd


def build_run_gated_bundle_command(workcell_path: str, output_dir: str | None = None, with_task_flow: bool = False) -> list[str]:
    script = Path(__file__).resolve().parent / "run_generated_workcell_bundle.py"
    cmd = [
        sys.executable,
        str(script),
        "--workcell",
        workcell_path,
        "--gated-dry-run",
        "--dry-run",
        "--no-replay",
        "--json",
    ]
    if with_task_flow:
        cmd.append("--preview-task-flow")
    if output_dir:
        cmd += ["--output-dir", output_dir]
    return cmd


def build_preview_task_flow_command(workcell_path: str, task_flow_preview_path: str) -> list[str]:
    script = Path(__file__).resolve().parent / "preview_generated_workcell_bundle.py"
    return [
        sys.executable,
        str(script),
        "--workcell",
        workcell_path,
        "--show-task-flow",
        "--task-flow-preview",
        task_flow_preview_path,
        "--json",
    ]


@dataclass
class BundlePaths:
    root: Path
    summary: Path
    environment: Path
    destinations: Path
    detected_example: Path
    visual_preview_summary: Path
    task_flow_preview: Path
    gated_report: Path


def _bundle_paths(workcell_path: str) -> BundlePaths:
    root = Path(workcell_path)
    gen = root / "generated"
    return BundlePaths(
        root=root,
        summary=gen / "generated_workcell_summary.json",
        environment=gen / "generated_environment_objects.yaml",
        destinations=gen / "generated_destinations.yaml",
        detected_example=gen / "generated_detected_objects_example.yaml",
        visual_preview_summary=gen / "visual_preview_summary.json",
        task_flow_preview=gen / "bundle_run" / "task_flow_preview.json",
        gated_report=gen / "bundle_run" / "bundle_run_report.json",
    )


def _open_or_print_path(path: Path) -> str:
    if not path.exists():
        return f"Path not found: {path}"
    try:
        if sys.platform.startswith("linux"):
            subprocess.Popen(["xdg-open", str(path)])
        elif sys.platform == "darwin":
            subprocess.Popen(["open", str(path)])
        elif os.name == "nt":
            os.startfile(str(path))  # type: ignore[attr-defined]
        return f"Opened: {path}"
    except Exception:
        return f"Open manually: {path}"


class StudioLiteApp:
    def __init__(self, root: Any, tk: Any, ttk: Any, filedialog: Any) -> None:
        self.root = root
        self.tk = tk
        self.ttk = ttk
        self.filedialog = filedialog
        self.root.title("Studio Lite")
        self.queue: queue.Queue[tuple[str, str]] = queue.Queue()

        self.cell_definition = tk.StringVar(value="cell_definitions/demo_ur5_sorting_cell.yaml")
        self.output_dir = tk.StringVar(value="/tmp/generated_workcells")
        self.package_name = tk.StringVar(value="demo_ur5_sorting_cell")
        self.bundle_path = tk.StringVar(value="")
        self.status = tk.StringVar(value=STATUS_NOT_RUN)
        self.last_command = tk.StringVar(value="")
        self.last_exit = tk.StringVar(value="")

        self._build_ui()
        self.root.after(100, self._poll)

    def _build_ui(self) -> None:
        frm = self.ttk.Frame(self.root, padding=8)
        frm.grid(sticky="nsew")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        frm.columnconfigure(1, weight=1)

        r = 0
        self.ttk.Label(frm, text=NO_MOTION_BANNER, foreground="red").grid(row=r, column=0, columnspan=4, sticky="w")
        r += 1
        self.ttk.Label(frm, text="Status").grid(row=r, column=0, sticky="w")
        self.ttk.Label(frm, textvariable=self.status).grid(row=r, column=1, sticky="w")
        self.ttk.Label(frm, text=SAFE_FOR_ROBOT_MOTION_TEXT).grid(row=r, column=2, columnspan=2, sticky="w")
        r += 1

        self.ttk.Label(frm, text="Cell definition").grid(row=r, column=0, sticky="w")
        self.ttk.Entry(frm, textvariable=self.cell_definition, width=80).grid(row=r, column=1, sticky="ew")
        self.ttk.Button(frm, text="Browse", command=self._browse_cell).grid(row=r, column=2, sticky="w")
        r += 1
        self.ttk.Label(frm, text="Output dir").grid(row=r, column=0, sticky="w")
        self.ttk.Entry(frm, textvariable=self.output_dir, width=80).grid(row=r, column=1, sticky="ew")
        r += 1
        self.ttk.Label(frm, text="Package name").grid(row=r, column=0, sticky="w")
        self.ttk.Entry(frm, textvariable=self.package_name, width=80).grid(row=r, column=1, sticky="ew")
        r += 1
        self.ttk.Button(frm, text="Validate Cell Definition", command=lambda: self._run(build_validate_cell_definition_command(self.cell_definition.get()))).grid(row=r, column=0, sticky="w")
        self.ttk.Button(frm, text="Generate Workcell Bundle", command=self._generate_bundle).grid(row=r, column=1, sticky="w")
        r += 1

        self.ttk.Label(frm, text="Bundle path").grid(row=r, column=0, sticky="w")
        self.ttk.Entry(frm, textvariable=self.bundle_path, width=80).grid(row=r, column=1, sticky="ew")
        self.ttk.Button(frm, text="Load Bundle", command=self._load_bundle).grid(row=r, column=2, sticky="w")
        self.ttk.Button(frm, text="Show Bundle Summary", command=lambda: self._report(_bundle_paths(self.bundle_path.get()).summary)).grid(row=r, column=3, sticky="w")
        r += 1

        self.ttk.Button(frm, text="Generate JSON Preview", command=lambda: self._run(build_preview_workcell_command(self.bundle_path.get()))).grid(row=r, column=0, sticky="w")
        self.ttk.Button(frm, text="Publish RViz Markers", command=lambda: self._run(build_publish_preview_markers_command(self.bundle_path.get()))).grid(row=r, column=1, sticky="w")
        self.ttk.Button(frm, text="Open RViz Preview Command", command=self._show_rviz_cmd).grid(row=r, column=2, sticky="w")
        r += 1

        self.ttk.Button(frm, text="Run Gated Dry-Run", command=lambda: self._run(build_run_gated_bundle_command(self.bundle_path.get()))).grid(row=r, column=0, sticky="w")
        self.ttk.Button(frm, text="Run Gated Dry-Run + Task Flow Preview", command=lambda: self._run(build_run_gated_bundle_command(self.bundle_path.get(), with_task_flow=True))).grid(row=r, column=1, columnspan=2, sticky="w")
        r += 1

        self.ttk.Button(frm, text="Open bundle summary path", command=lambda: self._report(_bundle_paths(self.bundle_path.get()).summary)).grid(row=r, column=0, sticky="w")
        self.ttk.Button(frm, text="Open visual preview summary path", command=lambda: self._report(_bundle_paths(self.bundle_path.get()).visual_preview_summary)).grid(row=r, column=1, sticky="w")
        self.ttk.Button(frm, text="Open task flow preview path", command=lambda: self._report(_bundle_paths(self.bundle_path.get()).task_flow_preview)).grid(row=r, column=2, sticky="w")
        self.ttk.Button(frm, text="Open gated dry-run report path", command=lambda: self._report(_bundle_paths(self.bundle_path.get()).gated_report)).grid(row=r, column=3, sticky="w")
        r += 1

        self.ttk.Label(frm, text="Last command").grid(row=r, column=0, sticky="w")
        self.ttk.Label(frm, textvariable=self.last_command).grid(row=r, column=1, columnspan=3, sticky="w")
        r += 1
        self.ttk.Label(frm, text="Last exit").grid(row=r, column=0, sticky="w")
        self.ttk.Label(frm, textvariable=self.last_exit).grid(row=r, column=1, sticky="w")
        r += 1
        self.log = self.tk.Text(frm, height=18, wrap="word")
        self.log.grid(row=r, column=0, columnspan=4, sticky="nsew")
        frm.rowconfigure(r, weight=1)

    def _browse_cell(self) -> None:
        p = self.filedialog.askopenfilename(title="Select cell definition")
        if p:
            self.cell_definition.set(p)

    def _generate_bundle(self) -> None:
        self._run(build_generate_workcell_command(self.cell_definition.get(), self.output_dir.get(), self.package_name.get()))
        self.bundle_path.set(str(Path(self.output_dir.get()) / self.package_name.get()))

    def _load_bundle(self) -> None:
        p = _bundle_paths(self.bundle_path.get())
        required = [p.summary, p.environment, p.destinations]
        missing = [str(x) for x in required if not x.exists()]
        self.log.insert("end", f"Loaded bundle: {p.root}\n")
        if p.detected_example.exists():
            self.log.insert("end", f"Detected object examples: {p.detected_example}\n")
        if missing:
            self.status.set(STATUS_FAIL)
            self.log.insert("end", "Missing required files:\n" + "\n".join(missing) + "\n")
        else:
            self.status.set(STATUS_PASS)

    def _show_rviz_cmd(self) -> None:
        cfg = Path(__file__).resolve().parents[1] / "rviz" / "generated_workcell_preview.rviz"
        cmd = f"rviz2 -d {cfg}" if cfg.exists() else "rviz2 (default config)"
        self.log.insert("end", f"RViz command: {cmd}\n")

    def _report(self, path: Path) -> None:
        self.log.insert("end", _open_or_print_path(path) + "\n")

    def _run(self, cmd: list[str]) -> None:
        self.last_command.set(" ".join(cmd))
        t = threading.Thread(target=self._worker, args=(cmd,), daemon=True)
        t.start()

    def _worker(self, cmd: list[str]) -> None:
        p = subprocess.run(cmd, capture_output=True, text=True, check=False)
        out = (p.stdout or "") + ("\n" + p.stderr if p.stderr else "")
        self.queue.put((str(p.returncode), out))

    def _poll(self) -> None:
        while not self.queue.empty():
            code, out = self.queue.get()
            self.last_exit.set(code)
            self.log.insert("end", out + "\n")
            self.status.set(STATUS_PASS if code == "0" else STATUS_FAIL)
            self._apply_dry_run_status(out)
        self.root.after(100, self._poll)

    def _apply_dry_run_status(self, out: str) -> None:
        try:
            data = json.loads(out)
            st = data.get("status", STATUS_NOT_RUN)
            self.status.set(st)
            self.log.insert("end", f"blockers: {data.get('blockers', [])}\n")
            self.log.insert("end", f"warnings: {data.get('warnings', [])}\n")
            self.log.insert("end", f"selected_object: {data.get('selected_object')}\n")
            self.log.insert("end", f"selected_destination: {data.get('selected_destination')}\n")
            self.log.insert("end", SAFE_FOR_ROBOT_MOTION_TEXT + "\n")
        except Exception:
            pass


def main() -> int:
    try:
        import tkinter as tk
        from tkinter import filedialog, ttk
    except Exception as exc:
        print(f"FAIL: tkinter unavailable: {exc}")
        return 1
    root = tk.Tk()
    StudioLiteApp(root, tk, ttk, filedialog)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
