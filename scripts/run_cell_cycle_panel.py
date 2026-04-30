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

try:
    import tkinter as tk
    from tkinter import filedialog, messagebox, ttk
except Exception as exc:  # pragma: no cover
    raise SystemExit(f"FAIL: tkinter is required for this optional panel: {exc}")


DEFAULTS = {
    "scene_package": "ur5_2f_test",
    "task_recipe": "tests/fixtures/task_recipes/mvp1_generated_cell_colour_sorting.yaml",
    "detected_objects": "tests/fixtures/detected_objects/mvp1_colour_sorting_with_fallback.yaml",
    "epd_topic": "/easy_perception_deployment/epd_localize_output",
    "output_dir": "/tmp/mvp1",
    "capture_timeout": "10",
    "min_objects": "1",
}


@dataclass
class CycleReportSummary:
    status: str
    warning_count: int
    error_count: int
    report_path: Path | None
    generated_files: dict[str, Path]


def build_cycle_command(config: dict[str, Any]) -> list[str]:
    script = Path(__file__).resolve().parent / "run_generated_cell_cycle.py"
    cmd = [
        sys.executable,
        str(script),
        "--scene-package",
        config["scene_package"],
        "--task-recipe",
        config["task_recipe"],
        "--output-dir",
        config["output_dir"],
        "--min-objects",
        str(config["min_objects"]),
        "--capture-timeout",
        str(config["capture_timeout"]),
        "--once",
    ]
    if config.get("capture_live"):
        cmd += ["--capture-live", "--epd-topic", config["epd_topic"]]
    else:
        cmd += ["--detected-objects", config["detected_objects"]]
    if config.get("dry_run"):
        cmd.append("--dry-run")
    if config.get("replay"):
        cmd.append("--replay")
    if config.get("strict"):
        cmd.append("--strict")
    if config.get("json"):
        cmd.append("--json")
    return cmd


def parse_cycle_report(output_dir: Path) -> CycleReportSummary:
    report_path = output_dir / "cycle_report.json"
    generated = {
        "detected_objects_used.yaml": output_dir / "detected_objects_used.yaml",
        "runtime_execution_plan.json": output_dir / "runtime_execution_plan.json",
        "emd_grasp_bridge_payload.json": output_dir / "emd_grasp_bridge_payload.json",
        "cycle_report.json": report_path,
    }
    if not report_path.exists():
        return CycleReportSummary("FAIL", 0, 1, None, generated)
    data = json.loads(report_path.read_text(encoding="utf-8"))
    warnings = data.get("warnings") or []
    acceptance = data.get("acceptance") or {}
    errors = acceptance.get("errors") or []
    return CycleReportSummary(data.get("status", "FAIL"), len(warnings), len(errors), report_path, generated)


class CellCyclePanel:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("Run Cell Cycle")
        self.proc: subprocess.Popen[str] | None = None
        self.queue: queue.Queue[tuple[str, str]] = queue.Queue()

        self.scene_package = tk.StringVar(value=DEFAULTS["scene_package"])
        self.task_recipe = tk.StringVar(value=DEFAULTS["task_recipe"])
        self.detected_objects = tk.StringVar(value=DEFAULTS["detected_objects"])
        self.epd_topic = tk.StringVar(value=DEFAULTS["epd_topic"])
        self.output_dir = tk.StringVar(value=DEFAULTS["output_dir"])
        self.capture_timeout = tk.StringVar(value=DEFAULTS["capture_timeout"])
        self.min_objects = tk.StringVar(value=DEFAULTS["min_objects"])
        self.input_mode = tk.StringVar(value="offline")
        self.dry_run = tk.BooleanVar(value=True)
        self.replay = tk.BooleanVar(value=False)
        self.strict = tk.BooleanVar(value=False)
        self.json_output = tk.BooleanVar(value=True)
        self.status = tk.StringVar(value="Status: idle")

        self._build_ui()
        self._toggle_input_mode()
        self.root.after(100, self._poll_queue)

    def _build_ui(self) -> None:
        frm = ttk.Frame(self.root, padding=8)
        frm.grid(sticky="nsew")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        row = 0
        for label, var in [("Scene package", self.scene_package), ("Task recipe", self.task_recipe), ("Detected objects", self.detected_objects), ("EPD topic", self.epd_topic), ("Output dir", self.output_dir), ("Capture timeout", self.capture_timeout), ("Min objects", self.min_objects)]:
            ttk.Label(frm, text=label).grid(row=row, column=0, sticky="w")
            ttk.Entry(frm, textvariable=var, width=80).grid(row=row, column=1, sticky="ew", padx=4, pady=2)
            row += 1

        ttk.Label(frm, text="Input mode").grid(row=row, column=0, sticky="w")
        ttk.Radiobutton(frm, text="Offline detected_objects file", variable=self.input_mode, value="offline", command=self._toggle_input_mode).grid(row=row, column=1, sticky="w")
        row += 1
        ttk.Radiobutton(frm, text="Live EPD capture", variable=self.input_mode, value="live", command=self._toggle_input_mode).grid(row=row, column=1, sticky="w")
        row += 1

        for txt, var in [("Dry run", self.dry_run), ("Replay to runtime", self.replay), ("Strict mode", self.strict), ("JSON output", self.json_output)]:
            ttk.Checkbutton(frm, text=txt, variable=var).grid(row=row, column=1, sticky="w")
            row += 1

        self.run_btn = ttk.Button(frm, text="Run Cycle", command=self.run_cycle)
        self.run_btn.grid(row=row, column=0, sticky="ew", pady=6)
        ttk.Button(frm, text="Open Output Folder", command=self.open_output).grid(row=row, column=1, sticky="w")
        row += 1
        ttk.Button(frm, text="Open cycle_report.json", command=self.open_report).grid(row=row, column=1, sticky="w")
        ttk.Button(frm, text="Clear Log", command=lambda: self.log.delete("1.0", tk.END)).grid(row=row, column=0, sticky="ew")
        row += 1

        ttk.Label(frm, textvariable=self.status, font=("TkDefaultFont", 12, "bold")).grid(row=row, column=0, columnspan=2, sticky="w")
        row += 1
        ttk.Label(frm, text="Live mode prerequisites: (1) EPD / RealSense node running, (2) run_grasp_execution running if replay is selected.").grid(row=row, column=0, columnspan=2, sticky="w")
        row += 1

        self.log = tk.Text(frm, height=20, wrap="word")
        self.log.grid(row=row, column=0, columnspan=2, sticky="nsew")
        frm.columnconfigure(1, weight=1)
        frm.rowconfigure(row, weight=1)

    def _toggle_input_mode(self) -> None:
        live = self.input_mode.get() == "live"
        # keep simple: both editable, backend validates; show mode in status
        self.status.set(f"Status: idle ({'live EPD' if live else 'offline file'})")

    def _append(self, text: str) -> None:
        self.log.insert(tk.END, text)
        self.log.see(tk.END)

    def _poll_queue(self) -> None:
        try:
            while True:
                kind, payload = self.queue.get_nowait()
                if kind == "log":
                    self._append(payload)
                elif kind == "done":
                    self._on_done(int(payload))
        except queue.Empty:
            pass
        self.root.after(100, self._poll_queue)

    def _config(self) -> dict[str, Any]:
        return {
            "scene_package": self.scene_package.get().strip(),
            "task_recipe": self.task_recipe.get().strip(),
            "detected_objects": self.detected_objects.get().strip(),
            "capture_live": self.input_mode.get() == "live",
            "epd_topic": self.epd_topic.get().strip(),
            "output_dir": self.output_dir.get().strip(),
            "capture_timeout": self.capture_timeout.get().strip(),
            "min_objects": self.min_objects.get().strip(),
            "dry_run": self.dry_run.get(),
            "replay": self.replay.get(),
            "strict": self.strict.get(),
            "json": self.json_output.get(),
        }

    def run_cycle(self) -> None:
        if self.proc is not None:
            return
        if self.replay.get() and not messagebox.askyesno("Replay confirmation", "Replay will send the generated payload to the running grasp execution node. Continue?"):
            return
        cfg = self._config()
        cmd = build_cycle_command(cfg)
        if not Path(cmd[1]).exists():
            messagebox.showerror("Missing backend", f"Backend script not found: {cmd[1]}")
            return
        self.run_btn.configure(state="disabled")
        self.status.set("Status: running...")
        self._append("\n$ " + " ".join(cmd) + "\n")

        def worker() -> None:
            self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
            assert self.proc.stdout is not None
            for line in self.proc.stdout:
                self.queue.put(("log", line))
            rc = self.proc.wait()
            self.queue.put(("done", str(rc)))

        threading.Thread(target=worker, daemon=True).start()

    def _on_done(self, rc: int) -> None:
        self.proc = None
        self.run_btn.configure(state="normal")
        summary = parse_cycle_report(Path(self.output_dir.get().strip()))
        self._append(f"\nProcess exited with code {rc}\n")
        self._append(f"Summary: status={summary.status} warnings={summary.warning_count} errors={summary.error_count}\n")
        for name, path in summary.generated_files.items():
            self._append(f"- {name}: {'FOUND' if path.exists() else 'MISSING'} ({path})\n")
        self.status.set(f"Status: {summary.status} (warnings={summary.warning_count}, errors={summary.error_count})")
        if rc != 0:
            messagebox.showerror("Cycle failed", "Backend command exited non-zero. See log output for details.")

    def open_output(self) -> None:
        self._open_path(Path(self.output_dir.get().strip()))

    def open_report(self) -> None:
        self._open_path(Path(self.output_dir.get().strip()) / "cycle_report.json")

    def _open_path(self, path: Path) -> None:
        if not path.exists():
            messagebox.showwarning("Missing path", f"Path does not exist: {path}")
            return
        if sys.platform.startswith("linux"):
            subprocess.Popen(["xdg-open", str(path)])
        elif sys.platform == "darwin":
            subprocess.Popen(["open", str(path)])
        else:
            os.startfile(str(path))


def main() -> int:
    root = tk.Tk()
    CellCyclePanel(root)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
