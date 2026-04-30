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



SCRIPTS_DIR = Path(__file__).resolve().parent
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))
from workcell_discovery import discover_all

DEFAULTS = {
    "scene_package": "ur5_2f_test",
    "task_recipe": "tests/fixtures/task_recipes/valid_garbage_sorting.yaml",
    "detected_objects": "tests/fixtures/detected_objects/valid_epd_garbage_sorting.yaml",
    "epd_topic": "/easy_perception_deployment/epd_localize_output",
    "output_dir": "/tmp/mvp1_live_smoke_test",
    "capture_timeout": "10",
    "min_objects": "1",
    "frame_fallback": "world",
}


@dataclass
class CycleReportSummary:
    status: str
    warning_count: int
    error_count: int
    report_path: Path | None
    generated_files: dict[str, Path]
    perception_source: str | None = None
    detected_objects_used: str | None = None


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
        "--frame-fallback",
        str(config["frame_fallback"]),
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
    else:
        cmd.append("--no-replay")
    if config.get("strict"):
        cmd.append("--strict")
    if config.get("json"):
        cmd.append("--json")
    return cmd





def build_gated_cycle_command(config: dict[str, Any]) -> list[str]:
    cmd = build_cycle_command(config)
    cmd += ['--require-preflight']
    if config.get('capture_live'):
        cmd += ['--preflight-live', '--preflight-check-ros-topics', '--preflight-check-tf', '--preflight-target-frame', 'world', '--preflight-camera-frame', 'camera_depth_optical_frame']
        if '--epd-qos-reliability' not in cmd:
            cmd += ['--epd-qos-reliability', 'best_effort']
    return cmd


def build_preflight_command(config: dict[str, Any]) -> list[str]:
    script = Path(__file__).resolve().parent / "run_cell_readiness_check.py"
    cmd = [
        sys.executable, str(script),
        "--scene-package", config["scene_package"],
        "--task-recipe", config["task_recipe"],
        "--epd-topic", config["epd_topic"],
        "--target-frame", config.get("target_frame", "world"),
        "--camera-frame", config.get("camera_frame", "camera_depth_optical_frame"),
        "--check-ros-topics",
        "--json",
    ]
    if config.get("detected_objects"):
        cmd += ["--detected-objects", config["detected_objects"]]
    if config.get("capture_live"):
        cmd += ["--live", "--check-tf"]
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
    return CycleReportSummary(data.get("status", "FAIL"), len(warnings), len(errors), report_path, generated, data.get("perception_source"), data.get("detected_objects_used"))


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
        self.frame_fallback = tk.StringVar(value=DEFAULTS["frame_fallback"])
        self.input_mode = tk.StringVar(value="offline")
        self.dry_run = tk.BooleanVar(value=True)
        self.replay = tk.BooleanVar(value=False)
        self.strict = tk.BooleanVar(value=False)
        self.json_output = tk.BooleanVar(value=True)
        self.show_dev_fixtures = tk.BooleanVar(value=False)
        self.status = tk.StringVar(value="Status: idle")
        self.preflight_btn: ttk.Button | None = None

        self.discovery_data = {"scenes": [], "task_recipes": [], "detected_objects": []}
        self._build_ui()
        self.refresh_discovery()
        self._toggle_input_mode()
        self.root.after(100, self._poll_queue)

    def _build_ui(self) -> None:
        frm = ttk.Frame(self.root, padding=8)
        frm.grid(sticky="nsew")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        row = 0
        ttk.Label(frm, text="Scene package").grid(row=row, column=0, sticky="w")
        self.scene_combo = ttk.Combobox(frm, textvariable=self.scene_package, width=77)
        self.scene_combo.grid(row=row, column=1, sticky="ew", padx=4, pady=2)
        self.scene_combo.bind("<<ComboboxSelected>>", lambda _e: self._update_selection_details())
        row += 1

        ttk.Label(frm, text="Task recipe").grid(row=row, column=0, sticky="w")
        self.task_combo = ttk.Combobox(frm, textvariable=self.task_recipe, width=77)
        self.task_combo.grid(row=row, column=1, sticky="ew", padx=4, pady=2)
        ttk.Button(frm, text="Browse...", command=self._browse_task_recipe).grid(row=row, column=2, sticky="w")
        self.task_combo.bind("<<ComboboxSelected>>", lambda _e: self._update_selection_details())
        row += 1

        ttk.Label(frm, text="Detected objects").grid(row=row, column=0, sticky="w")
        self.objects_combo = ttk.Combobox(frm, textvariable=self.detected_objects, width=77)
        self.objects_combo.grid(row=row, column=1, sticky="ew", padx=4, pady=2)
        ttk.Button(frm, text="Browse...", command=self._browse_detected_objects).grid(row=row, column=2, sticky="w")
        self.objects_combo.bind("<<ComboboxSelected>>", lambda _e: self._update_selection_details())
        row += 1

        for label, var in [("EPD topic", self.epd_topic), ("Output dir", self.output_dir), ("Capture timeout", self.capture_timeout), ("Min objects", self.min_objects), ("Frame fallback", self.frame_fallback)]:
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
        ttk.Checkbutton(frm, text="Show developer/test fixtures", variable=self.show_dev_fixtures, command=self.refresh_discovery).grid(row=row, column=1, sticky="w")
        row += 1

        self.run_btn = ttk.Button(frm, text="Run generated cycle", command=self.run_cycle)
        self.gated_btn = ttk.Button(frm, text="Run Gated Dry-Run", command=self.run_gated_cycle)
        self.preflight_btn = ttk.Button(frm, text="Run Preflight Check", command=self.run_preflight)
        ttk.Button(frm, text="Refresh discovery", command=self.refresh_discovery).grid(row=row, column=2, sticky="ew", pady=6)
        self.run_btn.grid(row=row, column=0, sticky="ew", pady=6)
        self.gated_btn.grid(row=row, column=0, sticky="e", pady=6)
        self.preflight_btn.grid(row=row, column=1, sticky="w", padx=4)
        ttk.Button(frm, text="Capture live snapshot only", command=self.capture_only).grid(row=row, column=1, sticky="w")
        row += 1
        ttk.Button(frm, text="Open Output Folder", command=self.open_output).grid(row=row, column=1, sticky="w")
        row += 1
        ttk.Button(frm, text="Open cycle_report.json", command=self.open_report).grid(row=row, column=1, sticky="w")
        ttk.Button(frm, text="Clear Log", command=lambda: self.log.delete("1.0", tk.END)).grid(row=row, column=0, sticky="ew")
        row += 1

        ttk.Label(frm, textvariable=self.status, font=("TkDefaultFont", 12, "bold")).grid(row=row, column=0, columnspan=2, sticky="w")
        row += 1
        ttk.Label(frm, text="Live mode prerequisites: (1) EPD / RealSense node running, (2) run_grasp_execution running if replay is selected.").grid(row=row, column=0, columnspan=3, sticky="w")
        row += 1
        self.selection_details = tk.Text(frm, height=5, wrap="word")
        self.selection_details.grid(row=row, column=0, columnspan=3, sticky="ew")
        self.selection_details.insert(tk.END, "Selection details will appear here.\n")
        self.selection_details.configure(state="disabled")
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
            "frame_fallback": self.frame_fallback.get().strip(),
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


    def run_gated_cycle(self) -> None:
        if self.proc is not None:
            return
        cfg = self._config()
        cfg['dry_run'] = True
        cfg['replay'] = False
        cmd = build_gated_cycle_command(cfg)
        self._append("\n$ " + " ".join(cmd) + "\n")
        proc = subprocess.run(cmd, capture_output=True, text=True, check=False)
        output = (proc.stdout or "") + (proc.stderr or "")
        self._append(output)
        payload = {}
        try:
            payload = json.loads(proc.stdout) if proc.stdout.strip().startswith('{') else {}
        except Exception:
            payload = {}
        pre = payload.get('preflight', {}) if isinstance(payload, dict) else {}
        cycle = payload.get('cycle', {}) if isinstance(payload, dict) else {}
        self._append(f"Preflight status: {pre.get('status', 'UNKNOWN')}\n")
        self._append("Blockers: " + json.dumps(pre.get('blockers', [])) + "\n")
        self._append("Warnings: " + json.dumps(pre.get('warnings', [])) + "\n")
        self._append(f"Selected object: {cycle.get('selected_object')}\n")
        self._append(f"Selected destination: {cycle.get('chosen_destination')}\n")
        self._append(f"Final status: {payload.get('operator_summary', {}).get('status', cycle.get('status', 'FAIL'))}\n")
        self._append(f"Report path: {Path(cfg['output_dir']) / 'cell_cycle_gated_report.json'}\n")
        self.status.set(f"Status: {payload.get('operator_summary', {}).get('status', 'FAIL')} (gated dry-run)")


    def run_preflight(self) -> None:
        cfg = self._config()
        cmd = build_preflight_command(cfg)
        self._append("\n$ " + " ".join(cmd) + "\n")
        proc = subprocess.run(cmd, capture_output=True, text=True, check=False)
        output = (proc.stdout or "") + (proc.stderr or "")
        self._append(output)
        try:
            payload = json.loads(proc.stdout) if proc.stdout.strip().startswith("{") else {}
            status = payload.get("status", "FAIL")
            blockers = payload.get("blockers") or []
            warns = payload.get("warnings") or []
            if blockers:
                self._append("Blockers:\n" + "\n".join(f"- {b}" for b in blockers) + "\n")
            if warns:
                self._append("Warnings:\n" + "\n".join(f"- {w}" for w in warns) + "\n")
            self.status.set(f"Status: {status} (preflight)")
        except Exception:
            self.status.set("Status: FAIL (preflight parse)")

    def capture_only(self) -> None:
        out = Path(self.output_dir.get().strip()) / "detected_objects_live.yaml"
        cmd = [
            sys.executable,
            str(Path(__file__).resolve().parent / "capture_epd_detected_objects.py"),
            "--scene-package", self.scene_package.get().strip(),
            "--topic", self.epd_topic.get().strip(),
            "--output", str(out),
            "--timeout", self.capture_timeout.get().strip(),
            "--min-objects", self.min_objects.get().strip(),
            "--frame-fallback", self.frame_fallback.get().strip(),
            "--once",
            "--json",
        ]
        self._append("\n$ " + " ".join(cmd) + "\n")
        proc = subprocess.run(cmd, capture_output=True, text=True, check=False)
        self._append((proc.stdout or "") + (proc.stderr or ""))
        self.status.set("Status: PASS (capture-only)" if proc.returncode == 0 else "Status: FAIL (capture-only)")

    def _on_done(self, rc: int) -> None:
        self.proc = None
        self.run_btn.configure(state="normal")
        summary = parse_cycle_report(Path(self.output_dir.get().strip()))
        self._append(f"\nProcess exited with code {rc}\n")
        self._append(f"Summary: status={summary.status} warnings={summary.warning_count} errors={summary.error_count}\n")
        if summary.perception_source:
            self._append(f"Perception source: {summary.perception_source}\n")
        if summary.detected_objects_used:
            self._append(f"Detected objects used: {summary.detected_objects_used}\n")
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


    def _browse_task_recipe(self) -> None:
        path = filedialog.askopenfilename(filetypes=[("YAML/JSON", "*.yaml *.yml *.json"), ("All", "*")])
        if path:
            self.task_recipe.set(path)
            self._update_selection_details()

    def _browse_detected_objects(self) -> None:
        path = filedialog.askopenfilename(filetypes=[("YAML/JSON", "*.yaml *.yml *.json"), ("All", "*")])
        if path:
            self.detected_objects.set(path)
            self._update_selection_details()

    def refresh_discovery(self) -> None:
        scene_sel = self.scene_package.get()
        task_sel = self.task_recipe.get()
        obj_sel = self.detected_objects.get()
        self.discovery_data = discover_all()
        scene_values = [x["package_name"] for x in self.discovery_data["scenes"]]
        show_dev = self.show_dev_fixtures.get()
        task_values = [x["path"] for x in self.discovery_data["task_recipes"] if show_dev or x.get("category") != "failure_test"]
        obj_values = [x["path"] for x in self.discovery_data["detected_objects"] if show_dev or x.get("category") != "failure_test"]
        self.scene_combo.configure(values=scene_values)
        self.task_combo.configure(values=task_values)
        self.objects_combo.configure(values=obj_values)
        if scene_sel in scene_values: self.scene_package.set(scene_sel)
        elif scene_values: self.scene_package.set(scene_values[0])
        if task_sel in task_values: self.task_recipe.set(task_sel)
        elif task_values:
            valid_default = next((p for p in task_values if p.endswith("valid_garbage_sorting.yaml")), task_values[0])
            self.task_recipe.set(valid_default)
        if obj_sel in obj_values: self.detected_objects.set(obj_sel)
        elif obj_values:
            valid_default = next((p for p in obj_values if p.endswith("valid_epd_garbage_sorting.yaml")), obj_values[0])
            self.detected_objects.set(valid_default)
        self._update_selection_details()

    def _update_selection_details(self) -> None:
        lines = []
        scene = next((x for x in self.discovery_data.get("scenes", []) if x.get("package_name") == self.scene_package.get().strip()), None)
        task = next((x for x in self.discovery_data.get("task_recipes", []) if x.get("path") == self.task_recipe.get().strip()), None)
        obj = next((x for x in self.discovery_data.get("detected_objects", []) if x.get("path") == self.detected_objects.get().strip()), None)
        lines.append(f"Scene: {self.scene_package.get().strip()}")
        if scene: lines.append(f"  source={scene.get('source_path')} installed={scene.get('installed')} generated={scene.get('generated')} warnings={scene.get('warnings')}")
        lines.append(f"Task: {self.task_recipe.get().strip()}")
        if task: lines.append(f"  version={task.get('version')} type={task.get('task_type')} category={task.get('category')} warnings={task.get('warnings')}")
        lines.append(f"Detected objects: {self.detected_objects.get().strip()}")
        if obj: lines.append(f"  version={obj.get('version')} object_count={obj.get('object_count')} category={obj.get('category')} warnings={obj.get('warnings')}")
        self.selection_details.configure(state='normal')
        self.selection_details.delete('1.0', tk.END)
        self.selection_details.insert(tk.END, "\n".join(lines) + "\n")
        self.selection_details.configure(state='disabled')

def main() -> int:
    root = tk.Tk()
    CellCyclePanel(root)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
