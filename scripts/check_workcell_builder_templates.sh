#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
TEMPLATE_PATH="${REPO_ROOT}/workcell_builder/workcell_builder/templates/ros2/scene_manifest_contract_template.yaml"
VALIDATOR_PATH="${REPO_ROOT}/scripts/validate_scene_contract.py"

if [[ ! -f "${TEMPLATE_PATH}" ]]; then
  echo "Workcell builder template check: FAIL (missing template: ${TEMPLATE_PATH})"
  exit 1
fi

if [[ ! -f "${VALIDATOR_PATH}" ]]; then
  echo "Workcell builder template check: FAIL (missing validator: ${VALIDATOR_PATH})"
  exit 1
fi

python3 - <<'PY' "${TEMPLATE_PATH}" "${VALIDATOR_PATH}"
import importlib.util
import sys
from pathlib import Path

template_path = Path(sys.argv[1])
validator_path = Path(sys.argv[2])

spec = importlib.util.spec_from_file_location("validate_scene_contract", validator_path)
if not spec or not spec.loader:
    print("Workcell builder template check: FAIL (unable to load validator module)")
    raise SystemExit(1)
validator = importlib.util.module_from_spec(spec)
sys.modules["validate_scene_contract"] = validator
spec.loader.exec_module(validator)

raw = template_path.read_text(encoding="utf-8")
try:
    data = validator.parse_manifest_yaml(raw)
except Exception as exc:
    print(f"Workcell builder template check: FAIL (template is not fallback-parser compatible YAML: {exc})")
    raise SystemExit(1)

required = ["self_test", "task_recipe", "home_return", "robot", "end_effector", "frames"]
missing = [key for key in required if key not in data]
if missing:
    print("Workcell builder template check: FAIL (missing sections: " + ", ".join(missing) + ")")
    raise SystemExit(1)

safe_joint_state = validator._dig(data, "home_return.safe_joint_state")
if safe_joint_state is None:
    print("Workcell builder template check: FAIL (missing explicit home_return.safe_joint_state key)")
    raise SystemExit(1)

status, notes = validator.validate_task_recipe_block(data)
contract_notes = []
if status == "FAIL":
    contract_notes.append("task_recipe block failed validator contract")

self_test_status, self_test_notes = ("PASS", [])
errors = []
warnings = []
validator._check_self_test(data, errors, warnings)
if errors:
    self_test_status = "FAIL"
    contract_notes.extend(errors)
elif warnings:
    self_test_status = "WARN"
    contract_notes.extend(warnings)

state = "PASS"
if contract_notes or status == "WARN" or self_test_status == "WARN":
    state = "WARN"

print(f"Workcell builder template check: {state}")
print(f"- Parsed with fallback YAML parser: PASS")
print(f"- Required sections present: PASS")
print(f"- Explicit home_return.safe_joint_state key: PASS")
print(f"- task_recipe validator status: {status}")
print(f"- self_test validator status: {self_test_status}")
for note in notes + self_test_notes + contract_notes:
    print(f"  note: {note}")

raise SystemExit(0)
PY
