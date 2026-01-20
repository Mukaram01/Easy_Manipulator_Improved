#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

shopt -s nullglob

xacro_files=("${repo_root}"/scenes/*/urdf/scene.urdf.xacro)
if [ ${#xacro_files[@]} -eq 0 ]; then
  echo "No scene xacro files found under ${repo_root}/scenes"
else
  for xacro_file in "${xacro_files[@]}"; do
    echo "Validating xacro: ${xacro_file}"
    xacro --inorder "${xacro_file}" >/dev/null
  done
fi

mapfile -t launch_entries < <(
  python - <<'PY'
import pathlib
root = pathlib.Path(__file__).resolve().parents[1]
launch_globs = [
    'launch/*.launch.py',
    'launch/*.launch.xml',
    'launch/*.launch.yaml',
    'launch/*.launch.yml',
]
launch_files = []
for pattern in launch_globs:
    launch_files.extend(root.glob(f'**/{pattern}'))

entries = []
for launch_file in sorted(set(launch_files)):
    pkg_dir = next((p for p in [launch_file] + list(launch_file.parents) if (p / 'package.xml').exists()), None)
    if not pkg_dir:
        continue
    pkg_xml = (pkg_dir / 'package.xml').read_text()
    name_start = pkg_xml.find('<name>')
    name_end = pkg_xml.find('</name>')
    if name_start == -1 or name_end == -1:
        continue
    pkg_name = pkg_xml[name_start + 6:name_end].strip()
    entries.append((pkg_name, launch_file.name))

for pkg_name, launch_name in entries:
    print(f"{pkg_name}:{launch_name}")
PY
)

if [ ${#launch_entries[@]} -eq 0 ]; then
  echo "No launch files found for smoke testing."
else
  for entry in "${launch_entries[@]}"; do
    pkg="${entry%%:*}"
    launch_file="${entry#*:}"
    echo "Parsing launch file: ${pkg} ${launch_file}"
    ros2 launch "${pkg}" "${launch_file}" --show-args >/dev/null
  done
fi
