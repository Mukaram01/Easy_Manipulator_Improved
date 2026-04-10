#!/usr/bin/env bash
set -euo pipefail

# Build/package-relevant trees.
TARGET_DIRS=(
  assets
  workcell_builder
  easy_manipulation_deployment
  emd_msgs
  scenes
)

violations=0

for dir in "${TARGET_DIRS[@]}"; do
  [[ -d "$dir" ]] || continue
  while IFS= read -r path; do
    # Allow archived reference artifacts to use legacy naming.
    if [[ "$path" == docs/reference/* ]]; then
      continue
    fi

    base="$(basename "$path")"
    lower="$(printf '%s' "$base" | tr '[:upper:]' '[:lower:]')"

    if [[ "$base" == *" "* || "$lower" == *"(copy)"* ]]; then
      echo "Disallowed filename in build-relevant tree: $path"
      violations=1
    fi
  done < <(find "$dir" -type f)
done

if (( violations )); then
  echo "Found disallowed filenames (contains spaces or '(copy)')."
  exit 1
fi

echo "Filename hygiene check passed."
