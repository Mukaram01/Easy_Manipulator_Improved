#!/usr/bin/env bash
set -euo pipefail

APT_GET="sudo apt-get"
command -v sudo >/dev/null 2>&1 || APT_GET="apt-get"

REQUIRED_PACKAGES=(
  libtinyxml2-dev
  libboost-dev
  libboost-graph-dev
  libboost-program-options-dev
  libboost-serialization-dev
)

missing=()
for pkg in "${REQUIRED_PACKAGES[@]}"; do
  if ! dpkg -s "$pkg" >/dev/null 2>&1; then
    missing+=("$pkg")
  fi
done

if [[ ${#missing[@]} -eq 0 ]]; then
  echo "All required system dependencies are already installed."
  exit 0
fi

echo "Installing missing system dependencies: ${missing[*]}"
$APT_GET update -y
$APT_GET install -y "${missing[@]}"
