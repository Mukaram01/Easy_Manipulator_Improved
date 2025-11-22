#!/usr/bin/env bash
set -euo pipefail

APT_GET="apt-get"
if command -v sudo >/dev/null 2>&1; then
  if sudo -n true 2>/dev/null; then
    APT_GET="sudo apt-get"
  elif [[ $EUID -ne 0 ]]; then
    echo "sudo is present but requires a password; falling back to apt-get" >&2
  fi
fi

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

# Boost headers are occasionally absent even when dpkg reports the packages as
# installed (for example, in stripped-down container images).  Fail fast by
# forcing a reinstall if the canonical graph header cannot be located.
BOOST_HEADER="/usr/include/boost/graph/adjacency_list.hpp"
if [[ ! -f $BOOST_HEADER ]]; then
  echo "Boost graph headers are missing; reinstalling Boost development packages" >&2
  for pkg in libboost-dev libboost-graph-dev libboost-program-options-dev libboost-serialization-dev; do
    if [[ ! " ${missing[*]} " =~ \ $pkg\  ]]; then
      missing+=("$pkg")
    fi
  done
fi

if [[ ${#missing[@]} -eq 0 ]]; then
  echo "All required system dependencies are already installed."
  exit 0
fi

echo "Installing missing system dependencies: ${missing[*]}"
$APT_GET update -y
$APT_GET install -y "${missing[@]}"
