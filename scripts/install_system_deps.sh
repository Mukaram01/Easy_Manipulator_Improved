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
  libboost-stacktrace-dev
  libboost-program-options-dev
  libboost-serialization-dev
  libcereal-dev
)

OSQP_NATIVE_PACKAGE=libosqp-dev
OSQP_PROVIDER=""

if apt-cache show "$OSQP_NATIVE_PACKAGE" >/dev/null 2>&1; then
  OSQP_PROVIDER="$OSQP_NATIVE_PACKAGE"
  REQUIRED_PACKAGES+=("$OSQP_PROVIDER")
  echo "Using $OSQP_PROVIDER from the distribution to provide OSQP." >&2
else
  if [[ -n ${ROS_DISTRO:-} ]]; then
    ROS_OSQP_VENDOR="ros-${ROS_DISTRO}-osqp-vendor"
    if apt-cache show "$ROS_OSQP_VENDOR" >/dev/null 2>&1; then
      OSQP_PROVIDER="$ROS_OSQP_VENDOR"
      REQUIRED_PACKAGES+=("$OSQP_PROVIDER")
      echo "$OSQP_NATIVE_PACKAGE not found; installing $OSQP_PROVIDER as the OSQP provider." >&2
    fi
  fi

  if [[ -z $OSQP_PROVIDER ]]; then
    echo "Neither $OSQP_NATIVE_PACKAGE nor ros-<ROS_DISTRO>-osqp-vendor are available via APT." >&2
    echo "OSQP will need to be built from source (see https://osqp.org/installation) or provided by another compatible package." >&2
  fi
fi

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
STACKTRACE_HEADER="/usr/include/boost/stacktrace.hpp"
if [[ ! -f $BOOST_HEADER || ! -f $STACKTRACE_HEADER ]]; then
  echo "Boost headers are missing; reinstalling Boost development packages" >&2
  for pkg in libboost-dev libboost-graph-dev libboost-program-options-dev libboost-serialization-dev libboost-stacktrace-dev; do
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

# Verify tinyxml2 installs its CMake package configuration so downstream
# packages that call find_package(tinyxml2) succeed.  The upstream package
# renamed the config file in Ubuntu Noble, so accept either variant to keep the
# check working across distributions.
TINYXML2_CONFIG_DIR=/usr/lib/x86_64-linux-gnu/cmake/tinyxml2
shopt -s nullglob
tinyxml2_configs=(
  "$TINYXML2_CONFIG_DIR"/tinyxml2Config.cmake
  "$TINYXML2_CONFIG_DIR"/tinyxml2-config.cmake
)
shopt -u nullglob

if [[ ${#tinyxml2_configs[@]} -eq 0 ]]; then
  echo "tinyxml2 CMake package config not found even after installation." >&2
  echo "Ensure libtinyxml2-dev is available for your distribution and rerun this script." >&2
  exit 1
fi
