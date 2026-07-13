#!/usr/bin/env bash
set -euo pipefail

WORKSPACE=${1:-${WORKSPACE:-$(pwd)}}
SRC_DIR="$WORKSPACE/src"
REPO_ROOT="${REPO_ROOT:-$SRC_DIR/easy_manipulation_deployment}"
EXPECTED_TRAJOPT_REF="884e6177c4a220fc20a0c43100a0473c180a3bec"
EXPECTED_OSQP_REF="v1.0.0"
REQUIRE_PINNED_OSQP_PROVIDER=${REQUIRE_PINNED_OSQP_PROVIDER:-1}

fail() { echo "TrajOpt/OSQP preflight: FAILED: $*" >&2; exit 1; }
info() { echo "TrajOpt/OSQP preflight: $*"; }

[[ -d "$SRC_DIR" ]] || fail "workspace source directory not found: $SRC_DIR"

mapfile -t sco_pkgs < <(find "$SRC_DIR" -path '*/trajopt_sco/package.xml' -print | sort)
[[ ${#sco_pkgs[@]} -gt 0 ]] || fail "trajopt_sco package not found under $SRC_DIR. Corrective command: vcs import --recursive --skip-existing src < src/easy_manipulation_deployment/dependencies/emd_epd_ws.repos"
[[ ${#sco_pkgs[@]} -eq 1 ]] || fail "multiple trajopt_sco packages found: ${sco_pkgs[*]}. Remove stale/flattened duplicate checkouts, then rerun scripts/fix_workspace_layout.sh."

sco_dir=$(dirname "${sco_pkgs[0]}")
trajopt_root="$sco_dir"
while [[ "$trajopt_root" != "$SRC_DIR" && ! -d "$trajopt_root/.git" ]]; do
  trajopt_root=$(dirname "$trajopt_root")
done
[[ -d "$trajopt_root/.git" ]] || fail "trajopt_sco at $sco_dir is not inside a git checkout; likely stale/flattened source. Corrective command: remove that tree and re-import dependencies/emd_epd_ws.repos."
actual_ref=$(git -C "$trajopt_root" rev-parse HEAD 2>/dev/null || true)
[[ -n "$actual_ref" ]] || fail "cannot read TrajOpt git commit at $trajopt_root"
[[ "$actual_ref" == "$EXPECTED_TRAJOPT_REF" ]] || fail "wrong TrajOpt commit at $trajopt_root: expected $EXPECTED_TRAJOPT_REF (trajopt 0.33.0), found $actual_ref. Corrective command: git -C '$trajopt_root' fetch --tags && git -C '$trajopt_root' checkout '$EXPECTED_TRAJOPT_REF'"
info "TrajOpt checkout OK: $trajopt_root @ $actual_ref"

osqp_src="$SRC_DIR/osqp"
[[ -d "$osqp_src/.git" ]] || fail "pinned OSQP source checkout is required at $osqp_src; generic Jammy libosqp-dev is API-incompatible. Corrective command: vcs import --recursive --skip-existing src < src/easy_manipulation_deployment/dependencies/emd_epd_ws.repos"
osqp_ref=$(git -C "$osqp_src" describe --tags --exact-match 2>/dev/null || git -C "$osqp_src" rev-parse HEAD 2>/dev/null || true)
[[ "$osqp_ref" == "$EXPECTED_OSQP_REF" ]] || fail "wrong OSQP source at $osqp_src: expected $EXPECTED_OSQP_REF, found $osqp_ref. Corrective command: git -C '$osqp_src' fetch --tags && git -C '$osqp_src' checkout '$EXPECTED_OSQP_REF'"

check_header_dir() {
  local dir="$1"
  [[ -f "$dir/osqp_api_types.h" ]] || return 1
  grep -R -q "OSQPSolver" "$dir" || return 1
  grep -R -q "OSQPInt" "$dir" || return 1
  grep -R -q "OSQPCscMatrix" "$dir" || return 1
  grep -R -q "osqp_update_data_vec" "$dir" || return 1
  grep -R -q "warm_starting" "$dir" || return 1
}

providers=()
for dir in /usr/local/include/osqp /usr/include/osqp; do
  [[ -d "$dir" ]] && providers+=("$dir")
done
[[ ${#providers[@]} -gt 0 ]] || fail "no installed OSQP headers found. Build/install pinned OSQP $EXPECTED_OSQP_REF from $osqp_src before building trajopt_sco."

for dir in "${providers[@]}"; do
  if check_header_dir "$dir"; then
    if [[ "$REQUIRE_PINNED_OSQP_PROVIDER" == "1" && "$dir" == /usr/include/osqp ]]; then
      fail "OSQP v1-compatible headers were found from /usr ($dir), but this workspace requires the pinned source provider from /usr/local. Corrective command: build/install src/osqp $EXPECTED_OSQP_REF to /usr/local and ensure CMAKE_PREFIX_PATH prefers /usr/local."
    fi
    info "OSQP headers OK: $dir"
    exit 0
  fi
done

fail "incompatible OSQP headers found (${providers[*]}): trajopt 0.33.0 requires the OSQP v1 API symbols OSQPSolver, OSQPCscMatrix, OSQPInt, osqp_update_data_vec/osqp_update_data_mat, OSQPSettings::warm_starting, OSQP_NO_ERROR and OSQP_ALGEBRA_LOAD_ERROR. Jammy libosqp-dev exposes the older OSQPWorkspace/warm_start API."
