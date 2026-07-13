#!/usr/bin/env bash
# Dependency management helpers

clone_repositories() {
    local manifest
    manifest="$(resolve_dependency_manifest)"
    if [[ -z "$manifest" ]]; then
        log_warn "No dependency manifest found (checked dependencies/emd_epd_ws.repos, tesseract.repos); skipping repository import"
        return
    fi

    log_info "Importing repositories from ${manifest#$REPO_ROOT/}"
    local import_cmd=(vcs import --recursive --skip-existing "$SRC_DIR")
    if ! GIT_TERMINAL_PROMPT=${GIT_TERMINAL_PROMPT:-0} "${import_cmd[@]}" <"$manifest"; then
        die "vcs import failed; set GIT_TERMINAL_PROMPT=1 or configure credentials to continue"
    fi
}

resolve_dependency_manifest() {
    local canonical="$REPO_ROOT/dependencies/emd_epd_ws.repos"
    local legacy="$REPO_ROOT/tesseract.repos"

    if [[ -f "$canonical" ]]; then
        echo "$canonical"
        return 0
    fi

    if [[ -f "$legacy" ]]; then
        log_warn "Using legacy manifest tesseract.repos; prefer dependencies/emd_epd_ws.repos"
        echo "$legacy"
        return 0
    fi

    return 1
}

fix_workspace_layout() {
    log_info "Running workspace layout adjustments"
    WORKSPACE_ROOT="$WORKSPACE_ROOT" WS="$WORKSPACE_ROOT" SRC_DIR="$SRC_DIR" "$REPO_ROOT/scripts/fix_workspace_layout.sh"
}

verify_workspace_packages() {
    log_info "Verifying package discovery after layout fix"
    if ! colcon list --base-paths "$SRC_DIR" >/dev/null; then
        log_warn "colcon list failed (possibly due to cyclic dependencies); continuing"
    fi

    if colcon list --base-paths "$SRC_DIR" --names-only 2>/dev/null | grep -E '^tesseract_motion_planners($|_)' >/dev/null; then
        log_info "Verified tesseract motion planner source packages are present in workspace"
    else
        log_warn "No tesseract_motion_planners* packages discovered in workspace package list"
    fi
}

install_system_packages() {
    local required=(
        libboost-dev
        libboost-program-options-dev
        libboost-serialization-dev
        libboost-stacktrace-dev
        libboost-graph-dev
        libtinyxml2-dev
        libcereal-dev
        libeigen3-dev
        libconsole-bridge-dev
    )

    local missing=()
    for pkg in "${required[@]}"; do
        if ! dpkg -s "$pkg" >/dev/null 2>&1; then
            missing+=("$pkg")
        fi
    done

    if [[ ${#missing[@]} -gt 0 ]]; then
        apt_update_once
        log_info "Installing system packages: ${missing[*]}"
        $APT_GET install -y "${missing[@]}"
    else
        log_info "System packages already installed"
    fi

    if [[ -x "$REPO_ROOT/scripts/install_system_deps.sh" ]]; then
        ROS_DISTRO="$ROS_DISTRO" "$REPO_ROOT/scripts/install_system_deps.sh"
    fi
}

workspace_has_required_packages() {
    mapfile -t workspace_packages < <(colcon list --base-paths "$SRC_DIR" --names-only 2>/dev/null || true)
    if [[ ${#workspace_packages[@]} -eq 0 ]]; then
        mapfile -t workspace_packages < <(find "$SRC_DIR" -name package.xml -print0 | xargs -0 -n1 python3 - <<'PY'
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

pkg = Path(sys.argv[1])
try:
    name = ET.parse(pkg).getroot().findtext("name")
except Exception:
    name = None
if name:
    print(name.strip())
PY
        )
        if [[ ${#workspace_packages[@]} -eq 0 ]]; then
            return 1
        fi
    fi

    local -A present
    for pkg in "${workspace_packages[@]}"; do
        present["$pkg"]=1
    done

    local required=(trajopt_sco trajopt_common tesseract_collision tesseract_state_solver tesseract_urdf)
    local missing=()
    for pkg in "${required[@]}"; do
        if [[ -z ${present[$pkg]:-} ]]; then
            missing+=("$pkg")
        fi
    done

    if [[ ${#missing[@]} -gt 0 ]]; then
        log_warn "Workspace at $SRC_DIR is missing required packages: ${missing[*]}"
        return 1
    fi

    return 0
}

remove_duplicate_packages() {
    log_info "Checking for duplicate packages in workspace"
    local -A preferred=()
    local had_packages=0

    while read -r name path _; do
        [[ -z "$name" || -z "$path" ]] && continue
        had_packages=1

        if [[ -z ${preferred[$name]:-} ]]; then
            preferred[$name]="$path"
            continue
        fi

        local current_preferred="${preferred[$name]}"
        local keep="$current_preferred"
        local drop="$path"

        if [[ "$path" == "$REPO_ROOT"/* && "$current_preferred" != "$REPO_ROOT"/* ]]; then
            keep="$path"
            drop="$current_preferred"
        fi

        if [[ "$drop" == "$keep" ]]; then
            drop="$path"
        fi

        if [[ "$drop" == "$REPO_ROOT"/* ]]; then
            log_warn "Duplicate package $name at $drop; managed by repository, skipping removal"
            continue
        fi

        if [[ -d "$drop" ]]; then
            log_warn "Removing duplicate package $name from $drop (keeping $keep)"
            rm -rf "$drop"
        fi

        preferred[$name]="$keep"
    done < <(colcon list --base-paths "$SRC_DIR" 2>/dev/null || true)

    if [[ $had_packages -eq 0 ]]; then
        log_warn "No packages detected by colcon in $SRC_DIR (skipping duplicate pruning)"
    fi
}

install_rosdeps() {
    log_info "Installing rosdep dependencies"
    mapfile -t rosdep_paths < <(colcon list --base-paths "$SRC_DIR" --paths-only 2>/dev/null || true)

    if [[ ${#rosdep_paths[@]} -eq 0 ]]; then
        mapfile -t rosdep_paths < <(find "$SRC_DIR" -name package.xml -print0 | xargs -0 -n1 dirname | sort -u)
    fi

    [[ ${#rosdep_paths[@]} -gt 0 ]] || die "No packages found for rosdep installation"

    local ensure_overrides="$REPO_ROOT/scripts/ensure_rosdep_overrides.sh"
    if [[ -x "$ensure_overrides" ]]; then
        "$ensure_overrides" cereal
    else
        die "Missing rosdep override helper: $ensure_overrides"
    fi


    local skip_keys=(
        tesseract_visualization
        qt_advanced_docking
        osqp
        osqp_vendor
        osqp-eigen
        osqp_eigen
    )

    local preflight_helper="$REPO_ROOT/scripts/preflight_tesseract_apt.sh"
    if [[ -x "$preflight_helper" ]]; then
        local fallback_skip_csv=""
        local preflight_status=0
        if fallback_skip_csv="$(ROS_DISTRO="$ROS_DISTRO" "$preflight_helper" --print-rosdep-skip-keys)"; then
            log_info "APT preflight found binary ROS Tesseract + QtADS providers"
        else
            preflight_status=$?
            if [[ $preflight_status -eq 2 ]]; then
                log_warn "APT preflight selected source-overlay fallback path for Tesseract/QtADS"
            else
                die "APT preflight check failed unexpectedly (exit=$preflight_status)"
            fi
        fi

        if [[ -n "$fallback_skip_csv" ]]; then
            IFS=',' read -r -a fallback_skip_keys <<<"$fallback_skip_csv"
            local key
            for key in "${fallback_skip_keys[@]}"; do
                [[ -n "$key" ]] || continue
                skip_keys+=("$key")
            done
        fi
    else
        log_warn "APT preflight helper missing or not executable: $preflight_helper"
    fi

    local skip_arg
    skip_arg=$(IFS=,; echo "${skip_keys[*]}")

    rosdep install --from-paths "${rosdep_paths[@]}" --ignore-src -yr --rosdistro "$ROS_DISTRO" --skip-keys "$skip_arg"
}

install_dependencies() {
    if [[ ${SKIP_DEPS:-0} -eq 1 ]]; then
        log_info "Skipping dependency installation (--skip-deps)"
        return
    fi

    ensure_dir "$STATE_DIR"
    local marker="$STATE_DIR/deps_installed"
    if [[ -f "$marker" ]]; then
        if workspace_has_required_packages; then
            log_info "Dependency installation already completed (marker present)"
            return
        fi

        log_warn "Dependency marker is present but required packages are missing; reinstalling dependencies"
        rm -f "$marker"
    fi

    clone_repositories
    fix_workspace_layout
    verify_workspace_packages
    remove_duplicate_packages
    install_system_packages
    install_rosdeps

    touch "$marker"
}
