#!/usr/bin/env bash
# Dependency management helpers

clone_repositories() {
    if [[ ! -f "$REPO_ROOT/tesseract.repos" ]]; then
        log_warn "No tesseract.repos file found; skipping repository import"
        return
    fi

    log_info "Importing repositories from tesseract.repos"
    local import_cmd=(vcs import --recursive --skip-existing "$SRC_DIR")
    if ! GIT_TERMINAL_PROMPT=${GIT_TERMINAL_PROMPT:-0} "${import_cmd[@]}" <"$REPO_ROOT/tesseract.repos"; then
        die "vcs import failed; set GIT_TERMINAL_PROMPT=1 or configure credentials to continue"
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
        return 1
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
        log_warn "No packages detected by colcon in $SRC_DIR"
    fi
}

install_rosdeps() {
    log_info "Installing rosdep dependencies"
    mapfile -t rosdep_paths < <(colcon list --base-paths "$SRC_DIR" --paths-only)
    [[ ${#rosdep_paths[@]} -gt 0 ]] || die "No packages found for rosdep installation"

    local overrides="$REPO_ROOT/scripts/rosdep_overrides.yaml"
    if [[ -f "$overrides" ]]; then
        export ROSDEP_ADDITIONAL_SOURCES_PATHS="$overrides${ROSDEP_ADDITIONAL_SOURCES_PATHS:+:$ROSDEP_ADDITIONAL_SOURCES_PATHS}"
        log_info "Using rosdep overrides from $overrides"
    fi

    rosdep update

    local skip_keys=(
        catkin
        rviz
        roslib
        tesseract
        tesseract_process_planners
        trajopt_ifopt
        trajopt_sqp
        trajopt
        jsoncpp
        message_generation
    )
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
    remove_duplicate_packages
    install_system_packages
    install_rosdeps

    touch "$marker"
}
