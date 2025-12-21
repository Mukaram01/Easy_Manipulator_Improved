#!/usr/bin/env bash
# Environment detection and setup helpers

APT_GET="apt-get"
APT_UPDATED=0

setup_locale() {
    export LANG=C.UTF-8 LC_ALL=C.UTF-8
}

detect_workspace() {
    local repo_parent
    repo_parent="$(dirname "$REPO_ROOT")"

    if [[ -n ${WS:-} ]]; then
        WORKSPACE_ROOT="$(cd "$WS" && pwd)"
    elif [[ $(basename "$repo_parent") == "src" ]]; then
        WORKSPACE_ROOT="$(cd "$repo_parent/.." && pwd)"
    else
        WORKSPACE_ROOT="$HOME/workcell_ws"
    fi

    SRC_DIR="$WORKSPACE_ROOT/src"
    ensure_dir "$SRC_DIR"
}

detect_ros_distro() {
    if [[ -n ${ROS_DISTRO:-} ]]; then
        [[ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]] || die "ROS_DISTRO is set to '$ROS_DISTRO' but /opt/ros/$ROS_DISTRO/setup.bash is missing"
        return
    fi

    for distro in humble jazzy; do
        if [[ -f "/opt/ros/$distro/setup.bash" ]]; then
            ROS_DISTRO="$distro"
            return
        fi
    done

    die "No supported ROS distro found (expected humble or jazzy under /opt/ros)"
}

source_ros() {
    log_info "Using ROS_DISTRO=$ROS_DISTRO"
    set +u
    : "${AMENT_TRACE_SETUP_FILES:=}"
    # shellcheck source=/dev/null
    source "/opt/ros/$ROS_DISTRO/setup.bash"
    set -u
}

configure_apt() {
    if command -v sudo >/dev/null 2>&1; then
        if sudo -n true 2>/dev/null; then
            APT_GET="sudo apt-get"
        elif [[ $EUID -ne 0 ]]; then
            log_warn "sudo requires a password; commands may prompt as needed"
            APT_GET="sudo apt-get"
        fi
    elif [[ $EUID -ne 0 ]]; then
        die "Root privileges are required to install packages"
    fi
}

apt_update_once() {
    if [[ $APT_UPDATED -eq 0 ]]; then
        $APT_GET update -y
        APT_UPDATED=1
    fi
}

ensure_tools() {
    local packages=()
    for tool in git rosdep vcs colcon; do
        if ! command -v "$tool" >/dev/null 2>&1; then
            case "$tool" in
                git) packages+=(git);;
                rosdep) packages+=(python3-rosdep);;
                vcs) packages+=(python3-vcstool);;
                colcon) packages+=(python3-colcon-common-extensions);;
            esac
        fi
    done

    if [[ ${#packages[@]} -gt 0 ]]; then
        apt_update_once
        $APT_GET install -y "${packages[@]}"
    fi

    if command -v rosdep >/dev/null 2>&1; then
        if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
            if [[ $EUID -eq 0 ]]; then
                rosdep init
            elif command -v sudo >/dev/null 2>&1; then
                sudo rosdep init || log_warn "rosdep init failed; continuing"
            else
                log_warn "rosdep init skipped (requires root)"
            fi
        fi
    fi
}

setup_environment() {
    detect_workspace
    setup_locale
    detect_ros_distro
    configure_apt
    ensure_tools
    source_ros
}
