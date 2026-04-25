#!/usr/bin/env bash
# Build orchestration helpers

source "$SCRIPT_DIR/lib/motion_planner_configs.sh"

BASE_CMAKE_ARGS=(
    # Workspace compatibility baseline: Ubuntu 22.04 / ROS 2 Humble / C++17-era libstdc++.
    # Keep Ruckig on the pre-<format> line documented in dependencies/emd_epd_ws.repos.
    -DCMAKE_CXX_STANDARD=17
    -DCMAKE_CXX_STANDARD_REQUIRED=ON
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON
    -DTESSERACT_ENABLE_TESTING=OFF
    -DTESSERACT_ENABLE_EXAMPLES=OFF
)

reset_colcon_environment() {
    local ros_distro="${ROS_DISTRO:-humble}"
    local ros_setup="/opt/ros/${ros_distro}/setup.bash"
    local local_setup="$WORKSPACE_ROOT/install/local_setup.bash"

    if [[ "$ros_distro" != "humble" ]]; then
        die "Unsupported ROS_DISTRO='${ros_distro}' for this build helper (expected humble)"
    fi

    unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH

    if [[ ! -f "$ros_setup" ]]; then
        die "Expected ROS base setup is missing: $ros_setup"
    fi

    set +u
    : "${AMENT_TRACE_SETUP_FILES:=}"
    # shellcheck source=/dev/null
    source "$ros_setup"
    if [[ -f "$local_setup" ]]; then
        # shellcheck source=/dev/null
        source "$local_setup"
    fi
    set -u
}

source_install() {
    local setup_file="$WORKSPACE_ROOT/install/local_setup.bash"
    if [[ -f "$setup_file" ]]; then
        set +u
        # shellcheck source=/dev/null
        source "$setup_file"
        set -u
        return
    fi

    setup_file="$WORKSPACE_ROOT/install/setup.bash"
    if [[ -f "$setup_file" ]]; then
        set +u
        # shellcheck source=/dev/null
        source "$setup_file"
        set -u
    fi
}

compose_cmake_args() {
    CMAKE_ARGS=("${BASE_CMAKE_ARGS[@]}")

    if [[ ${LIGHTWEIGHT:-0} -eq 1 ]]; then
        CMAKE_ARGS+=(
            -DTESSERACT_BUILD_OMPL=OFF
            -DTESSERACT_BUILD_DESCARTES=OFF
            -DTESSERACT_BUILD_TRAJOPT=OFF
            -DTESSERACT_BUILD_TRAJOPT_IFOPT=OFF
        )
    fi
}


colcon_override_args() {
    # This workspace intentionally overlays source packages on top of the ROS underlay.
    # Keep these explicit so duplicate-package warnings stay actionable and future
    # colcon releases do not fail hard on intentional overrides.
    local args=(
        --allow-overriding
        ruckig
        tesseract_monitoring
        tesseract_msgs
        tesseract_rosutils
    )
    printf '%s
' "${args[@]}"
}

colcon_base_args() {
    local args=(
        --symlink-install
        --merge-install
        --parallel-workers "$JOBS"
    )

    if [[ ${VERBOSE:-0} -eq 1 ]]; then
        args+=(--event-handlers console_cohesion+ console_direct+)
    else
        args+=(--event-handlers console_cohesion+)
    fi

    printf '%s
' "${args[@]}"
}

# Copy motion planner component configs to expected locations
# CMake expects each component in its own directory
copy_motion_planner_configs() {
    local install_cmake_dir="$WORKSPACE_ROOT/install/lib/cmake"
    local src_dir="${install_cmake_dir}/tesseract_motion_planners"

    if [[ ! -d "$src_dir" ]]; then
        return 0
    fi

    log_info "Copying motion planner component configs to expected locations"

    for comp in core simple ompl descartes trajopt; do
        local config_file="${src_dir}/tesseract_motion_planners_${comp}-config.cmake"
        if [[ -f "$config_file" ]]; then
            local dst_dir="${install_cmake_dir}/tesseract_motion_planners_${comp}"
            mkdir -p "$dst_dir"
            cp "${src_dir}/tesseract_motion_planners_${comp}-config.cmake" "$dst_dir/"
            cp "${src_dir}/tesseract_motion_planners_${comp}-targets"*.cmake "$dst_dir/" 2>/dev/null || true
        fi
    done
}

repair_motion_planner_configs() {
    local install_dir="$WORKSPACE_ROOT/install"
    if [[ ! -d "$install_dir" ]]; then
        return 0
    fi

    log_info "Repairing motion planner component configs in ${install_dir}"
    ensure_motion_planners_component_configs "$WORKSPACE_ROOT" || true
}

ensure_install_layout() {
    local install_dir="$WORKSPACE_ROOT/install"
    local layout_file="${install_dir}/.colcon_install_layout"

    if [[ ! -d "$install_dir" ]]; then
        return 0
    fi

    if [[ -f "$layout_file" ]]; then
        local layout
        layout="$(<"$layout_file")"
        if [[ "$layout" != "merged" ]]; then
            log_warn "Existing install directory uses layout '${layout}'; removing to enforce merged install"
            rm -rf "$install_dir"
            rm -f "$STATE_DIR/foundation_built"
        fi
    else
        log_warn "Existing install directory missing layout marker; removing to enforce merged install"
        rm -rf "$install_dir"
        rm -f "$STATE_DIR/foundation_built"
    fi
}

build_workspace() {
    compose_cmake_args
    ensure_dir "$STATE_DIR"
    ensure_install_layout
    repair_motion_planner_configs

    local foundation_marker="$STATE_DIR/foundation_built"
    local trajopt_setup="$WORKSPACE_ROOT/install/share/trajopt_sco/local_setup.bash"
    local args
    local override_args
    mapfile -t args < <(colcon_base_args)
    mapfile -t override_args < <(colcon_override_args)

    pushd "$WORKSPACE_ROOT" >/dev/null

    if [[ -f "$foundation_marker" && ! -f "$trajopt_setup" ]]; then
        log_warn "Foundation marker present but ${trajopt_setup} is missing; rebuilding foundation packages"
        rm -f "$foundation_marker"
    fi

    if [[ ! -f "$foundation_marker" ]]; then
        log_info "Building foundation packages (through trajopt_sco)"
        reset_colcon_environment
        colcon build --base-paths "$SRC_DIR" "${args[@]}" "${override_args[@]}" \
            --packages-up-to trajopt_sco --cmake-args "${CMAKE_ARGS[@]}"
        touch "$foundation_marker"
    else
        log_info "Foundation packages already built; skipping first pass"
    fi

    source_install
    repair_motion_planner_configs

    local full_args=("${args[@]}")
    if [[ ${#PACKAGES[@]} -gt 0 ]]; then
        full_args+=(--packages-select "${PACKAGES[@]}")
    fi

    log_info "Building full workspace"
    reset_colcon_environment
    colcon build --base-paths "$SRC_DIR" "${full_args[@]}" "${override_args[@]}" --cmake-args "${CMAKE_ARGS[@]}"

    copy_motion_planner_configs
    ensure_motion_planners_component_configs "$WORKSPACE_ROOT"

    popd >/dev/null
}

clean_build_artifacts() {
    log_info "Removing build, install, and log directories"
    rm -rf "$WORKSPACE_ROOT/build" "$WORKSPACE_ROOT/install" "$WORKSPACE_ROOT/log"
    rm -f "$STATE_DIR/foundation_built"
}
