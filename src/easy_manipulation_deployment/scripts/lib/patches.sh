#!/usr/bin/env bash
# Patch application helpers

PATCH_DIR="$SCRIPT_DIR/patches"
PATCH_MARKER_FILE="$STATE_DIR/patches_applied"

list_patches() {
    [[ -d "$PATCH_DIR" ]] || return 0
    find "$PATCH_DIR" -maxdepth 1 -name '*.patch' | sort
}

is_patch_applied() {
    local patch="$1"
    git -C "$REPO_ROOT" apply --reverse --check "$patch" >/dev/null 2>&1
}

record_patch() {
    local name="$1"
    ensure_dir "$STATE_DIR"
    grep -Fxq "$name" "$PATCH_MARKER_FILE" 2>/dev/null || echo "$name" >>"$PATCH_MARKER_FILE"
}

apply_patches() {
    if [[ ${SKIP_PATCHES:-0} -eq 1 ]]; then
        log_info "Skipping patch application (--skip-patches)"
        return
    fi

    local patches=()
    mapfile -t patches < <(list_patches)
    if [[ ${#patches[@]} -eq 0 ]]; then
        log_warn "No patches found under $PATCH_DIR"
        return
    fi

    pushd "$REPO_ROOT" >/dev/null
    for patch in "${patches[@]}"; do
        local name
        name="$(basename "$patch")"
        if is_patch_applied "$patch"; then
            log_info "Patch $name already applied"
            record_patch "$name"
            continue
        fi

        git apply --check "$patch" || die "Patch $name failed validation"
        git apply --whitespace=fix "$patch"
        record_patch "$name"
        log_info "Applied patch $name"
    done
    popd >/dev/null

    touch "$PATCH_MARKER_FILE"
}

revert_patches() {
    local patches=()
    mapfile -t patches < <(list_patches | sort -r)
    [[ ${#patches[@]} -gt 0 ]] || return

    pushd "$REPO_ROOT" >/dev/null
    for patch in "${patches[@]}"; do
        local name
        name="$(basename "$patch")"
        if git apply --reverse --check "$patch" >/dev/null 2>&1; then
            git apply -R "$patch"
            log_info "Reverted patch $name"
        fi
    done
    popd >/dev/null

    rm -f "$PATCH_MARKER_FILE"
}
