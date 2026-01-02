#!/usr/bin/env bash
# Common helpers for fix_and_build.sh

log_info()    { echo -e "\033[0;34m[INFO]\033[0m $*"; }
log_success() { echo -e "\033[0;32m[SUCCESS]\033[0m $*"; }
log_warn()    { echo -e "\033[0;33m[WARN]\033[0m $*" >&2; }
log_error()   { echo -e "\033[0;31m[ERROR]\033[0m $*" >&2; }

die() {
    log_error "$@"
    exit 1
}

ensure_dir() {
    local dir="$1"
    [[ -d "$dir" ]] || mkdir -p "$dir"
}

# Trap errors with context
trap 'die "Command failed at line $LINENO: $BASH_COMMAND"' ERR
