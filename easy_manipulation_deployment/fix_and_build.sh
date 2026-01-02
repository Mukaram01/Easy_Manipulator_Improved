#!/usr/bin/env bash
# Wrapper to preserve legacy entry point; delegates to scripts/fix_and_build.sh
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "$SCRIPT_DIR/scripts/fix_and_build.sh" "$@"
