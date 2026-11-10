#!/usr/bin/env bash
# No source/setup/mkdir on the default path: dry-run performs no writes.
set -euo pipefail
exec python3 -B "$(dirname "${BASH_SOURCE[0]}")/onboard_deploy.py" "$@"
