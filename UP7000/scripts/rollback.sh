#!/usr/bin/env bash
# Usage: bash rollback.sh BACKUP_DIRECTORY [--restart]
set -euo pipefail
exec python3 -B "$(dirname "${BASH_SOURCE[0]}")/onboard_deploy.py" --rollback "$@"
