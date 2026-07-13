#!/bin/bash
# Regenerates src/rai.cfg from src/rai.cfg.in, substituting this checkout's
# own absolute path for the @REPO_ROOT@ placeholder -- so rai.cfg never has
# any one user's home directory hardcoded into it. Safe to re-run any time
# (e.g. after moving or re-cloning the checkout); always overwrites
# src/rai.cfg from the template.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
sed "s|@REPO_ROOT@|$HERE|g" "$HERE/src/rai.cfg.in" > "$HERE/src/rai.cfg"
echo "generated $HERE/src/rai.cfg (REPO_ROOT=$HERE)"
