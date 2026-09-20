#!/usr/bin/env bash
set -euo pipefail

mkdir -p "${HOME}/.codex"

touch /tmp/.docker.xauth
xauth nlist "${DISPLAY:-}" | sed -e 's/^..../ffff/' | xauth -f /tmp/.docker.xauth nmerge - 2>/dev/null || true
chmod 644 /tmp/.docker.xauth
