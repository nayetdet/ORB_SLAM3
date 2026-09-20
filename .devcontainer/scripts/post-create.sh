#!/usr/bin/env bash
set -euo pipefail

mkdir -p /home/vscode/.local/share/zsh
bun add --global @openai/codex
