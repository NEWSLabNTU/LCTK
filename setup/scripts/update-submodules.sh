#!/usr/bin/env bash
# Update git submodules
# Warning: This discards any local changes in submodules

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

echo "Updating git submodules..."

cd "$PROJECT_ROOT"

# Initialize and update submodules, discarding local changes
git submodule update --init --recursive --force

echo "Git submodules updated."
