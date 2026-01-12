#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PREFIX="$HOME/gpredict-dev"

cd "$REPO_DIR"

# Force the correct branch name
git rev-parse --verify Dev >/dev/null 2>&1 || {
  echo "ERROR: branch 'Dev' does not exist. Run: git branch"
  exit 1
}
git checkout Dev

# Clean + rebuild in a dedicated build dir
rm -rf build
mkdir build
cd build

# Configure from build dir (never in source dir)
../configure --prefix="$PREFIX" --disable-maintainer-mode

make -j"$(sysctl -n hw.ncpu 2>/dev/null || echo 8)"
make install

# Run the newly installed binary (not whatever is in PATH)
exec "$PREFIX/bin/gpredict"
