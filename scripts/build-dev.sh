#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PREFIX="${PREFIX:-$HOME/gpredict-dev}"
BUILD_DIR="${BUILD_DIR:-$REPO_DIR/build-dev}"
JOBS="${JOBS:-$(sysctl -n hw.ncpu 2>/dev/null || echo 8)}"

cd "$REPO_DIR"

if [[ ! -x configure ]]; then
  autoreconf -fi
fi

rm -rf "$BUILD_DIR"
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

../configure --prefix="$PREFIX" --disable-maintainer-mode

make -j"$JOBS"
make install

exec "$PREFIX/bin/gpredict"
