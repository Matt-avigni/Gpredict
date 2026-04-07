#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PREFIX="${PREFIX:-$HOME/gpredict-install}"
BUILD_DIR="${BUILD_DIR:-$ROOT/build-latest}"
JOBS="${JOBS:-$(sysctl -n hw.ncpu 2>/dev/null || echo 8)}"

echo "repo: $ROOT"
echo "build: $BUILD_DIR"
echo "git:  $(git -C "$ROOT" rev-parse --short HEAD 2>/dev/null || echo 'no-git')"
echo "prefix: $PREFIX"

if [[ ! -x "$ROOT/configure" ]]; then
  (cd "$ROOT" && autoreconf -fi)
fi

mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

CURRENT_PREFIX=""
if [[ -f Makefile ]]; then
  CURRENT_PREFIX="$(sed -n 's/^prefix = //p' Makefile | head -n1)"
fi

if [[ ! -f Makefile || "$CURRENT_PREFIX" != "$PREFIX" ]]; then
  "$ROOT/configure" --prefix="$PREFIX"
fi

make -j"$JOBS"
make install

BIN="$PREFIX/bin/gpredict"

echo "running: $BIN"
ls -l "$BIN"
"$BIN" --version || true
exec "$BIN"
