#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PREFIX="${PREFIX:-$HOME/gpredict-install}"

cd "$ROOT"

echo "repo: $ROOT"
echo "git:  $(git rev-parse --short HEAD 2>/dev/null || echo 'no-git')"
echo "prefix: $PREFIX"

make -j"$(sysctl -n hw.ncpu)"
make install

BIN="$PREFIX/bin/gpredict"

echo "running: $BIN"
ls -l "$BIN"
"$BIN" --version || true
exec "$BIN"
