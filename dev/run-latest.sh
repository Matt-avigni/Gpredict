#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PREFIX="${PREFIX:-$HOME/gpredict-install}"
JOBS="${JOBS:-$(sysctl -n hw.ncpu 2>/dev/null || echo 8)}"

cd "$ROOT"

echo "repo: $ROOT"
echo "git:  $(git rev-parse --short HEAD 2>/dev/null || echo 'no-git')"
echo "prefix: $PREFIX"

CURRENT_PREFIX=""
if [[ -f Makefile ]]; then
  CURRENT_PREFIX="$(sed -n 's/^prefix = //p' Makefile | head -n1)"
fi

if [[ ! -f Makefile || "$CURRENT_PREFIX" != "$PREFIX" ]]; then
  ./configure --prefix="$PREFIX"
  if [[ -n "$CURRENT_PREFIX" && "$CURRENT_PREFIX" != "$PREFIX" ]]; then
    # The install prefix is compiled into PACKAGE_* paths, so switch prefixes
    # by forcing a rebuild of object files without touching tracked test files.
    find src -type f \( -name '*.o' -o -name '*.lo' -o -name 'gpredict' \) -delete
  fi
fi

make -j"$JOBS"
make install

BIN="$PREFIX/bin/gpredict"

echo "running: $BIN"
ls -l "$BIN"
"$BIN" --version || true
exec "$BIN"
