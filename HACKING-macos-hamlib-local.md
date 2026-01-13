macOS Hamlib Local Build

This build is wired to prefer Hamlib installed under:
  $HOME/hamlib-local

Commands (fresh shell):
  autoreconf -fi
  ./configure --prefix="$HOME/gpredict-local"
  make -j
  make install

Verification (build + installed binary):
  otool -L "$PWD/src/gpredict" | grep -i hamlib
  otool -l "$PWD/src/gpredict" | grep -A2 LC_RPATH
  otool -L "$HOME/gpredict-local/bin/gpredict" | grep -i hamlib
  otool -l "$HOME/gpredict-local/bin/gpredict" | grep -A2 LC_RPATH

Expected: hamlib dylib paths resolve under $HOME/hamlib-local/lib.
