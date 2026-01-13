# HACKING-macos-rotctld-local

This note documents how to build Hamlib and Gpredict on macOS so Gpredict
spawns `rotctld` from `$HOME/hamlib-local` and finds the correct dylibs
without environment variables.

## Build Hamlib into $HOME/hamlib-local

```sh
cd /path/to/hamlib
./configure --prefix="$HOME/hamlib-local"
make -j
make install
```

## Build Gpredict against that Hamlib

```sh
cd /path/to/Gpredict
autoreconf -fi
./configure --prefix="$HOME/gpredict-local"
make -j
make install
```

## Verify rotctld location

```sh
$HOME/hamlib-local/bin/rotctld -V
```

Launch Gpredict from Finder and open the rotator control window. The log
should include a `rotctld_mgr: spawn argv` line showing the absolute
`$HOME/hamlib-local/bin/rotctld` path.

## Verify dylib usage

```sh
otool -L src/gpredict | grep -i hamlib
otool -l src/gpredict | grep -A2 LC_RPATH
```

If you are inspecting an installed binary, replace `src/gpredict` with:

```sh
$HOME/gpredict-local/bin/gpredict
```
