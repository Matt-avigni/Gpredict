# Windows installer

This packaging path builds a 64-bit Windows installer that bundles:

- `gpredict.exe`
- the pinned modified Hamlib build (`rigctld.exe`, `rotctld.exe`, `libhamlib*.dll`)
- the GTK/Glib runtime required by the app
- Gpredict data, pixmaps, locale files, and CA bundle

The pinned Hamlib source lives in [`packaging/windows/hamlib-source.env`](./hamlib-source.env).
Update that file whenever the Windows installer should follow a different fork,
branch, or exact commit.

Packaging-specific Hamlib fixes that are needed to keep the Windows installer
reproducible live in [`packaging/windows/patches/`](./patches). Those patches
are applied automatically before the Hamlib build starts.

## CI

GitHub Actions builds the installer with
[`/.github/workflows/windows-installer.yml`](../../.github/workflows/windows-installer.yml).
The workflow clones the configured Hamlib branch and fails unless it resolves
to `HAMLIB_EXPECTED_COMMIT`. That keeps the installer tied to the exact
Hamlib revision Gpredict is expecting.

Artifacts:

- `gpredict-<version>-windows-x86_64-setup.exe`
- `gpredict-<version>-windows-x86_64-portable.zip`

## Manual build

Build under an MSYS2 `UCRT64` shell with these packages installed:

- `autoconf`
- `automake-wrapper`
- `intltool`
- `libtool`
- `make`
- `zip`
- `mingw-w64-ucrt-x86_64-gcc`
- `mingw-w64-ucrt-x86_64-binutils`
- `mingw-w64-ucrt-x86_64-pkgconf`
- `mingw-w64-ucrt-x86_64-curl`
- `mingw-w64-ucrt-x86_64-gtk3`
- `mingw-w64-ucrt-x86_64-goocanvas`
- `mingw-w64-ucrt-x86_64-librsvg`
- `mingw-w64-ucrt-x86_64-gsettings-desktop-schemas`
- `mingw-w64-ucrt-x86_64-adwaita-icon-theme`
- `mingw-w64-ucrt-x86_64-hicolor-icon-theme`
- `mingw-w64-ucrt-x86_64-ntldd`
- `mingw-w64-ucrt-x86_64-nsis`

Then:

```bash
git clone --depth 1 --branch Dev https://github.com/matteo-avigni/Hamlib.git hamlib-src
git -C hamlib-src checkout 0e27135a1bb2f4d709e7f2dc9ead4c6e2467f88e
packaging/windows/build-installer.sh
```

Artifacts are written to `build/windows-ucrt64/artifacts/`.
