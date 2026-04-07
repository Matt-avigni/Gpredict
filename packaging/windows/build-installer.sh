#!/usr/bin/env bash
set -euo pipefail

log() {
    printf '==> %s\n' "$*"
}

die() {
    printf 'error: %s\n' "$*" >&2
    exit 1
}

require_dir() {
    local dir="$1"

    [[ -d "$dir" ]] || die "missing directory: $dir"
}

copy_dir_contents() {
    local src="$1"
    local dest="$2"

    [[ -d "$src" ]] || return 0
    mkdir -p "$dest"
    cp -R "$src"/. "$dest"/
}

copy_optional_file() {
    local src="$1"
    local dest="$2"

    [[ -f "$src" ]] || return 0
    install -m 0644 "$src" "$dest"
}

apply_hamlib_patches() {
    local patch_dir="$REPO_ROOT/packaging/windows/patches"
    local patch_file

    [[ -d "$patch_dir" ]] || return 0

    while IFS= read -r -d '' patch_file; do
        if git -C "$HAMLIB_SRC_DIR" apply --reverse --check "$patch_file" >/dev/null 2>&1; then
            log "skipping $(basename "$patch_file") (already applied)"
            continue
        fi

        log "applying $(basename "$patch_file")"
        git -C "$HAMLIB_SRC_DIR" apply --check "$patch_file"
        git -C "$HAMLIB_SRC_DIR" apply "$patch_file"
    done < <(find "$patch_dir" -type f -name '*.patch' -print0 | sort -z)
}

normalize_path() {
    local raw="$1"

    case "$raw" in
        [A-Za-z]:\\*)
            cygpath -u "$raw"
            ;;
        *)
            printf '%s\n' "$raw"
            ;;
    esac
}

copy_dependency_tree() {
    local target="$1"

    while IFS= read -r dep; do
        [[ -n "$dep" ]] || continue
        dep="$(normalize_path "$dep")"

        case "$dep" in
            "$MINGW_PREFIX"/*|"$HAMLIB_PREFIX"/*)
                [[ -f "$dep" ]] || continue
                install -m 0755 "$dep" "$APP_DIR/$(basename "$dep")"
                ;;
        esac
    done < <(
        ntldd -R "$target" 2>/dev/null | tr -d '\r' | sed -nE \
            -e 's#^.*=> ([A-Za-z]:[^ ]+|/[^ ]+) .*#\1#p' \
            -e 's#^([A-Za-z]:[^ ]+|/[^ ]+) .*#\1#p' | sort -u
    )
}

copy_stage_dependencies() {
    local pass

    for pass in 1 2; do
        while IFS= read -r -d '' target; do
            copy_dependency_tree "$target"
        done < <(find "$APP_DIR" -type f \( -iname '*.exe' -o -iname '*.dll' \) -print0)
    done
}

copy_hamlib_binaries() {
    local have_rigctld=0
    local have_rotctld=0
    local candidate

    for candidate in \
        "$HAMLIB_PREFIX/rigctld.exe" \
        "$HAMLIB_PREFIX/rotctld.exe" \
        "$HAMLIB_PREFIX/bin/rigctld.exe" \
        "$HAMLIB_PREFIX/bin/rotctld.exe"; do
        if [[ -f "$candidate" ]]; then
            install -m 0755 "$candidate" "$APP_DIR/$(basename "$candidate")"
            case "$(basename "$candidate")" in
                rigctld.exe) have_rigctld=1 ;;
                rotctld.exe) have_rotctld=1 ;;
            esac
        fi
    done

    [[ "$have_rigctld" -eq 1 ]] || die "failed to locate rigctld.exe in $HAMLIB_PREFIX"
    [[ "$have_rotctld" -eq 1 ]] || die "failed to locate rotctld.exe in $HAMLIB_PREFIX"

    while IFS= read -r -d '' dll; do
        install -m 0755 "$dll" "$APP_DIR/$(basename "$dll")"
    done < <(
        find "$HAMLIB_PREFIX" -type f \
            \( -iname 'libhamlib*.dll' -o -iname 'hamlib*.dll' \) \
            -print0
    )
}

stage_runtime_layout() {
    local pixbuf_dir pixbuf_subdir

    pixbuf_dir="$(find "$MINGW_PREFIX/lib/gdk-pixbuf-2.0" -type d -name loaders | head -n 1)"
    [[ -n "$pixbuf_dir" ]] || die "failed to locate gdk-pixbuf loaders under $MINGW_PREFIX"
    pixbuf_subdir="${pixbuf_dir#"$MINGW_PREFIX/lib/"}"
    pixbuf_subdir="${pixbuf_subdir%/loaders}"

    copy_dir_contents "$MINGW_PREFIX/etc/fonts" "$APP_DIR/etc/fonts"
    copy_dir_contents "$MINGW_PREFIX/etc/gtk-3.0" "$APP_DIR/etc/gtk-3.0"
    copy_dir_contents "$MINGW_PREFIX/etc/pango" "$APP_DIR/etc/pango"

    copy_dir_contents "$MINGW_PREFIX/lib/gio" "$APP_DIR/lib/gio"
    copy_dir_contents "$MINGW_PREFIX/lib/gtk-3.0" "$APP_DIR/lib/gtk-3.0"
    copy_dir_contents "$MINGW_PREFIX/libexec/glib-2.0" "$APP_DIR/libexec/glib-2.0"
    copy_dir_contents "$MINGW_PREFIX/share/glib-2.0" "$APP_DIR/share/glib-2.0"
    copy_dir_contents "$MINGW_PREFIX/share/icons/Adwaita" "$APP_DIR/share/icons/Adwaita"
    copy_dir_contents "$MINGW_PREFIX/share/icons/hicolor" "$APP_DIR/share/icons/hicolor"
    copy_dir_contents "$MINGW_PREFIX/share/themes" "$APP_DIR/share/themes"

    mkdir -p "$APP_DIR/lib/$pixbuf_subdir"
    copy_dir_contents "$pixbuf_dir" "$APP_DIR/lib/$pixbuf_subdir/loaders"
    install -m 0644 "$REPO_ROOT/win32/loaders.cache" \
        "$APP_DIR/lib/$pixbuf_subdir/loaders.cache"

    copy_optional_file "$MINGW_PREFIX/bin/gspawn-win64-helper.exe" "$APP_DIR/gspawn-win64-helper.exe"
    copy_optional_file "$MINGW_PREFIX/bin/gspawn-win64-helper-console.exe" \
        "$APP_DIR/gspawn-win64-helper-console.exe"

    if [[ -f "$MINGW_PREFIX/bin/curl-ca-bundle.crt" ]]; then
        copy_optional_file "$MINGW_PREFIX/bin/curl-ca-bundle.crt" "$APP_DIR/curl-ca-bundle.crt"
    elif [[ -f "$MINGW_PREFIX/ssl/certs/ca-bundle.crt" ]]; then
        copy_optional_file "$MINGW_PREFIX/ssl/certs/ca-bundle.crt" "$APP_DIR/curl-ca-bundle.crt"
    fi

    sed "s#@GDK_PIXBUF_SUBDIR@#${pixbuf_subdir//\//\\\\}#g" \
        "$REPO_ROOT/packaging/windows/gpredict-launcher.cmd.in" \
        > "$APP_DIR/gpredict.cmd"
    chmod 0755 "$APP_DIR/gpredict.cmd"
}

build_hamlib() {
    apply_hamlib_patches

    log "bootstrapping Hamlib"
    pushd "$HAMLIB_SRC_DIR" >/dev/null
    ./bootstrap
    popd >/dev/null

    log "configuring Hamlib"
    mkdir -p "$HAMLIB_BUILD_DIR"
    pushd "$HAMLIB_BUILD_DIR" >/dev/null
    "$HAMLIB_SRC_DIR/configure" \
        --prefix="$HAMLIB_PREFIX" \
        --bindir='${prefix}' \
        --without-cxx-binding \
        --without-indi \
        --without-libusb \
        --without-lua-binding \
        --without-perl-binding \
        --without-python-binding \
        --without-readline \
        --without-tcl-binding \
        --disable-html-matrix

    log "building Hamlib"
    make -j"$(nproc)"

    log "installing Hamlib"
    make install
    popd >/dev/null
}

build_gpredict() {
    export PATH="$HAMLIB_PREFIX:$HAMLIB_PREFIX/bin:$MINGW_PREFIX/bin:$PATH"
    export PKG_CONFIG_PATH="$HAMLIB_PREFIX/lib/pkgconfig${PKG_CONFIG_PATH+:$PKG_CONFIG_PATH}"

    log "bootstrapping Gpredict"
    pushd "$REPO_ROOT" >/dev/null
    autoreconf -fi
    popd >/dev/null

    mkdir -p "$GPREDICT_BUILD_DIR"
    pushd "$GPREDICT_BUILD_DIR" >/dev/null
    log "configuring Gpredict"
    "$REPO_ROOT/configure" \
        --prefix="$APP_DIR" \
        --bindir='${prefix}'

    log "building Gpredict"
    make -j"$(nproc)"

    log "installing Gpredict"
    make install
    popd >/dev/null
}

make_portable_zip() {
    log "creating portable zip"
    pushd "$STAGE_ROOT" >/dev/null
    rm -f "$ARTIFACT_DIR/$PORTABLE_ARCHIVE"
    zip -9 -r "$ARTIFACT_DIR/$PORTABLE_ARCHIVE" Gpredict >/dev/null
    popd >/dev/null
}

make_installer() {
    local stage_dir_windows out_file_windows

    stage_dir_windows="$(cygpath -w "$APP_DIR")"
    out_file_windows="$(cygpath -w "$ARTIFACT_DIR/$INSTALLER_NAME")"

    log "building NSIS installer"
    pushd "$REPO_ROOT" >/dev/null
    makensis \
        -DAPP_NAME="Gpredict" \
        -DAPP_VERSION="$APP_VERSION" \
        -DOUT_FILE="$out_file_windows" \
        -DSTAGE_DIR="$stage_dir_windows" \
        "$REPO_ROOT/packaging/windows/gpredict-windows-installer.nsi"
    popd >/dev/null
}

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
MANIFEST="$REPO_ROOT/packaging/windows/hamlib-source.env"

[[ -f "$MANIFEST" ]] || die "missing Hamlib packaging manifest: $MANIFEST"
# shellcheck disable=SC1090
source "$MANIFEST"

: "${HAMLIB_REPOSITORY:?missing HAMLIB_REPOSITORY}"
: "${HAMLIB_REF:?missing HAMLIB_REF}"
: "${HAMLIB_EXPECTED_COMMIT:?missing HAMLIB_EXPECTED_COMMIT}"

MINGW_PREFIX="${MINGW_PREFIX:-/ucrt64}"
require_dir "$MINGW_PREFIX"

BUILD_ROOT="${BUILD_ROOT:-$REPO_ROOT/build/windows-ucrt64}"
HAMLIB_SRC_DIR="${HAMLIB_SRC_DIR:-$REPO_ROOT/hamlib-src}"
HAMLIB_BUILD_DIR="$BUILD_ROOT/hamlib-build"
HAMLIB_PREFIX="$BUILD_ROOT/hamlib-prefix"
GPREDICT_BUILD_DIR="$BUILD_ROOT/gpredict-build"
STAGE_ROOT="$BUILD_ROOT/stage"
APP_DIR="$STAGE_ROOT/Gpredict"
ARTIFACT_DIR="$BUILD_ROOT/artifacts"
APP_VERSION="$("$REPO_ROOT/git-version-gen" "$REPO_ROOT/.tarball-version")"
APP_VERSION_SAFE="$(printf '%s' "$APP_VERSION" | tr '/:' '--')"
INSTALLER_NAME="gpredict-${APP_VERSION_SAFE}-windows-x86_64-setup.exe"
PORTABLE_ARCHIVE="gpredict-${APP_VERSION_SAFE}-windows-x86_64-portable.zip"

[[ -d "$HAMLIB_SRC_DIR/.git" ]] || die "missing Hamlib checkout: $HAMLIB_SRC_DIR"
ACTUAL_HAMLIB_COMMIT="$(git -C "$HAMLIB_SRC_DIR" rev-parse HEAD)"
[[ "$ACTUAL_HAMLIB_COMMIT" == "$HAMLIB_EXPECTED_COMMIT" ]] || \
    die "Hamlib checkout is $ACTUAL_HAMLIB_COMMIT, expected $HAMLIB_EXPECTED_COMMIT"

log "cleaning build directories"
rm -rf "$HAMLIB_BUILD_DIR" "$HAMLIB_PREFIX" "$GPREDICT_BUILD_DIR" "$STAGE_ROOT" "$ARTIFACT_DIR"
mkdir -p "$ARTIFACT_DIR" "$APP_DIR"

build_hamlib
build_gpredict
copy_hamlib_binaries
stage_runtime_layout
copy_stage_dependencies

if [[ -d "$APP_DIR/share/glib-2.0/schemas" ]]; then
    log "compiling glib schemas"
    glib-compile-schemas "$APP_DIR/share/glib-2.0/schemas"
fi

make_portable_zip
make_installer

log "artifacts written to $ARTIFACT_DIR"
