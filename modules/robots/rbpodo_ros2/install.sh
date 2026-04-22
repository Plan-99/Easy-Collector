#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# Build rbpodo C++ library if cmake_pkgs/rbpodo exists in the module
RBPODO_SRC="$SCRIPT_DIR/cmake_pkgs/rbpodo"
if [ -d "$RBPODO_SRC" ]; then
    echo "[rbpodo] Building C++ library..."
    cd "$RBPODO_SRC"
    mkdir -p build
    cd build
    cmake -DCMAKE_BUILD_TYPE=Release ..
    make -j"$(nproc)"
    make install
    echo "[rbpodo] C++ library installed."
else
    echo "[rbpodo] No cmake_pkgs/rbpodo found, skipping C++ build."
fi
