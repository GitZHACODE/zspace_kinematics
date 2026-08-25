#!/usr/bin/env sh
set -eu

ROOT=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
BUILD_DIR=${BUILD_DIR:-"$ROOT/build"}
CXX=${CXX:-clang++}

if ! command -v "$CXX" >/dev/null 2>&1; then
  echo "C++ compiler '$CXX' was not found." >&2
  exit 1
fi
if ! command -v ninja >/dev/null 2>&1; then
  echo "Ninja was not found." >&2
  exit 1
fi

cmake -G Ninja -S "$ROOT" -B "$BUILD_DIR" -DCMAKE_CXX_COMPILER="$CXX"
cmake --build "$BUILD_DIR" --parallel
ctest --test-dir "$BUILD_DIR" --output-on-failure
