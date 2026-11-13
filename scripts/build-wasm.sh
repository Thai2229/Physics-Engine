#!/usr/bin/env bash
# Builds the embind module with Emscripten and copies it to web/public/wasm/.
# Needs emsdk: either `emcmake` on PATH or $EMSDK pointing at an activated emsdk.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

if ! command -v emcmake >/dev/null 2>&1 && [ -n "${EMSDK:-}" ] && [ -f "$EMSDK/emsdk_env.sh" ]; then
  # shellcheck disable=SC1091
  source "$EMSDK/emsdk_env.sh" >/dev/null
fi
if ! command -v emcmake >/dev/null 2>&1 || [ -z "${EMSDK:-}" ]; then
  echo "error: Emscripten not found. Install emsdk, run 'emsdk activate', set EMSDK (or source emsdk_env.sh)." >&2
  exit 1
fi

cmake --preset wasm
cmake --build --preset wasm

DEST="$ROOT/web/public/wasm"
mkdir -p "$DEST"
cp "$ROOT/build/wasm/dist/engine.js" "$ROOT/build/wasm/dist/engine.wasm" "$DEST/"
echo "Copied engine.js and engine.wasm to web/public/wasm/"
