# Physics-Engine
Physics Engine built in C++ to simulate real physics

Layout: `engine/` (C++17 core, namespace `physics`), `bindings/` (Emscripten embind),
`web/` (Next.js demo), `scripts/`, `docs/PLAN.md`.

## Native build and tests
```sh
cmake --preset native          # Debug, tests on, -Wall -Wextra -Wpedantic -Werror
cmake --build --preset native -j
ctest --preset native
```

## WebAssembly
Requires [emsdk](https://github.com/emscripten-core/emsdk) (CI pins 3.1.74):
```sh
git clone https://github.com/emscripten-core/emsdk && cd emsdk
./emsdk install 3.1.74 && ./emsdk activate 3.1.74 && source ./emsdk_env.sh
cd /path/to/Physics-Engine && scripts/build-wasm.sh
```
This builds the `wasm` preset and copies `engine.js` + `engine.wasm` to
`web/public/wasm/`. Commit them: CI rebuilds and fails if they differ. Without
them the web app falls back to its TypeScript mock engine.

## Web demo
```sh
cd web
npm ci
npm run dev      # http://localhost:3000
npm run lint && npm test && npm run build   # static export to web/out
```

## Deploy (Vercel)
Import the repo at its root; `vercel.json` sets the build (`cd web && npm ci && npm run build`)
and output (`web/out`), and serves `/wasm/*.wasm` as `application/wasm`. Preview
deploys run per PR, production on `main`. The committed wasm files are deployed
as-is; Vercel does not build C++.
