# Physics Engine — Build Plan

A 2D rigid-body physics engine written in modern C++ (C++17), compiled to
WebAssembly with Emscripten, and showcased in an interactive web demo deployed
on Vercel.

## 1. Key decisions

| Decision | Choice | Why |
|---|---|---|
| Dimension | **2D** first | A multi-joint arm, chains, ragdolls and stacks all read clearly in 2D; 3D can come later on the same architecture. |
| Integrator | Semi-implicit (symplectic) Euler, fixed timestep (1/120 s default) with sub-stepping | Stable, energy-friendly, industry standard (Box2D, Chipmunk). |
| Constraint solver | Sequential impulses (projected Gauss-Seidel) with warm starting + Baumgarte / soft-constraint position correction | Handles joints, limits, motors and contacts in one framework. |
| Collision | Broadphase: sort-and-sweep on AABBs. Narrowphase: circle-circle, circle-polygon, polygon-polygon (SAT + clipping for 2-point manifolds) | Enough for stacks, ragdolls and arms hitting objects. |
| Units / axes | Meters, kilograms, seconds, radians; **y-up** in the engine (renderer flips) | Avoids the classic "pixels as meters" instability. |
| Build | CMake ≥ 3.20, native + Emscripten presets | One source tree, two targets. |
| Tests | GoogleTest via `FetchContent` | Standard, works with CTest. |
| WASM bindings | Emscripten `embind`, `MODULARIZE=1 EXPORT_ES6=1` | Clean ES-module import from the web app. |
| Web app | Next.js (App Router) + TypeScript, static export, Canvas 2D renderer | Deploys to Vercel with zero config; Canvas 2D is plenty fast for hundreds of bodies. |
| Deploy | GitHub Actions builds the WASM, then Vercel builds the static site | Vercel's build image has no Emscripten; installing emsdk per deploy is slow. Built `engine.js`/`engine.wasm` are committed under `web/public/wasm/` and CI checks they match the sources. |

## 2. Repository layout

```
engine/                     C++ core library (no platform deps)
  include/physics/          public headers
    math.hpp                Vec2, Mat22, Rot, Transform, helpers
    body.hpp                RigidBody (static / kinematic / dynamic)
    shape.hpp               Circle, Polygon (convex, ≤ 8 verts), Box helper
    joint.hpp               Joint base + RevoluteJoint, DistanceJoint, MouseJoint
    collision.hpp           AABB, Manifold, narrowphase functions
    broadphase.hpp          Sort-and-sweep
    contact_solver.hpp      Contact constraints
    world.hpp               World: owns bodies/joints, step(), queries
  src/                      implementations
  tests/                    GoogleTest suites (one per header)
  CMakeLists.txt
bindings/
  wasm_bindings.cpp         embind wrapper exposing the API in §4
  CMakeLists.txt
web/                        Next.js app (static export)
  public/wasm/              built engine.js + engine.wasm (committed)
  src/engine/               TS adapter: types, WASM loader, mock engine
  src/scenes/               scene definitions (JSON-serialisable)
  src/render/               Canvas 2D renderer + debug overlays
  src/app/                  pages + editor UI
scripts/build-wasm.sh       emsdk-based build, copies output to web/public/wasm
CMakeLists.txt              top-level; CMakePresets.json (native, wasm)
.github/workflows/ci.yml    native build + tests, wasm build + diff check, web lint/test/build
vercel.json                 rootDirectory = web, outputDirectory = out
```

## 3. Engine milestones (each one is test-first)

### M1 — Math & bodies
- `Vec2`, `Rot` (sin/cos), `Transform`, `Mat22`, cross products (vec×vec, scalar×vec), length/normalise with epsilon guards.
- `RigidBody`: position, angle, linear/angular velocity, force/torque accumulators, mass/inertia and inverses (static ⇒ inverse mass 0), linear/angular damping.
- Mass properties computed from shapes (circle, convex polygon) and density.
- **Tests:** vector algebra identities, polygon area/centroid/inertia vs. analytic box values, static bodies never move.

### M2 — World & integration
- `World { gravity, bodies, joints, step(dt, velocityIterations, positionIterations) }`.
- Fixed-step accumulator helper (`World::advance(frameDt)` → N sub-steps) so rendering frame rate doesn't change the physics.
- Semi-implicit Euler: integrate velocities → solve constraints → integrate positions → position correction.
- Stable integer handles (`BodyId`, `JointId`) with generation counters so JS can hold IDs safely after removals.
- **Tests:** free fall matches `y = ½gt²` within tolerance; zero gravity conserves momentum; damping decays velocity.

### M3 — Joints (needed for the arm demo)
- `RevoluteJoint` (pin between two bodies at a world anchor) with:
  - lower/upper angle **limits**,
  - **motor** (target speed + max torque),
  - optional spring (frequency, damping ratio) for "soft" joints.
- `DistanceJoint` (rod / spring rope).
- `MouseJoint` (soft spring pulling a body point toward the cursor) — used for dragging in the demo.
- Warm starting of accumulated impulses.
- **Tests:** simple pendulum period ≈ 2π√(L/g) (±2 %); joint anchor drift < 1 mm after 10 s of a 5-link chain; limits are respected; motor reaches target speed; energy of an undamped pendulum stays within ±5 % over 30 s.

### M4 — Collision & contacts
- Shapes attached to bodies with friction, restitution, density, collision filter (category/mask bits — lets arm links ignore each other).
- AABB + sort-and-sweep broadphase.
- Narrowphase: circle–circle, polygon–circle, polygon–polygon (SAT, reference/incident face clipping), contact IDs for warm starting.
- Contact constraints: normal impulse (non-penetration, restitution with velocity threshold), Coulomb friction, accumulated impulse clamping.
- Optional sleeping of resting bodies.
- **Tests:** each narrowphase pair (overlap, touching, separated); a 10-box stack settles and stays standing for 10 s; a ball with restitution 1 bounces to ~the same height; friction stops a box on a slope above the friction angle.

### M5 — Extras (stretch)
- Two-bone / CCD **inverse kinematics** helper so the arm can reach for the cursor via joint motors.
- Ray casts and point queries (for picking bodies with the mouse).
- Deterministic replay (same input ⇒ same output; useful for sharing scenes).

## 4. WASM API contract (shared by the bindings and the web app)

The web app only talks to this interface (`web/src/engine/types.ts`). A pure-TS
`MockEngine` implements it so UI work can proceed before the WASM build exists.

```ts
interface PhysicsWorld {
  setGravity(x: number, y: number): void;
  addBody(def: BodyDef): number;            // returns body id
  removeBody(id: number): void;
  setBodyTransform(id: number, x: number, y: number, angle: number): void;
  setBodyVelocity(id: number, vx: number, vy: number, w: number): void;
  addRevoluteJoint(def: RevoluteJointDef): number;
  updateRevoluteJoint(id: number, patch: Partial<RevoluteJointDef>): void;
  addDistanceJoint(def: DistanceJointDef): number;
  removeJoint(id: number): void;
  createMouseJoint(bodyId: number, x: number, y: number, maxForce: number): number;
  setMouseTarget(jointId: number, x: number, y: number): void;
  queryPoint(x: number, y: number): number;  // body id or -1
  step(frameDt: number): void;               // fixed-step internally
  /** Float32Array view: [x, y, angle] per body, in getBodyIds() order. Zero-copy into WASM memory. */
  getBodyStates(): Float32Array;
  getBodyIds(): Int32Array;
  getJointAnchors(): Float32Array;           // [ax, ay, bx, by] per joint
  getContacts(): Float32Array;               // [px, py, nx, ny] per contact point (debug overlay)
  getStats(): { bodies: number; contacts: number; stepMs: number; kineticEnergy: number; potentialEnergy: number };
  destroy(): void;
}
```

`BodyDef = { type: 'static'|'dynamic'|'kinematic', x, y, angle, shape: {kind:'circle', radius} | {kind:'box', w, h} | {kind:'polygon', vertices}, density, friction, restitution, linearDamping, angularDamping, collisionGroup }`.
`RevoluteJointDef = { bodyA, bodyB, anchorX, anchorY, enableLimit, lowerAngle, upperAngle, enableMotor, motorSpeed, maxMotorTorque, collideConnected }`.

## 5. Web demo

### Main showcase: editable robotic arm
- Arm = fixed base + N links (1–8) connected by revolute joints.
- **Sidebar editor** per link: length, width, mass/density; per joint: limits on/off + range, motor on/off + speed/torque, spring stiffness.
- Add / remove link buttons; global gravity slider, time-scale slider, damping.
- **Direct manipulation:** drag any link with the mouse (mouse joint), drag joint handles in edit mode (paused) to change link lengths.
- **IK mode** (stretch): end-effector follows the cursor using joint motors.
- Play / pause / single-step / reset; scene serialised to the URL hash so a configuration can be shared as a link.

### Other ways to show off the engine (recommended additions)
1. **Scene gallery** with one-click presets: double pendulum (chaos, with trail drawing), Newton's cradle (restitution), chain/rope bridge, box pyramid stacking (solver stability), ragdoll you can throw around, Galton board.
2. **Debug overlays** toggles: AABBs, contact points + normals, joint anchors, velocity vectors, center of mass.
3. **Live charts**: kinetic/potential/total energy over time — the most convincing proof that the integrator is sound.
4. **Solver comparison**: side-by-side canvases with different iteration counts or timesteps, showing drift/jitter.
5. **Performance HUD**: body count, contact count, step time in ms (WASM vs. a JS baseline is a good talking point).

### Rendering
- Canvas 2D, `requestAnimationFrame`, physics advanced by real elapsed time (clamped) via the fixed-step accumulator.
- World→screen camera with pan/zoom (wheel + drag on empty space); HiDPI aware.
- Renderer reads the zero-copy `Float32Array` each frame — no per-body JS↔WASM calls.

## 6. CI / deploy

1. **ci.yml — native job:** configure with the `native` preset, build, run `ctest`, enable `-Wall -Wextra -Werror` and ASan/UBSan in a debug job.
2. **ci.yml — wasm job:** `mymindstorm/setup-emsdk` (pinned version), run `scripts/build-wasm.sh`, fail if `web/public/wasm/` differs from the committed files.
3. **ci.yml — web job:** `npm ci`, `npm run lint`, `npm test` (Vitest: adapter + scene serialisation + mock engine), `npm run build`.
4. **Vercel:** project root `web/`, framework Next.js, `output: 'export'`; serve `.wasm` with `Content-Type: application/wasm` (Vercel does this by default). Preview deploys on every PR, production on `main`.

## 7. Order of work and agent ownership

| Step | Work | Agent | Depends on |
|---|---|---|---|
| 1 | Plan (this doc), repo layout, API contract | main session | — |
| 2 | CMake scaffold, presets, GoogleTest; M1 → M3 test-first | `tdd-guide` | 1 |
| 3 | Next.js app, TS adapter + `MockEngine`, renderer, arm editor, scene gallery | `nextjs-developer` | 1 (runs in parallel with 2) |
| 4 | M4 collision & contacts, test-first | `tdd-guide` | 2 |
| 5 | embind bindings, `build-wasm.sh`, WASM loader in web app, CI workflow, `vercel.json` | `general-purpose` (no C++/build specialist agent exists) | 2, 3 |
| 6 | Strict TS pass on the adapter/types if needed | `typescript-pro` | 5 |
| 7 | Fresh-context review of the whole diff; fix confirmed findings | `code-reviewer` | 4, 5 |
| 8 | Security review — **skipped**: no auth, sessions, PII, uploads, or input reaching SQL/shell/HTTP (URL-hash scene JSON is parsed and validated client-side only) | — | — |
| 9 | Connect the repo to Vercel (one-time, needs the owner's account) and merge | human | 7 |

## 8. Definition of done
- `ctest` green natively, including the pendulum-period, chain-drift and stack-stability tests.
- `npm run lint && npm test && npm run build` green in `web/`.
- Demo loads the real WASM engine, the arm is editable (add/remove links, limits, motors, drag), at least three gallery scenes run at 60 fps with 200 bodies.
- Vercel preview URL attached to the PR.

## 9. Open questions (assumptions made)
- **2D vs 3D?** Assumed 2D. 3D (Three.js renderer, GJK/EPA collision) is a much larger follow-up.
- **Committed WASM artifacts vs. building on Vercel?** Assumed committed + CI-verified, to keep Vercel builds fast and free of emsdk.
- **Next.js vs. a plain Vite app?** Assumed Next.js to match the team's tooling; a Vite SPA would be equally valid since everything is client-side.
