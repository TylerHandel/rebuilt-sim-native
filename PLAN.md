# Porting plan

The browser game ([rebuilt-sim-2026](https://github.com/TylerHandel/rebuilt-sim-2026), "the JS game"
below) stays the reference. This plan says how its simulation is rebuilt here in Rust, in what
order, and how each piece is proven right before the next one starts.

## In short

1. **The native physics engine already matches the browser's bit for bit.** Pinned to the same
   Rapier and run single-threaded, it steps the field and its FUEL to the same bits as the JS game,
   through a thrown heap of 120 FUEL (`crates/sim/tests/parity.rs`). It is also 3 to 10 times faster.
2. **So the port is checked by exact comparison, not by tuning.** Every module ported from JS is run
   through the same scripted scenario on both sides. The Rust side must produce the same state, to
   the bit, at every step. The first step that differs points at the bug.
3. **Data is exported, logic is ported.** Geometry, configs, tables, meshes and drawn models come
   out of the JS game as files. Only code that runs every step is rewritten by hand.
4. **Each stage ends in a release you can play.** Driving a robot comes first, then one robot that
   intakes and shoots (2910), then the other twelve one at a time.

## What was measured (2026-11-10, 4-core Linux box)

| | JS game (Rapier WASM) | native, `parallel` | native, serial | native, serial + `simd8` |
|---|---:|---:|---:|---:|
| FUEL settling after the start, ms per simulated s | 205 | 55 | **28** | 34 |
| at rest | 56 | 13 | **5.4** | 6.5 |
| a 120-FUEL heap dropped | 344 | 139 | **105** | 128 |

- The JS game's `@dimforge/rapier3d-simd-compat 0.21.0` embeds `parry3d 0.31.1`, the same parry that
  `rapier3d 0.36.1` uses. Both are the same engine generation.
- Serial `rapier3d =0.36.1` with no `simd` or `parallel` feature gives bit-identical FUEL positions,
  rotations, velocities and spins to the JS game after 360 + 360 steps of the scenario in
  `tools/native-parity.mjs` (JS side). This was measured on Linux. The Windows CI job now runs the
  same test, and its first result is not in yet.
- Rust's `std` math and the `libm` crate do **not** match JavaScript's `Math` bit for bit. Out of
  200,000 random inputs each: `libm` matches exactly for `tan asin acos atan atan2 exp expm1`. It
  mismatches for `sin` (672), `cos` (968), `log` (1083), `log1p` (47), `tanh`, `cbrt`, `pow`, and
  `hypot`. `std` mismatches on all of them. Hence `crates/jsmath` (D3), which matches on all.
- In the JS game's 3v3 profile, the game's own logic (mechanisms about 240 ms/s, aiming 40 to 90)
  costs more than Rapier (200 to 250). The Rust port of that logic is where most of the speedup
  will come from, more than the engine.

## Decisions

**D1. Rapier is pinned to `=0.36.1`, serial, no SIMD features.** It is bit-identical to the JS game
and the fastest of the four configurations measured. It leaves the other cores free for mechanisms
and drawing. Both sides upgrade Rapier together or not at all.

**D2. The sim is ported to be bit-identical to the JS game, and that is the acceptance test.** A
rigid-body sim is chaotic, so "close" can't be told apart from "wrong" after a few seconds. Exact
can. The JS code is already written this way (its `Reaction` class emulates Rapier's f32 math, and
`tools/samecheck.mjs` holds it to identical replays), so the port follows an existing discipline.
Optimizations that change the bits come only after a module's parity gate has passed. From then on
they are checked against the Rust build tagged at parity.

**D3. JavaScript's math functions come from V8's own source (`crates/jsmath`, done).** Sim code
calls `jsmath::sin`, never `f64::sin`. `crates/jsmath` compiles V8's `src/base/ieee754.cc`
(12.4.254.21, the V8 in Node 22) unmodified, with shims for the four V8 headers it includes and
floating-point contraction off. Node's V8 is built without `V8_USE_LIBM_TRIG_FUNCTIONS`, so `sin`
and `cos` are the fdlibm ones. `round`, `sign`, `max`, `min`, `hypot`, `imul` and `fround`
follow V8's builtins (`math.tq`). `tools/jsmath-vectors.mjs` checks 31 functions on a million inputs
each (special values, every exponent from -60 to 60, near multiples of pi/2, small integer and
half-integer exponents, any bits). Every result matches node's bit for bit
(`crates/jsmath/tests/vectors.rs`).

**D3b. The engine's own f32 math is `libm`'s, as in the browser (`crates/sim/src/libm_f32.rs`).**
Rapier and parry call `f32::sin`, `exp`, `atan2` and others directly. Compiled to WebAssembly those
are Rust's `libm`; natively they're the platform's C library, which differs in the last bit now and
then. The sim defines the C symbols (`sinf`, `cosf`, `atan2f`...) as `libm`'s, so the native engine
lands on the same functions. Found by the `drive-2910` script: identical for 44 steps, then a
wheel's contact impulse one bit off. (Rapier's `enhanced-determinism` feature is not the fix: it
also turns glam's SIMD off and changes parry's maps, and the FUEL then differ from the browser's.)

**D4. Data is exported from the JS game. Logic is ported by hand.** See the table below. Exported
data stays correct when a robot is retuned in JS: re-run `tools/sync-assets.mjs`. Ported logic is
tracked in a ledger (D10).

**D5. The `sim` crate is plain Rust with no ECS, and mirrors the JS files one to one.** One Rust
module per JS file, with the same function names and statement order, and a comment giving the JS
line it came from. This makes the port reviewable against the JS and makes later JS changes easy
to carry over. Bevy's ECS is used only in `app`, for drawing.

**D6. Shared objects become indices.** JS objects that point at each other (robot, FUEL, match,
physics) become one `World` struct that owns `physics`, `fuel: Vec<Ball>`, `robots: Vec<Robot>`
and `match_`. A FUEL is a `BallId(u32)`, a robot a `RobotId(u8)`, a body a Rapier handle.

**D7. A mechanism's step is a pure function over snapshots.** `Mechanism::step` takes the frame's
state (pose, velocities, center of mass, inverse mass, world inverse inertia) and its FUEL's states
as plain values, and returns the new FUEL velocities (and positions when sub-stepped) and the
frame's new velocities. It never touches Rapier. This matches what the JS already does (it reads
everything first and writes once at the end), it makes mechanisms safe to run on other threads,
and it makes a single step replayable in a test.

**D8. Threading: Rapier serial, mechanisms of separate robots in parallel, the sim on its own
thread.** The JS game's `preStepRobots` (game.js) already finds the robots that share no FUEL with
another and steps them in three phases. In Rust the middle phase runs under `rayon` over those
robots' snapshots, and results are applied in robot order. The results are the same bits as serial.
The sim runs at 120 Hz on its own thread and hands the app a snapshot per frame, using the JS
game's existing seam (`robot.visualState()` to `applyVisual()`).

**D9. Drawn robot models are exported as GLB from the browser, not re-modeled.** `robotModels.js`
is 4,055 lines of procedural three.js. A Playwright script builds each model in a real browser,
names every part its `anim` function moves, and writes a GLB. Only the `anim` functions are ported,
10 to 30 lines per robot. Custom shaders (FUEL skin, spin blur, polycarbonate) and the hopper nets
don't export. They are redone in Bevy later (M6) and start as plain materials.

**D10. A port ledger.** `PORT.md` lists each JS file, its Rust module, the JS commit it was ported
at, and its parity scenario. `tools/port-status.mjs` prints the JS files that changed since the
commit they were ported at, so a session can re-port just those diffs.

**D11. Only code that the physical robots reach is ported.** Thirteen robots are built of physical
mechanisms: 2910, 4414, 8793, 1678, 1690, 341, 1706, 581, 1114, 604, 6328, 2056, 7769. Four use the
older emulated intake and hopper: 971, 4946, 3928, 4930. The emulated path (`hopper.js` packing
and large parts of `robot.js`) is not ported. Run the parity scenarios under V8 coverage
(`NODE_V8_COVERAGE`) to see which functions are reached. **This leaves those four robots out of
the native game. It is the owner's call (see "Open questions").**

## How parity is checked

Three pieces, built first (M2a):

- **Scripts** (`parity/scripts/*.json`, here): a seed, the robots with their start poses and
  preload, a list of timed commands (`vx vz omega intake shoot pass outtake`), a step count, and
  how often to take a digest.
- **`tools/native-trace.mjs`** (in the JS game): runs a script with the real game classes
  (`createWorld`, `createGame` with `driver: 'external'`, `stepGame`), and writes digests to
  `assets/parity/<script>.json`. `tools/sync-assets.mjs` runs it for every script. The files are
  committed, and CI checks they match what `assets/SOURCE` produces.
- **`crates/sim/tests/parity_*.rs`**: runs the same script and compares.

A digest is FNV-1a 64 over raw bits, as in `crates/sim/src/parity.rs`. It is taken after each phase
of a step (commands, each robot's pre-step, FUEL pre-step, `physics.step`, FUEL post-step, each
robot's post-step, rules) and covers:

- every body: position, rotation, velocity, spin (f32 bits);
- every FUEL: its state, owner, and flags;
- every robot: swerve module angles and speeds, every shaft's speed and command, arm, hood and
  turret positions, stored count, stats (f64 bits);
- the number of random draws so far.

On a mismatch, the test reports the first step, phase and field that differ. Both sides take
`--dump <step>` to print full state at that step, and for mechanisms the contact list (wheel or
surface name, FUEL index, depth, normal, impulses).

The workflow for every ported module: write or pick its script, generate the JS digests, port,
run, fix until identical, record it in `PORT.md`. Don't start the next module on top of one that
isn't identical.

## Rules for an exact port

- **Numbers.** JS numbers are `f64`. Values read from Rapier are `f32` widened to `f64`. Values
  written to Rapier are rounded with `as f32`, which rounds like the WASM boundary does. Keep
  `Math.fround` calls as `as f32 as f64`.
- **Order of operations.** Keep it. `a + b + c` stays `(a + b) + c`. Don't factor, don't use
  `mul_add`, don't replace `x / y` with `x * (1.0 / y)`. Keep `Math.hypot(a, b)` as
  `jsmath::hypot`, not `sqrt(a*a + b*b)`.
- **Collections.** A JS `Map` or `Set` iterates in insertion order, and deleting keeps the order of
  the rest. Use `indexmap` with `shift_remove`, never `HashMap` iteration or `swap_remove`. JS
  `sort` is stable: use `sort_by`, not `sort_unstable_by`.
- **Random draws.** `Math.random` in headless runs is `mulberry32(seed)` (tools/headless.mjs). Port
  it, and make every draw in the same order. Sim draws are in `match.js`, `robot.js` (shot period,
  feed lane), `field.js:288` and `util.js`. three.js draws on `Math.random` when it creates
  objects, so the trace tool reseeds after construction and counts draws per step. If drawing code
  is found to draw during a step, give it its own generator in the JS game.
- **Rapier calls.** For each Rapier JS call the sim makes, check what the wrapper does before
  choosing the Rust call (for example, whether `setRotation` normalizes). The surface is small:
  `setLinvel setAngvel setTranslation setRotation setTranslationWrtParent setRotationWrtParent
  setCollisionGroups setSolverGroups setEnabled setBodyType setNextKinematicTranslation
  applyImpulse applyImpulseAtPoint addForce addTorque resetForces resetTorques wakeUp sleep
  castRay intersectionsWithShape contactPair contactPairsWith effectiveInvMass
  effectiveWorldInvInertia worldCom`. Each gets a line in a parity script.
- **Creation order.** Bodies and colliders are created in the JS game's order (FUEL first, then
  robots, in `createGame`'s order). Rapier's results depend on handle order.
- **Generators.** A JS `*preSteps` with one `yield` becomes two methods, `pre_mech` and
  `post_mech`, with the state the generator kept across the yield stored on the struct.
- **JSON.** `serde_json` has `float_roundtrip` on, so exported numbers load as the exact doubles
  JS wrote.
- **Environment switches.** JS reads tuning switches from `process.env` (`MU_FUEL`, `TWIST`,
  `CARRIED`, `FASTSUB`, `CURVED`, `PHASED`, `HOODSUB`). Port only the default path of each.

## Exported or ported

| From the JS game | How it gets here |
|---|---|
| Field colliders, physics settings, FUEL setup (`field.js`, `fuel.js` setup) | Exported: `assets/world.json` (done) |
| Rapier integration parameters, every one the JS API exposes | Exported into `world.json`. A test asserts the Rust defaults equal them |
| Robot configs (`robotConfigs.js` `ROBOTS`, `CLIMBER_OPTIONS`, `AUTO_ROUTINES`) | Exported: `assets/robots.json` |
| Each robot's rigid body as built (`robot.js _createBody`: hull points, masses, groups) | **Ported**, and checked against an export of the built colliders |
| Each robot's mechanisms as built (`mech/r*.js`: shafts, motors, trains, wheels, surfaces, blocks, and `mech/path*.js` meshes) | Exported: `assets/mech/<robot>.json`, written by serializing the built `Mechanism` objects |
| The per-step parts of `mech/r*.js` (`placeIntake`, `placeHood`, `hoodElevation`, shot and pass table lookups) | **Ported.** The tables themselves are exported |
| `mech/mechanism.js` (foam, motors, shafts, trains, wheels, surfaces, contact solve, reaction) | **Ported** |
| `mech/physical.js` and each `mech/phys*.js` (controls, which FUEL are its, shots) | **Ported** |
| `drivetrain.js`, the drive part of `robot.js`, `physics.js` (TRENCH arm rule) | **Ported** |
| `fuel.js` (states, own and unown, launch, release, flight, HUB and return handling) | **Ported** |
| `ballistics.js`, `localize.js`, `match.js`, `rules.js`, `humanPlayer.js` | **Ported** (M4) |
| `auto.js`, `teamAutos.js`, `auto6328.js`, `opponent.js`, `nav.js`, `nn/` | **Ported**. Their paths, tables and network weights are exported (M5) |
| Field CAD, robot CAD parts | Copied and dequantized: `assets/cad/` (done) |
| Drawn robot models (`robotModels.js`) | Exported as GLB from the browser. `anim` functions ported |
| Menus, HUD, cameras, sound, venue, LEDs (`ui.js`, `main.js`, `cameras.js`, `sound.js`, `venue.js`, `leds.js`) | Rebuilt in Bevy (M4, M6) |

## The `sim` crate

```
crates/jsmath/      JavaScript's Math, exactly (D3)
crates/sim/src/
  rng.rs           mulberry32
  constants.rs     js/constants.js
  world.rs         the exported data's types
  physics.rs       js/physics.js: the Rapier world, groups, the TRENCH arm rule
  fuel.rs          js/fuel.js
  drivetrain.rs    js/drivetrain.js
  robot.rs         js/robot.js (the physical robots' paths only, D11)
  mech/
    mechanism.rs   js/mech/mechanism.js
    physical.rs    js/mech/physical.js: trait Physical + the shared state
    r2910.rs ...   one per robot: phys<robot>.js and the per-step parts of r<robot>.js
  game.rs          js/game.js: World, create, step
  parity.rs        digests
```

- `trait Physical` carries what `PhysicalRobot` subclasses override: `where_`, `shooter_balls`,
  `pre_mech`, `post_mech`, `post_step`, `capacity`, and the optional hooks `robot.js` asks for
  (`arm_turn`, `hood_turn`, `turret_step`, `aim_psi`, `pass_target`, `feed_push`, `ext_now`,
  `lift_now`, `load_top`, `mech_pose`). Optional JS hooks become trait methods returning `Option`.
- `robot.rs` and a robot's `Physical` both need `&mut World`. Take the robot out of
  `world.robots[i]` for the duration of its step (`std::mem::take` or split borrows), as the JS
  steps one robot at a time anyway.
- `Mechanism` holds its shafts, wheels and surfaces in `Vec`s and refers to them by index. A
  `Surface` keeps the JS grid and tree (`_buildTree`, `_near`, `touches`) and its per-FUEL cache.

## Milestones

Each lettered step ends with its parity gate green on CI and, where marked, a release.

### M1: foundation (done)
Field and FUEL in native Rapier, tests, benchmark, Bevy app, bundled models, Windows installer
with auto-update, engine parity test.

### M2: a robot you can drive

**M2a. The parity harness.**
- `jsmath` with node's test vectors (D3). Done: 31 functions, a million inputs each, all identical.
- `rng.rs`, `constants.rs`. Done: mulberry32 and the constants (generated from the export, JS
  names kept: `FUEL::radius`) checked against the JS game.
- Script format, `tools/native-trace.mjs` (JS game), `parity_*.rs` runner, `--dump`. Done: the JS
  game's `stepGame` takes an observe-only probe (identical replays); three scripts (`fuel-start`,
  `fuel-launch`, `drive-2910`) and their traces; the Rust types and `first_mismatch`. Each script
  runs here once its modules are ported.
- Export all integration parameters. Export `robots.json`. Done.
- `PORT.md` and `tools/port-status.mjs`. Done.
- Gate: the existing engine scenario, re-expressed as a script, is identical on Linux and Windows.
  If Windows differs, find which call differs. If it is inside Rapier, gate on Linux only and say
  so in `PORT.md`.

**M2b. FUEL.** `fuel.rs`: states, `stage`, `launch`, `own`, `unown`, `release`, `pre_step`,
`post_step`, flight (drag and Magnus), and enough of `update` for FUEL leaving the field to come
back. Gate: a script that launches FUEL with set velocities and spins at walls, the HUBs and the
floor. **Done:** `fuel.rs`, `game_match.rs`, `human_player.rs` (all but throwing), `field.rs`, the
game `World` (createWorld, stepGame, frameGame without robots). `fuel-start` (the match's FUEL
through the pre-match into AUTO) and `fuel-launch` (FUEL thrown spinning at walls, both HUBS, a BUMP,
into a CORRAL and out of the FIELD and returned) are identical to the JS game after every part of
every step.

**M2c. The robot body and drive.** In progress: `drivetrain.rs` (Drivetrain, SwerveDrive),
`mech/drives.rs` (Motor, Shaft, Train, velLoop), `robot.rs` (the body as built, spawn, the drive,
the wall assist, wheels on the floor, tip damping, the hopper sliding out, the intake block), the
TRENCH arm rule, and 2910's intake pivot (the arm decides when its hopper slides out, which moves
the body's mass). `drive-2910` (driving, strafing into the guardrail, spinning on it, under the TRENCH
arm and back) is identical to the JS game for all 600 steps. Still to do here: folding and sliding
intakes and lids (the other robots' bodies), two robots meeting (`rules.js`), a BUMP. `drivetrain.rs`; `robot.rs` constructor, `_createBody`, `reset`,
`spawn`, `_preStepDrive`, `_alongWalls`, `_wheelsDown`, tip damping, the sliding hopper and top
colliders; `physics.rs` `passesUnder`. For now a robot has no mechanisms and holds no FUEL.
- Gate 1: each of the 13 robots' built colliders equal the JS export (points, masses, center of
  mass, inertia).
- Gate 2: drive scripts. Accelerate, brake, spin, strafe; drive into a wall square and at an
  angle; cross a BUMP; pass under a TRENCH arm with millimeters to spare; plough through FUEL; two
  robots pushing each other.

**M2d. Drawing and input.** Sim thread and snapshot (D8). Model export (D9) for the first robots.
`anim` for swerve modules. Keyboard and gamepad to commands (`input.js`), field-relative drive.
Chase and driver-station cameras. Graphics presets for slow machines (render scale, shadows off,
no MSAA), in a settings menu.
**Release: pick one of the 13 robots and drive it around the field, pushing FUEL.**
*Started:* the app runs the ported game (sim::game::World) with 2910 in TELEOP, drawn as its
colliders, driven by keyboard or controller (input.js and driverCommand: field-relative from the
camera), the camera following it; Backspace or View starts a new match. HUMAN PLAYERS are off until
their throws are ported (M3). Still to do: the CAD models (D9), the sim thread, the cameras, presets.

### M3: mechanisms

**M3a. The mechanism core.** `mech/mechanism.rs`, whole file, and the `assets/mech/*.json`
export. Gate: `tools/mechlab2910.mjs`'s `rollers`, `path` and `shooter` set-ups as scripts (a
mechanism with no robot: FUEL on a treadmill, FUEL fed to the shooter), identical at every step.
This is the hardest single port. Expect most of the time to go to `_contacts`, `_solve`,
`velLoop` and `Surface.touches`.

**M3b. The first physical robot: 2910.** `mech/physical.rs`, `mech/r2910.rs`, and the parts of
`robot.rs` a physical robot uses: `_intakePhys`, `_hopper`, `_shooter`, `_target`, `_fire`,
`postStep`, with `ballistics.rs` and `localize.rs`. Gate: scripts for intake (drive through the
NEUTRAL ZONE line, as `tools/intake-test.mjs`), fill to full, and a dump at the HUB from three
distances. Add 2910's `anim`.
**Release: 2910 intakes, fills and shoots.** Scoring isn't counted yet.

**M3c. The other twelve, one per step, each with the same three scripts and a release.**
Order: 4414, 8793 (with 2910, the default blue alliance), 1678, 1690, then 7769, 341, 1114, 1706,
6328, 604, 2056, 581. Each is `phys<robot>.js` (220 to 390 lines) plus the per-step parts of
`r<robot>.js`. New mechanism features a robot needs (`turretDrive.js` for turrets, 1706's twin
turrets, `shot4414.js`, nets) are ported with the first robot that uses them.

**M3d. Mechanisms in parallel, and the speed check.** `preStepRobots`'s phases under `rayon` (D8),
gated by the same scripts (same bits as serial). A native `perf3v3` (as `tools/perf3v3.mjs`): six
physical robots, autos off, scripted driving. **Target: a 3v3 costs under 150 ms of one core per
simulated second on the reference box**, against roughly 600 in the browser. If it's over, profile
before changing anything, and keep parity.

### M4: the match
`match.rs`, `rules.rs`, `humanPlayer.rs`, HUB scoring and FUEL returns, the timer, climbing
(`_climbStep`), HUD, menus (robot and slot pickers reading the JS game's settings names), practice
/ 1v1 / 3v3. Gate: a full scripted match's score and digests. **Release: a complete match with
human drivers only.**

### M5: AI and autos
`auto.rs`, team autos, `opponent.rs`, `nav.rs`, the neural policy (`nn/`). Gate: a full 3v3 of AI
robots, identical to the JS game's headless `tools/match.mjs` for the same seed. **Release: the
full game.**

### M6: feel and low-end graphics
Sound (Cheesy Arena audio fetched at runtime, never bundled), rumble, LEDs, venue, FUEL skins and
squash, nets, spin blur, and graphics presets (render scale, shadows off, no MSAA).

## Risks

- **Windows matching Linux and the browser: confirmed so far.** The Windows CI job runs the engine
  parity test, the jsmath vectors, and the trace regeneration (Windows Node making the same traces
  as the committed Linux ones), and all are identical (runs 3 and 5, 2026-11-10). Each new parity
  script is checked there the same way.
- **`jsmath` on Windows.** It's compiled V8 source, so it matches on any compiler that doesn't
  contract or reorder floating point (MSVC `/fp:precise`, the default). The vector test runs in
  the Windows CI job to prove it.
- **Low-end GPUs.** Bevy needs DirectX 12 or Vulkan. An old laptop that ran the browser game in
  WebGL may run Bevy worse, or not at all. Test v0.1 on the weakest target machine now (see below),
  before more is built on it.
- **Size.** The sim logic to port is roughly 15,000 lines of JS: `robot.js` 1,900, `mechanism.js`
  1,160, thirteen robots about 5,500, AI and autos about 2,800, the rest about 3,500. The parity
  method keeps each piece checkable, but it is many sessions of work.
- **The JS game changing.** Robot tuning is frozen for the port (owner's answer 3), but fixes still
  land there. Every JS change after a file's ported commit has to be carried over, and the ledger
  (D10) finds them.

## The owner's answers (2026-11-10)

1. **Leave 971, 4946, 3928 and 4930 out** of the native game (D11).
2. **No low-end computer to test on yet.** So the HUD's frame-rate and physics numbers are the only
   check for now, and the graphics presets (render scale, shadows off, no MSAA) move from M6 to
   M2d, so a slow machine has settings to fall back on from the first playable release.
3. **The JS game's robots are frozen during the port** where that's better for it. Robot tuning in
   the JS game waits, so ported modules don't have to be redone. Fixes the port turns up in the JS
   game (a bug found by a mismatch) are still made there first, then carried over.
