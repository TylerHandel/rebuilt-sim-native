# REBUILT Sim (native)

The [REBUILT 2026 simulator](https://github.com/TylerHandel/rebuilt-sim-2026) rebuilt as a native Windows
game in Rust, so its physics runs well on low-end computers. It installs once, works offline (the field
and robot models ship with it), and updates itself from this repo's releases.

## Install
Download `RebuiltSim-win-Setup.exe` from the latest [release](https://github.com/TylerHandel/rebuilt-sim-native/releases/latest)
and run it. Every push to `main` publishes a new release. The game checks for it when it starts,
downloads it in the background, and installs it the next time it starts.

## What's here
| | |
|---|---|
| `crates/sim` | The simulation, with no graphics: native [Rapier](https://rapier.rs) physics on every core. Its tests and benchmark run headless. |
| `crates/app` | The game: [Bevy](https://bevyengine.org) drawing, input, the HUD and the update check ([Velopack](https://velopack.io)). |
| `assets/world.json` | The field exactly as the browser game builds it: every collider's shape, pose, friction and groups, the physics settings, and the FUEL. Its `tools/export-native.mjs` exports it. |
| `assets/cad/` | The field and robot models. These aren't committed; `tools/sync-assets.mjs` brings them over. |
| `assets/SOURCE` | The rebuilt-sim-2026 commit both came from. |

## Develop
```
git clone https://github.com/TylerHandel/rebuilt-sim-2026 ../rebuilt-sim-2026   # with `npm ci` in it
cd tools && npm ci && cd ..
node tools/sync-assets.mjs ../rebuilt-sim-2026
cargo run -p app                                          # the game
cargo test -p sim --release                               # the sim's tests
cargo run --release -p sim --example bench                # physics time per simulated second
```
In the game: right-drag turns the camera, the wheel zooms, WASD moves it. Space drops a heap of
FUEL, R resets the field, and F1 shows the field's colliders.

## Performance
The field and its 456 FUEL, the same scenario in each, on one 4-core machine (ms of CPU per
simulated second, lower is better):

| | browser (Rapier WASM) | native |
|---|---:|---:|
| FUEL settling after the start | 205 | 55 |
| at rest | 56 | 13 |
| a 120-FUEL heap dropped | 344 | 139 |

See [PLAN.md](PLAN.md) for what's left to port.
