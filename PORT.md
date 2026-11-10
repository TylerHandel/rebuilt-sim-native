# Port ledger

What of the JS game ([rebuilt-sim-2026](https://github.com/TylerHandel/rebuilt-sim-2026)) is here,
and the JS commit it was taken from. `node tools/port-status.mjs ../rebuilt-sim-2026` lists the JS
files that changed since (their diffs are what's left to carry over). See PLAN.md for how a module
is ported and proven.

| JS | Here | At | How | Proven by |
|---|---|---|---|---|
| `js/constants.js` (values) | `assets/constants.json` -> `crates/sim/src/constants.rs` | d867bd4 | exported | `tests/basics.rs` |
| `js/constants.js` (`other sideSign fw groups trenchArmParts`) | `crates/sim/src/constants.rs` | d867bd4 | ported | `tests/basics.rs` |
| `js/robotConfigs.js` (`ROBOTS CLIMBER_OPTIONS AUTO_ROUTINES ROBOT_ORDER`) | `assets/robots.json` | d867bd4 | exported | (read in M2c) |
| `js/field.js` (colliders), `js/fuel.js` (FUEL bodies), `js/physics.js` (settings) | `assets/world.json` -> `crates/sim/src/lib.rs` | d867bd4 | exported | `tests/parity.rs`, `tests/basics.rs` |
| `tools/headless.mjs` (`mulberry32`) | `crates/sim/src/rng.rs` | d867bd4 | ported | `tests/basics.rs` |
| V8's `Math` (Node 22) | `crates/jsmath` | V8 12.4.254.21 | compiled from V8's source | `crates/jsmath/tests/vectors.rs` |
| `tools/native-trace.mjs` (digest v1, scripts) | `crates/sim/src/parity.rs` | cd3b5ff | ported | `tests/parity_scripts.rs` |
| `js/fuel.js` (all but drawing: `sync`, shadows, wear, and the emulated robots' `captor`) | `crates/sim/src/fuel.rs` | cd3b5ff | ported | `fuel-start`, `fuel-launch` |
| `js/match.js` (all but the HUD's `hubNextChange hubLightMode clockText phaseLabel`) | `crates/sim/src/game_match.rs` | cd3b5ff | ported | `fuel-start`, `fuel-launch` |
| `js/humanPlayer.js` (all but `throwOne`, which needs the shot solver: M3) | `crates/sim/src/human_player.rs` | cd3b5ff | ported | `fuel-launch` |
| `js/field.js` (`hubCenter allianceLineX neutralSlots depotSlots`, the OUTPOSTS' info exported) | `crates/sim/src/field.rs` | cd3b5ff | ported | `fuel-start` |
| `js/ballistics.js` (`liftCoef magnus`) | `crates/sim/src/ballistics.rs` | cd3b5ff | ported | `fuel-launch` |
| `js/game.js` (`stepGame frameGame`, no robots yet), `tools/headless.mjs` (`createWorld`) | `crates/sim/src/game.rs` | cd3b5ff | ported | `fuel-start`, `fuel-launch` |
| `js/util.js` (`rand clamp approach`) | `crates/sim/src/fuel.rs`, `robot.rs` | cd3b5ff | ported | `fuel-launch`, `drive-2910` |
| `js/drivetrain.js` (`MOTORS Drivetrain SwerveDrive`; not the robot cards' `force stats`) | `crates/sim/src/drivetrain.rs` | cd3b5ff | ported | `drive-2910` |
| `js/mech/mechanism.js` (`Motor Shaft Train velLoop`) | `crates/sim/src/mech/drives.rs` | cd3b5ff | ported | `drive-2910` (2910's pivot) |
| `js/robot.js` (`_createBody` for an intake block, `reset spawn preStep _preStepDrive _alongWalls _wheelsDown _hopper _stepTopColliders`, `postStep`'s pose) | `crates/sim/src/robot.rs` | cd3b5ff | ported | `drive-2910` |
| `js/robotConfigs.js` (`frameShape offsetShape modulePoints`) | `crates/sim/src/robot.rs` | cd3b5ff | ported | `drive-2910` |
| `js/physics.js` (`passesUnder`, `step`'s TRENCH arm rule) | `crates/sim/src/robot.rs` | cd3b5ff | ported | `drive-2910` |
| `js/mech/phys2910.js`, `r2910.js` (`profile`, the intake pivot: `preSteps`' arm, `postStep`'s goal) | `crates/sim/src/mech/r2910.rs` | cd3b5ff | ported | `drive-2910` |

## Scripts

| Script | Needs | Runs here |
|---|---|---|
| `fuel-start` | M2b (FUEL) | yes, identical |
| `fuel-launch` | M2b (FUEL) | yes, identical |
| `drive-2910` | M2c (robot body and drive) | yes, identical |
