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
| `js/mech/phys8793.js`, `r8793.js` (the trains, conveyor, arm and its pivot loop, turret indexer, shooter, `trenchArm trenchPush`), `js/nav.js` (`TRENCH_ARMS underTrench`) | `crates/sim/src/mech/r8793.rs`, `phys8793.rs` | 0e2e9d1 | ported | `intake-8793`, `shoot-8793`, `trench-8793` |
| `js/echain.js` (`chainLoad8793`, `TrackChain.insideLinks`; its geometry as data, `gen8793.json`), `js/mech/turretDrive.js` (`drive8793`) | `crates/sim/src/mech/turret_drive.rs` | 0e2e9d1 | ported | `shoot-8793` |
| `js/mech/phys1678.js`, `r1678.js` (the trains, the arm's position loop, the extension on its pin and slot, the net, the lid on the climber, the tunnel's loop, prefill, the shot verifier, contractHopper, ferries, `watchFull`) | `crates/sim/src/mech/r1678.rs`, `phys1678.rs` | 0e2e9d1 | ported | `intake-1678`, `shoot-1678` |
| `js/mech/phys1690.js`, `r1690.js` (the carriage on its racks and its agitation, the kicker bar on its pin, the floor's rolling resistance, the belts, kickers and feeder wheels' loops, the top plate's bow, the turret's aim; the code-built surfaces as data, `gen1690.json`), `phys1706.js` (`speedLoop profile`) | `crates/sim/src/mech/r1690.rs`, `phys1690.rs` | 0e2e9d1 | ported | `intake-1690`, `shoot-1690` |
| `js/echain.js` (`chainLoad1690`, the swivel chain's arm table as data), `js/mech/turretDrive.js` (`drive1690`) | `crates/sim/src/mech/turret_drive.rs` | 0e2e9d1 | ported | `shoot-1690` |
| `js/mech/phys7769.js`, `r7769.js` (the intake on its racks (a shaft whose racks hold it against small pushes, `Racks`), the kick bar riding up the bumper, the belt floor, the vectored feeder wheels, the shot table's velocity loop and hood, the Truffle Shuffle; its net over the hopper, a surface with its own touches) | `crates/sim/src/mech/r7769.rs`, `phys7769.rs` | 90983b3 | ported | `intake-7769`, `shoot-7769` |
| `js/mech/phys341.js`, `r341.js` (the slapdown arm on its pivot, the zipper's rollers, mecanums and flat omnis, the uptake, the turret's shooter and hood, their state machine and unjam, the feeder's back-off, their RobotState aiming point; the guards not in their CAD) | `crates/sim/src/mech/r341.rs`, `phys341.rs` | 90983b3 | ported | `intake-341`, `shoot-341` |
| `js/mech/phys1114.js`, `r1114.js` (the box on its racks and their slide requests, the kicker, the floor, squish wheels, side and passive rollers, the elevator and roller wall, the flywheel and hood rollers, their aim, Shoot and AgitateIntake gates, LowerIntake and RunIntakeBallevator; its front panel (a solid sheet that remembers each FUEL's side) and its blanket, surfaces with their own touches) | `crates/sim/src/mech/r1114.rs`, `phys1114.rs` | 90983b3 | ported | `intake-1114`, `shoot-1114` |
| `js/mech/phys6328.js`, `r6328.js` (the slam on its position loop and slews, the hopper's front on its pin and slot, the Superdexer floor, the kickers' velocity loops, Flywheel.runVelocity's profiled setpoint and bang-bang, LaunchCalculator's maps, the compaction; its bungee net, a surface with its own touches that flexes from the load's push) | `crates/sim/src/mech/r6328.rs`, `phys6328.rs` | 90983b3 | ported | `intake-6328`, `shoot-6328` |
| `js/mech/phys604.js`, `r604.js` (the arm on Motion Magic, the hopper box on its cam off the arm (drives.rs `cam`), the DPC rotor and its infeed and up-feed, the turret's main and hood rollers, TrackAndLaunch's LaunchCalculator, their tolerances and zones, the serializer's loops and jam detection, the drive's supply limit while launching) | `crates/sim/src/mech/r604.rs`, `phys604.rs` | 90983b3 | ported | `intake-604`, `shoot-604` |
| `js/mech/phys581.js`, `r581.js` (the deploy's homing and position loop, the floor tilting on a cam off the deploy pinions, the tower and the drum, their robot, hopper and shooter state machines, power budgets, torque-current shooter loop with the feeder's feedforward, the hopper and tower range sensors; its leaning net (NetLean)); `js/robot.js` the sliding intake's colliders (ic.slide) | `crates/sim/src/mech/r581.rs`, `phys581.rs`, `robot.rs` | 90983b3 | ported | `intake-581`, `shoot-581` |
| `js/mech/phys2056.js`, `r2056.js` (every ball-path surface built in code: tools/export-mech.mjs gen2056.json; the intake on racks with retract-to-empty, the kicker bar on a pin riding the intake, the net (Net2056), the moving shot from the exit point with a hold, the backspin kicker's loop, the one-angle pass table, accumulating); `turretDrive.js` drive2056 | `crates/sim/src/mech/r2056.rs`, `phys2056.rs`, `mechanism.rs` (Plate::new2056, set_pivot), `turret_drive.rs`, `robot.rs` | 90983b3 | ported | `intake-2056`, `shoot-2056` |
| `js/mech/phys1706.js`, `r1706.js` (the arm's Motion Magic and weight, the hopper box on a cam off the arm, two spindexers, two kickers, two turrets' accelerators and hoods, each turret's moving shot, the slack net (Net1706)); `js/robot.js` `_twinTurrets`, `_passLands(ti)` | `crates/sim/src/mech/r1706.rs`, `phys1706.rs`, `mechanism.rs` (NetSlack), `robot.rs` (`twin_turrets`, `turret_goal_reach`, `pass_lands_twin`) | 90983b3 | ported | `intake-1706`, `shoot-1706` |
| `js/robot.js` (the climber worked by hand: `driverClimb toggleClimber _manualClimb climbPose climbLevelFromLift`; the automatic climb, `requestClimb _alignCommand _climbStep`, comes with the AIs) | `crates/sim/src/robot.rs` | f325b3c | ported | `tests/climb.rs` |
| `js/robot.js` (a folding arm's collider, `_intakePhys`, `_shooter`'s autoShuttle, the turret's travel center, a hood the robot sets) | `crates/sim/src/robot.rs` | 0e2e9d1 | ported | `intake-8793`, `trench-8793` |

## Scripts

| Script | Needs | Runs here |
|---|---|---|
| `fuel-start` | M2b (FUEL) | yes, identical |
| `fuel-launch` | M2b (FUEL) | yes, identical |
| `drive-2910` | M2c (robot body and drive) | yes, identical |
| `intake-8793` | M3c (8793) | yes, identical |
| `shoot-8793` | M3c (8793) | yes, identical |
| `trench-8793` | M3c (8793) | yes, identical |
| `intake-1678` | M3c (1678) | yes, identical |
| `shoot-1678` | M3c (1678) | yes, identical |
| `intake-1690` | M3c (1690) | yes, identical |
| `shoot-1690` | M3c (1690) | yes, identical |
| `intake-7769` | M3c (7769) | yes, identical |
| `shoot-7769` | M3c (7769) | yes, identical |
| `intake-341` | M3c (341) | yes, identical |
| `shoot-341` | M3c (341) | yes, identical |
| `intake-1114` | M3c (1114) | yes, identical |
| `shoot-1114` | M3c (1114) | yes, identical |
| `intake-6328` | M3c (6328) | yes, identical |
| `shoot-6328` | M3c (6328) | yes, identical |
| `intake-604` | M3c (604) | yes, identical |
| `shoot-604` | M3c (604) | yes, identical |
| `intake-581` | M3c (581) | yes, identical |
| `shoot-581` | M3c (581) | yes, identical |
| `intake-2056` | M3c (2056) | yes, identical |
| `shoot-2056` | M3c (2056) | yes, identical |
| `intake-1706` | M3c (1706) | yes, identical |
| `shoot-1706` | M3c (1706) | yes, identical |
