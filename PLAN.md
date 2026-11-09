# Porting plan

The browser game stays the reference. Each milestone ports one layer and checks it against the
browser game's numbers, using the same headless scenarios its `tools/` run.

## M1: foundation (this)
- [x] The field's colliders and the FUEL, exported from the browser game (`assets/world.json`) and loaded into native Rapier.
- [x] Headless tests: the field loads as exported, the FUEL settles on the field, and a dropped FUEL bounces and comes to rest.
- [x] Benchmark against the browser's Rapier on the same scenario.
- [x] Bevy app: the field CAD, the FUEL, an orbit camera, a HUD, and a collider view (F1).
- [x] Field and robot models bundled for offline play; quantized meshes are dequantized for Bevy.
- [x] Windows build, installer and auto-update (Velopack on GitHub Releases) on every push to `main`.

## M2: robots
- Export each robot's config (frame, bumpers, drivetrain, intake, hopper and shooter geometry) and its colliders from `js/robotConfigs.js` and `js/robot.js`, the same way as the field.
- Swerve and tank drivetrains (`js/drivetrain.js`): current limits, azimuth time, pushing.
- Robot models: the CAD parts (`assets/cad/robots`) and the drawn ones (`js/robotModels.js`).
- Driving with keyboard and gamepad (`js/input.js`).

## M3: mechanisms
- The surfaces FUEL rides on and the rollers, flywheels, hoods and turrets (`js/mech/`, `js/surfaces.js`, `js/hopper.js`), with the foam contact model.
- Check each robot's intake rate, capacity and BPS against the browser game's tools.

## M4: the match
- Rules, scoring, HUB and OUTPOST logic, the timer and the HUD (`js/rules.js`, `js/match.js`).
- Shooting: ballistics, Magnus, aim noise (`js/ballistics.js`).

## M5: AI and autos
- The opponent AI, navigation and team autos (`js/opponent.js`, `js/nav.js`, `js/auto*.js`, `js/teamAutos.js`).
- The trained policy (`js/nn`, `js/trainedBrain.js`).

## M6: feel
- Sound (Cheesy Arena audio loaded at runtime, not bundled), rumble, LEDs, the venue, and graphics presets for low-end machines.
