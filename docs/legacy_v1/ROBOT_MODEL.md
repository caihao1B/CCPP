# Robot parameter provenance and model scope

**Reference product:** Agria 9500-70 remote-controlled caterpillar mower (not autonomous).
Manufacturer: https://agria.de/en/products/remote-controlled-devices/agria-9500-70
Source checked 2026-09-14. No product pictures, logos or manuals are redistributed.

## Published facts used

| Fact | Value |
|---|---|
| Length × width | 1.10 × 1.05 m |
| Working width | 0.70 m |
| Published upper speed range | 5.8 km/h |
| Turning on the spot | Supported |

The implementation does not substitute data for the 9500-80 or 9500-90. Mass/engine torque/slope specifications are not used to invent dynamics, energy, terrain or traction claims.

## Explicit research assumptions

Centred rectangle body and centred circular cutter; equivalent left/right track separation .80 m; work/transit .50/.80 m/s; linear acceleration .35 m/s² and jerk .70 m/s³; yaw speed .60 rad/s, acceleration .40 rad/s², jerk .80 rad/s³; track speed1.0 m/s, acceleration .4 m/s² and jerk .7 m/s³. Geometry clearance .10 m plus .01 m numerical planning reserve. Tool startup/shutdown waits2/3s are arbitrary conservative event conventions, **not verified blade stopping times**. Stationary startup service credits a cutter disk for the initial task, not a measured grass-cutting rate.

These parameters live in `models.Robot` and are exported under `simulation_parameters`. Published facts are a separate block. `.80 m` drive spacing and cutter position require measurement before any hardware comparison. The GUI intentionally fixes the robot reference; it does not invite a user to shrink the robot to make a difficult map pass.

## Strategy

Forward translation with equal drive speeds; finite-rate zero-turn with opposite drive speeds. No chassis reverse, no simultaneous moving curved turns in the supported baseline. Straight segments can pass some gates too narrow for the all-angle circle if the whole rectangle translation is safe and endpoints permit turning. Pivot must have room for all intermediate orientations.

`WORK`/`SERVICE` create coverage credit; `TRANSIT`/`TURN`/`WAIT` do not. This is a planning bookkeeping convention, not a safe command interface to a real mower clutch. Finished ground remains traversable. Returning home checks position, not restoring the initial body heading.

## What is not modeled

Track slip or lateral soil shear; terrain and slopes; suspension; blade inertia and material loads; crop discrimination; people or dynamic obstacles; GPS/LiDAR/perception uncertainty; feedback control; battery/fuel/radio operating limits; safety-rated exclusion distances. Real mowing may avoid repeated zero-turns to protect turf. This model gives a conservative and auditable research comparison, not deployment instructions.

The .80 m manual setback is a **declared coverage policy**, not a manufacturer safety perimeter. Outside the task contract still counts in original-area residuals. A 100% task-centre result may leave edges and areas between coarse centres uncut.
