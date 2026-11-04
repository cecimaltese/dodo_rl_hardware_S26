# Dodo hardware — HANDOFF (cross-session context)

> Purpose: hand this to a new session (Claude or human) so it can pick up exactly where we
> left off. The README is the manual; this file is the state, the decisions and the history.

---

## 0. WORKING AGREEMENT (read first, every session)
**Rule:** after doing *anything* in this project, update this `HANDOFF.md` and/or the
`README.md` before ending the session. We work across different computers with fresh
sessions, so the repo files are the *only* shared memory — a new chat gets its full context
by reading these. Keep §2 and the log honest (mark what's done, note what's untested,
record decisions). If you change code, reflect it here in the same turn.

Machines: since 2026-11-04 the robot is driven from **`TUM770035243-11`** (Ubuntu 24.04,
Python 3.12, RTX 3090; repos in `/home/dodo/git/`). It replaced the old robot PC
(`/home/dodo/Documents/dodo/`, Python 3.10), which had an issue. Development also
happens on Macs. Anything touching the motors/CAN runs on the robot machine. The IMU is
plain USB serial, so `imu_check.py` and `imu_locate.py` also run from a Mac with the
Nicla plugged into it (`--port /dev/cu.usbmodem*`). Setting up a machine: README
*Machine setup*.

**Resuming on any machine.** Until `merge-zmq-dev` is merged, `main` still has the
pre-merge ROS2 stack and its old docs, so work from the branch:
```bash
git clone git@github.com:cecimaltese/dodo_rl_hardware_S26.git && cd dodo_rl_hardware_S26
git switch merge-zmq-dev
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt   # Python >= 3.10
.venv/bin/python -m pytest   # 153 passed, 1 known failure, 1 skip (2026-11-04)
                             # (3 more skips without dodo_rl next to the repo)
```
Then read §2. A Claude session's local memory doesn't travel between machines; this file
does.

---

## 1. Goal
Run a trained policy on the physical Dodo through the same `step(action) → observation`
contract it was trained with. `motor_service` + `imu_service` are the robot,
`sim_service` is a real-time MuJoCo stand-in on the same ZMQ topics, and `ZmqEnv` +
`run_policy.py` drive either one. The policies so far come from the S26 Isaac Lab project
`dodo_rl`. The next one is trained by Shafeef Omar from `description/`.

**Roadmap (agreed with Shafeef, 2026-11-04):**
1. Joints real → sim, accurately: mapping, sign, zero, gear ratio (`real2sim.py`; the
   visualizer is the check).
2. The real joint limits.
3. A simple PD standing controller on the robot (joint-space PD to the default pose).
4. Shafeef trains a standing policy; deploy it through `ZmqEnv`.
5. Walking. Probably needs onboard compute (a Jetson) and a battery: later.

---

## 2. Current state (2026-11-04)
The subsystem table is in the README (*Status*). In short: everything is verified in sim,
the IMU pose is measured, CAN is up on the new machine, and nothing has been commanded on
the robot yet.
- **Verified:** the stand policy in MuJoCo (30 s+ lockstep; 10 s through the full ZMQ
  stack), the IMU on the bench and its pose on the robot (axes/signs by live check), power +
  CAN on the repaired robot (all 8 motors answer), the visualizer reading the live
  ODrives. The test suite runs anywhere: 168 passed, 1 known failure (the joint-order
  guard), 1 skip (stale flat run); without `dodo_rl` beside the repo, 3 more
  training-side skips. Python 3.12 (new robot machine); before 2026-11-04, 3.10.
- **Never run on hardware:** `motor_service`, the safety layer (including the silent-joint
  watchdog, fixed 2026-11-04), any commanded motion.

**Next steps, in order** (roadmap step 1 is 1–4 here):
1. **Knee/ankle gear ratios.** `python3 calibrate_joints.py --ratio <joint>` for each of
   `lower_leg_left/right`, `foot_left/right`: hold the link above still, phone on its edge
   on the moving link, turn the joint 40–80°, type the two angles; 2–3 turns each. A
   nameplate/CAD/WS25 number beats this if anyone has one. Enter them in
   `fleet_config.GEAR_RATIO`. Check: turn an ankle 90° with `visualize_motor_state.py`
   running — the model must turn 90°. (If the ankles' ~32:1 holds for the knee too, its
   real stops sit inside its URDF range: 2026-07-16 log.)
2. **Persistence:** `calibrate_joints.py --check-persistence` (power-cycle the motors
   without touching the robot). Record the verdict here: it decides whether the Damiao
   zeros can be stored (assumed) and confirms the ODrives are boot-relative.
3. **Calibrate** (README bring-up step 3): the main pass at the zero pose (the default:
   legs hanging straight down, soles level, body level by `imu_locate.py level`), then
   `--verify`, then the visualizer as the acceptance check: every joint follows your
   hands the right way and by the right amount. The July `joint_calib.py` was deleted
   2026-11-04 (git: `0ecb5c7`). `calibrate_joints.py` now polls every motor and calls a
   silent one OFFLINE; July's did neither. Watch the hip pitch (`upper_leg_*`) signs: their
   measured ranges are the mirror evidence (`fleet_config.py`).
4. **Limits** (roadmap 2): `calibrate_joints.py --limits` (both hard stops per joint, in
   sim radians); the visualizer's exit table shows the reached range too. Then
   `fleet_config.JOINT_LIMITS_RAD` ← the measured stops (all 8, in the sim frame) and
   `BOOT_POSE_RAD` ← the stops the ODrive joints are folded against at power-on (knees at
   `sim_lo`, ankles at `sim_hi`; `--home` uses `sim_hi` for every joint).
5. **Wire the calibration into `motor_service`:** `_JointAdapter` through
   `real2sim.JointCal` (sign, mapping; the ratio already goes through it). Damiao zero
   from `joint_calib.py` if the persistence test allows; ODrive zero from the boot pose.
6. **PD standing controller** (roadmap 3), on the gantry: bring-up steps 5–6 first (one
   joint, a few degrees; then `tools/dodo_startup.py`, which already enables, ramps to the
   default crouch and holds it — that IS a joint-space PD stand). Then the gains toward the
   trained kp 32 / kd 3 behavior (MIT units ≠ Isaac Lab units: match the step response).
7. **Shafeef's standing policy** (roadmap 4): adapt `rl_env.py` to its contract, add the
   run to `policies/`, rehearse on `sim_service`, then the gantry. Before it runs on the
   robot, settle the IMU's pitch (below).
8. **Merge `merge-zmq-dev` into `main`** once the team has looked at it (on origin; a
   fast-forward). Until then, fresh clones land on the old docs.

**IMU, for step 7:** `description/` (exact-fit `imu_joint`) went to Shafeef 2026-11-04.
Two phone-levelled placements still disagree by ~3° in pitch (11-02 reads 0°, today's
3.2°/3.9° nose-down; the phone lay on its edge, so not the camera bump). It doesn't touch
training (the policy observes the body frame, so the mount rotation cancels in sim) and
the PD stand doesn't use the IMU. It matters only through `AXIS_REMAP` on the robot: one
careful level with `imu_locate.py level` running, then `measure --skip-position --exact`
if it still reads ~3°. Optional: weigh the robot (the CAD says 4.72 kg).

---

## 3. Decisions (don't re-ask)
- Transport → **ZMQ, not ROS2** (2026-07-06). The ROS2 nodes were deleted on 2026-10-30;
  their last state is at `658874c`.
- ODrive transport → **CAN Simple**. USB only for configuration and gain tuning
  (gains can't be set over CAN).
- CAN → **two buses**, one adapter each, pinned by serial to `can_dm` / `can_od`.
  ODrive nodes 3/4/7/8 (measured on the live bus).
- **No powered homing, ever** (2026-07-06): it could overload the 3D-printed parts and
  belts. Power on with the ODrive joints folded at their stops, then `dodo_ctl calibrate`.
  `motor_service` never auto-enables.
- Safety: `|torque| > 0.8 × rated` → that one motor IDLE (torque normalized 0–1 of rated);
  full e-stop on fault, over-speed, over-travel, silent feedback or a stalled command
  stream.
- Deployment (professor, 2026-06-18): the policy must **not** observe `base_lin_vel`. The
  first target is stand-and-balance (obs 30).
- IMU → firmware **v2** (GAMERV fusion, no magnetometer) + `imu_protocol.py` (2026-10-30).
  The streaming firmware from `zmq_dev` was dropped: it fused the magnetometer and was
  never flashed.
- `fleet_config.py` is the single source of truth for the robot; never copy it
  (`test_fleet_config.py` enforces this).
- The repo is self-contained (2026-10-30): robot description in `description/`, reference
  policy in `policies/`. It is meant to move to an ATARI lab repo.
- Policy training → **Shafeef trains it** (2026-10-30). This repo owns sim2real + hardware.
- IMU pose → **one record, `description/imu_mount.json`** (2026-11-01). `imu_locate.py`
  generates the URDF's `imu_link`/`imu_joint`, `AXIS_REMAP` and the README's IMU table
  from it; never edit those by hand. Orientation is snapped to the nearest square
  mounting unless `--exact`. Position is measured by hand, since it only affects raw
  accelerometer readings, which the policy doesn't observe.

---

## 4. Log (newest first)

### 2026-11-04 (b) — real2sim: one reading→angle map, gear ratio, meshed visualizer
Roadmap step 1 tooling (§1). Software only; the visualizer was smoke-tested on the real
ODrive bus (listen + RTR, nothing enabled).
- **`real2sim.py`**: `q = sign·(raw − zero)·scale`, scale 1 (Damiao) or 2π/ratio
  (ODrive), as `JointCal`. `load()` builds every joint's map from `joint_calib.py` +
  `fleet_config.GEAR_RATIO`, identity where there's no usable entry, never a guess.
  `calibrate_joints.py`, `visualize_motor_state.py` and `motor_service.py` (unit scale
  only, so far) all go through it; before, each had its own copy and none knew the ratio.
- **`fleet_config.GEAR_RATIO`** per knee/ankle, `None` = unmeasured = treated as 1 (same
  behavior as before) with a warning everywhere. Each `joint_calib.py` entry now records
  the ratio it was made with, and is refused once `GEAR_RATIO` differs.
- **`calibrate_joints.py --ratio <joint>`**: the ratio measured with a phone on the
  moving link (|Δturns|·360/|Δdeg|), also re-checking which node moved.
  **`--pose zero`** (now the default): the main pass's reference is the URDF zero, legs
  hanging straight down and soles level, which a phone / plumb line / `imu_locate.py
  level` can check, instead of an all-joints crouch by eye (`--pose crouch` keeps July's
  way). `--out` defaults to the repo's `joint_calib.py`, the one `real2sim` reads.
- **The July `joint_calib.py` is deleted** (in git at `0ecb5c7`): its ODrive zeros were
  boot-relative and stale, upper_leg missing, `lower_leg_right` broken (node 7's loose
  connector). With `GEAR_RATIO` unset its entries still "matched", so the visualizer was
  quietly applying them. `real2sim` now also warns that an ODrive zero only holds until
  the next power cycle.
- **Visualizer**: the URDF's STL meshes (parsed in memory with
  `discardvisual="false"`; the file and the physics models are untouched, so no lost
  MJCF needed), collision boxes hidden (`--boxes`, or geom group 3 in the viewer), fresh
  feedback only, short labels in policy order, ! past the URDF range, and on exit the
  range each joint reached next to the URDF and `fleet_config` limits. Clean shutdown (it
  raced the reader thread, then core-dumped in GLFW teardown).
- **Live readings (motors powered, idle, not calibrated):** knee_L −0.094, knee_R +1.845,
  ankle_L 0.000, ankle_R −0.514 motor turns since power-up. ankle_L at exactly 0 is fine
  if nobody moved it since power-up; check it moves.
- Tests: `test_real2sim.py` (the map, ratio refusal, `--ratio` maths, write/read round
  trip, `motor_service`'s scale, the meshed model). Suite: 168 passed, 1 known failure,
  1 skip.

### 2026-11-04 — new robot machine; silent-joint watchdog fixed; IMU refit exact
Moved to `TUM770035243-11` (§0). Full review of both repos; no motor was enabled.
- **Hardware (user): node 7's driver (`lower_leg_right`, per the config) had loose
  cables and connector. Refitted and re-soldered; a major issue, fixed.** That likely
  explains July's `lower_leg_right` "never moved" (stops 0.03 turns apart), and it is
  exactly the failure the old watchdog would have missed. The node → joint mapping is
  still the July calibration's word only (it agreed with `fleet_config.py` for all four
  ODrives); a hand-move check against `candump` would settle it in minutes.
- **First IMU live check (user, phone level), exact fit:** level step 3.9° (3.8°
  nose-down, 0.6° right-down) — FAIL; nose-down PASS (+55° about +y); left-down FAIL
  only because the robot wasn't rolled in time (+3° about x). The snapped remap would
  have read 7.7° on the same data. Today's and 11-02's phone-levelled poses differ by
  3.8° in pitch and agree in roll: open question, §2 step 1. Tooling: `imu_locate.py
  level` (live tilt readout while levelling), and the live check now says "re-run" for a
  tilt that never got there, keeping "re-measure" for one that went the wrong way.
- **Second live check:** nose-down PASS, left-down PASS (the first had been rolled to
  the user's left, i.e. the robot's right; the gantry cable under the hip makes a clean
  sideways tilt awkward). With a proper rotation, x and y right means all three axes
  right: signs/axes are verified. Level 3.2° (2.9° nose-down, 1.3° right-down), against
  3.9° (3.8°/0.6°) the first time. The Nicla is firm in its housing (< 1° of play). CAD
  check: the top plate and base plates of `body.STL` are exactly parallel to the body
  frame, so the top plate is the right phone reference; the remaining suspect is the
  phone's own bias, which differs with how it lies (§2 step 1).
- **Safety bug, fixed: the per-joint feedback watchdog could never fire.** `has_feedback`
  meant "has ever reported": the ODrive driver keeps its last encoder frame, `damiao_motor`
  1.0.6 never clears or timestamps its state (and its polling thread exits *silently* on any
  bus error), and `_safety_check` refreshed the watchdog timestamp every tick. A dead
  joint, bus or polling thread would have kept publishing frozen numbers flagged as live.
  Now both drivers have `feedback_age()` (DamiaoJoint timestamps the library's
  `decode_sensor_feedback`), `has_feedback` means younger than
  `fleet_config.FEEDBACK_FRESH_S` (0.1 s), and the watchdog e-stops when an ODrive that
  has streamed, or a Damiao while enabled, is silent for > `FB_WATCHDOG_S`. A silent
  ODrive also loses its calibration: a rebooted drive re-zeroes its encoder. Disabled
  Damiaos are silent by design and now show `has_feedback: false`; `demo_combined`'s
  default joint list no longer depends on it. Watchdog timing moved to the monotonic
  clock. `test_motor_watchdog.py` runs the real service, drivers and `damiao_motor` on
  python-can's `virtual` bus against a fake fleet; mutating any of the four pieces makes
  it fail.
- **`calibrate_joints.py` couldn't see a silent motor, and never polled.** `Fleet.read()`
  returned the cached value (0.0 before any frame), so `online()` always said 8/8 and
  `--check-persistence` would have called an offline drive "BOOT-RELATIVE". It also sent
  nothing, so the Damiaos (which only answer frames) were frozen at their enable reply
  unless they stream on their own. Unknown which, but it may explain part of the July
  run. It now pokes every motor before reading (zero-gain MIT frame / RTR, like
  `read_motor_state.py`) and returns None for one without fresh feedback.
  `read_motor_state.py`'s Damiao ONLINE uses freshness too.
- **IMU: 2026-11-02 quality numbers** (they are in `imu_mount.json`, never logged): level
  vs z axis 4.44°, snapped vs exact 4.73°, cross-check 0.43°. The level pose says the
  board sits ~4.4° off square, and it looks tilted. Snapped, a level robot would have read
  3.6° nose-down and 2.6° right-side-down, and the live check's level test
  (`g_z < −0.97`, ~14°) let it through. Now: `imu_locate.py refit [--exact]` re-solves
  from the record's stored poses (no IMU), and the live level step fails above 2°. The
  record is refit **exact** (chip x→left 3.0°, y→forward 4.0°, z→down 4.4° off square;
  `imu_joint` rpy −3.0785 −0.0449 1.5980). Not yet confirmed by a live check (§2 step 1).
  The two files not pushed on 11-02 came in as `1c826a4`.
- **Sign evidence re-read** (`fleet_config.py`, README): mirrored hip-ROLL ranges are
  expected (both hips share +x, so + is outward on the left and inward on the right),
  which fits July's +1/+1. The hip-PITCH pair is the real evidence of an inverted motor,
  and July never measured it. The 2026-07-16 knee-ratio remark was backwards (corrected
  there).
- **CAN udev rule committed as `setup/99-dodo-can.rules`** (the old robot PC's was never
  committed): adapter `0027…3438` → `can_dm`, `0038…3438` → `can_od`, each brought up at
  1 Mbit by the rule. Verified on the robot, motors powered, nothing enabled: ODrive
  nodes 3/4/7/8 stream on `can_od` (all IDLE, axis_error 0, encoder estimates at
  100 Hz); Damiao 1/2/5/6 answer a register read (MST_ID) on `can_dm` with feedback ids
  0x11/0x12/0x15/0x16, as in `fleet_config.py`. `odrive_can_scan.py` now defaults to
  `can_od`.
- **The ODrives aren't configured alike.** Node 7 streams exactly what
  `odrive_can_setup.py` sets (heartbeat, encoder, torques at 100 Hz). Nodes 3/4/8 send
  heartbeats at 10 Hz plus motor_error, iq, bus_v_i, torques and powers (node 8 also
  temperature) at 100 Hz each, presumably WS25's config. That's ~2200 frames/s, ~28 % of
  the 1 Mbit bus, before `motor_service`'s 4 × 200 Hz commands. Fine for now; if the bus
  gets tight, run `odrive_can_setup.py` (USB) on 3/4/8. Only encoder estimates and
  torques are read; heartbeats (axis state/error) are too.
- Small: `rl_env.py` / `dodo_rl` `dodo.py` said +0.2 `upper_leg` = "thigh forward"; it's
  back. `dodo.py` claimed kp 30 / kd 0.5; the actuator is 32 / 3. `read_motor_state.py`
  docs said can0/can1.
- Suite on Python 3.12: 153 passed, 1 known failure, 1 skip.

### 2026-11-02 — `imu_locate.py` cleared to run from the Mac
The robot PC had an issue, so the IMU session moves to the dev Mac. Nothing in the tool
is Linux-specific (pyserial, file edits, MuJoCo). Checked on the Mac in a fresh
Python 3.10 `.venv`: full suite 140 passed, 1 known failure, 1 skip; a complete
`measure` (poses, position, write, static + live check) against a fake Nicla on a macOS
pty passed. Not checked: macOS at 921600 baud with the real board (a pty can't set custom
bauds; `open_imu` falls back to 115200 if it fails). Docs: "Or on a Mac" in
`docs/imu_locate.md`; the port hint now names `/dev/cu.usbmodem*`.

### 2026-11-01 — `imu_locate.py`: measure the IMU pose and write it everywhere
Prep for the bench session on 2026-11-02. Not run on the real IMU yet.
- **Why:** the IMU pose was a manual recipe (`imu_sniff --axes` → paste `AXIS_REMAP` →
  hand-write `imu_link`/`imu_joint` → convert to rpy → check in MuJoCo → README). That is
  five places to get a sign wrong, two of which must agree exactly.
- **What:** `imu_locate.py measure | apply | check [--live]`. The record
  `description/imu_mount.json` is the single source; the URDF block (between
  `BEGIN/END imu_locate.py` markers), `AXIS_REMAP` and the README rows are generated from
  it.
  - Orientation from three resting poses. Level gives z. Nose-down and left-side-down each
    give the heading, and their disagreement is a cross-check that catches a pose done
    the wrong way round. The result is snapped to the nearest square mounting; it refuses
    above 10° (`--exact` keeps the fit).
  - Position from four ruler readings: offsets from the hip-pitch axis, whose x/z come
    from the URDF, and a left/right mirror pair for y.
  - `check` compares the record, URDF, `AXIS_REMAP`, MuJoCo's import and `sim_env`'s
    model. `--live` tilts the robot through the deployment path (`imu_protocol` →
    `imu_service` scales → `ZmqEnv` projected gravity).
- **`imu_sniff.py --axes`** now runs `imu_locate`'s solver (read-only), so there is one
  implementation.
- **Tests:** `test_imu_locate.py`, against a fake Nicla, covers all 24 square mountings
  including gimbal lock, wrong-way and skipped poses, an angled board, the files,
  MuJoCo, and the live check. Mutating the solver signs, the y formula, the URDF rotation
  or the live-check signs makes it fail. Suite: 140 passed, 1 known failure, 1 skip.
- **Rehearsed:** the full `measure` CLI against a fake Nicla on a pty, plus the partial
  re-measures, `apply`/`check`, hand-edit detection and `imu_sniff --axes`. With a
  measured, gimbal-locked mounting written into a repo copy (no `dodo_rl`), the whole
  suite still passed (137 passed, 4 skips): the stack accepts a non-identity
  `AXIS_REMAP` and an `imu_link`.
- **Open:** the IMU mass (5 g) is a guess. Untested against real-sensor noise.

### 2026-10-30 (b) — zmq_dev and main merged; repo made self-contained
On branch `merge-zmq-dev`, pushed to origin, not yet merged into `main`. `zmq_dev`
(2026-07-06) and main (to 2026-07-17) had diverged, and the ZMQ services still carried
their pre-07-16 config.
- **Merge** of `zmq_dev` (`582c32e`). Textual conflicts only in the README and HANDOFF.
- **`motor_service` ← `fleet_config`.** It hunted ODrive nodes 7–10 (the bus has 3/4/7/8),
  defaulted both motor types onto one `can0` (the robot has `can_dm`/`can_od`), and clamped
  the Damiao joints to URDF limits instead of the measured ranges. `BOOT_POSE_RAD` moved to
  `fleet_config`. `sim_service` now clamps with the same limits as the robot.
- **`imu_service` ← `imu_protocol`.** The ZMQ version couldn't read the flashed v2
  firmware: it opened the port at 115200 instead of 921600, would have scaled v2's
  ±4 g / ±1000 °/s counts with v1's ±8 g / ±2000 °/s (doubling `base_ang_vel`), and ignored
  the quaternion. Now it auto-detects v1/v2, takes the ranges from `'V'`, and applies
  `AXIS_REMAP` to accel, gyro and quaternion (a new test checks the quaternion-remap math
  for non-identity mountings). Firmware v1/v2 moved under `firmware/`; `imu.cpp` (a copy of
  v1) and `firmware/imu_stream_nicla` were dropped and remain on `zmq_dev`.
- **ROS2 nodes deleted** (`motor_controller`, `observer`, `real_env`).
- **Self-contained:** the robot description lives in `description/` (it was `../dodo_files`),
  with its path spelled once (`sim_env.URDF_PATH`). `visualize_motor_state` now defaults
  to the URDF. The reference stand policy is vendored in `policies/` (from `dodo_rl` @
  `52b01db`), so the acceptance tests also run without `dodo_rl`; checked in a copy of the
  repo with no sibling folders.
- **Flaky test fixed:** `test_step_detects_dead_service` failed about 1 run in 6, also on
  `zmq_dev`. After its 0.4 s pause a parked message legitimately delays detection by up to
  `max_state_age_s`; the test allowed 3 steps.
- **Docs:** README consolidated (main's measured hardware facts + the ZMQ architecture + one
  bring-up sequence). `ZMQ_MIGRATION.md` → `docs/zmq.md` (spec only). `firmware/README.md`
  rewritten.
- **Found, not fixed:** `99-dodo-can.rules` was never committed; `odrive_can_scan.py`
  defaults to `can0`; `joint_calib.py` is still the incomplete July run. The
  `dodo_rl_hardware_S26 copy/` folder on the dev Mac is content-identical to `zmq_dev`
  and can be deleted.

### 2026-10-30 — hardware repaired; URDF restored for Shafeef
- **Hardware (user report):** all motor drivers have power again and the CAN daisy chain
  has full continuity. No motor has been commanded since.
- **New plan:** Shafeef Omar trains the next policy himself, so `dodo_rl` (Isaac Lab) is
  mostly set aside. Work here is sim2real + hardware. He needs an up-to-date URDF
  including the IMU.
- **URDF restored** (it was never in a repo; `sim_env`/tests expected it as a sibling
  `../dodo_files`, since moved into `description/`). Source:
  `liambansbach/legged-robot_rl_genesis` @ `1cfa733`, `ressources/robots/dodo_daimao`
  (urdf + STL meshes). `dodo_rl`'s USD is byte-identical to the one stored there.
  `test_sim2sim_consistency.py` gives 17 pass / 1 fail / 1 skip with it, the same as on the
  robot PC.
- **URDF changes:** added `<mujoco><compiler fusestatic="false"/>`. Without it, MuJoCo's
  URDF import fuses fixed links and an `imu_link` silently vanishes. `imu_link`/`imu_joint`
  are pending the measured IMU pose (§2).
- **Finding:** `dodo_rl`'s USD has `hip_left` mass 0 in its physics layer. PhysX falls back
  to about 0.44 kg from the convex hull, so the impact on past training was small. Don't
  reuse that USD; regenerate it from the URDF.

### 2026-07-16 — calibration tool; first (incomplete) run; the ODrive joints are geared
`calibrate_joints.py` was written and works. The calibration itself is **not done**: the
first run produced a `joint_calib.py` that is incomplete and must be redone (don't command
motion from it).
- **The knee/ankle drives are geared, not direct.** `foot_left` and `foot_right`
  independently measured **13.402** and **13.346** motor turns between their hard stops
  (agreeing to 0.4 %, so real, repeatable stops). As `1 turn = 2π rad` that would be an
  84 rad ankle. Against the URDF's ankle range it is roughly **32:1**; `lower_leg_left`
  implies ~19.7, so the ratio is not yet trustworthy. *(Corrected 2026-11-04: this said
  "likely because the knee's real stops sit outside its URDF limits", which is backwards.
  inferred = true ratio × real range ÷ URDF range, so 19.7 against a true 32:1 means the
  knee's real travel is ~2.8 rad, INSIDE the URDF's 4.54 rad; or the knee has its own
  ratio.)* With the wrong conversion the ODrive joints under-travel
  ~32× ("nothing moves", not "drives into the stop").
- **Known bad in `joint_calib.py`:** `upper_leg_left`/`upper_leg_right` missing entirely;
  `lower_leg_right` never moved (stops 0.03 turns apart); both hips under-measured on one
  side; both hips came back `sign=+1` (no mirroring — contradicts the July 6 suspicion;
  re-measure). The `--check-persistence` verdict was not recorded.
- The calibration flow is zero-first (~17 prompts): pose the crouch once, nudge each joint;
  `--verify` proves the signs; `--limits` is an opt-in stop-walking pass.
- ODrive node ids measured off the live bus: 3/4/7/8 (the drives are still on WS25's ids;
  the web GUI never reached them). `fleet_config.py` became the single source of truth.
- IMU firmware v2 flashed and verified (`imu_check.py`: scale 0.5 % off 1 g, gyro bias
  ~1 °/s, 60 Hz).

### 2026-07-06 (b) — real IMU firmware wired in; homing replaced by boot-pose startup
*(On `zmq_dev`. The IMU parts are superseded: since 2026-10-30, `imu_service` uses main's
v2 firmware + `imu_protocol.py`, and `imu.cpp` / the streaming firmware are gone.)*
- IMU firmware committed (`imu.cpp`, Nicla Sense ME, poll-based `'L'` → `ALL …` raw
  int16 counts). `imu_service.py` converted raw → SI with guessed ±8 g / ±2000 °/s. A
  streaming firmware (`firmware/imu_stream_nicla/`, fused quaternion) was provided as a
  recommended upgrade.
- **No homing routine, ever** (user decision: a homing drive could overload the 3D-printed
  structure and belts). Startup convention: power on with the ODrive joints folded all the
  way back against their stops, then `dodo_ctl calibrate` maps turns-since-boot onto
  `BOOT_POSE_RAD` (knees −3.1416 / ankles +1.57 rad = placeholders until the stop angles
  and signs are bench-verified). Supporting changes: the drivers' `set_zero(current=…)`
  ("joint is AT angle X"); `DamiaoJoint.send_limp()` (zero-gain MIT frame, since Damiaos
  only report while enabled); a bumpless fail-closed `enable` (refuses silent joints,
  presets setpoints at the measured positions, unparks out-of-band joints to the
  safety-band edge over 1.5 s); a pre-start ramp in `ZmqEnv` (`run_policy --startup-ramp`,
  on by default for zmq); `ZmqEnv` disables on close; and `tools/dodo_startup.py`
  rehearses the whole ritual, including folding back down on exit.

### 2026-07-06 — ZMQ migration implemented (Phases 0–2); ROS2 retired
*(On `zmq_dev`; merged 2026-10-30.)* The team chose **ZMQ, not ROS2** for the real-time
framework, implemented and tested on the dev Mac. The spec is in `docs/zmq.md`.
- `dodo_ipc.py` (topics/ports, binary schemas, canonical joint order, CONFLATE
  latest-value subscriber, ctl REQ/REP, rate loop); `motor_service.py` (safety layer
  carried over verbatim, no auto-enable, per-joint flags, keeps publishing while
  e-stopped); `imu_service.py`; `zmq_env.py` + `run_policy.py --backend zmq`;
  `sim_service.py` (MuJoCo as a real-time virtual robot, which fixes the instructor's
  "sim2sim isn't real time"); `tools/` (`zmq_echo`, `zmq_hz`, `dodo_ctl`); tests
  `test_ipc.py` + `test_e2e_zmq.py`.
- Measured (dev, separate processes): states 200.0 Hz and IMU 100.0 Hz with 0 drops, the
  policy loop at 50.0 Hz with 0 overruns, state age at read 0.21 ms on average.

### 2026-07-06 — height domain randomization added to the stand policy (sim only)
`dodo_rl/.../stand_env_cfg.py` now randomizes standing height for robustness: the initial
crouch varies across a ~±5 cm band (`_HEIGHT_JOINT_SCALE=(0.7,1.3)`, STAGE 1), and the
leg-joint pose anchoring is relaxed (new `joint_deviation_legs`; `joint_deviation_all`
re-scoped to the hips), so the policy tolerates small standing-height / zero-calibration
offsets on the real robot. Robustness only: height is not commanded or observed, so the obs
stays 30-dim and the hardware side is unchanged. Retrain STAGE 1 (or resume from a STAGE-0
checkpoint) to bake it in. Tuning: `REWARD_TUNING.md` §9 in `dodo_rl`.

### 2026-07-01 (b) — safety layer implemented (then in motor_controller.py)
- **Physical joint limits** (`JOINT_LIMITS_RAD`, `RATED_TORQUE_NM`, `JOINT_VEL_LIMIT_RAD=6`)
  are the single source of truth; `_JointAdapter` converts them to each motor's native
  units, and position commands are clamped with a 0.9 margin.
- **Torque → IDLE (professor's spec):** `|torque| > 0.8 × rated` disables that ONE motor;
  the fleet keeps running.
- **Full e-stop** (disable all) on: motor fault, over-speed (> 0.9 × 6 rad/s), measured
  position past the hard limit (+0.10 rad margin), a stalled command stream
  (`CMD_WATCHDOG_S=0.5`), or a live joint going silent (`FB_WATCHDOG_S=0.5`).
- Caveat (biped): IDLE-ing one leg motor makes the robot fall — run on a gantry.

### 2026-07-01 — sim side verified; hardware bring-up prep
- **MuJoCo sim2sim works** for `dodo_stand`. The root cause of the earlier fall/shake was
  stale PD gains in `sim_env.py` (KP 30 / KD 0.5) vs the trained ImplicitActuator
  (stiffness 32 / damping 3.0, see `params/env.yaml`). Fixed to KP=32, KD=3.0. The same
  mismatch exists on the real side (Damiao kp=30 kd=0.5) — an open bench task.
- **`read_motor_state.py` added:** read-only diagnostic, marks each motor ONLINE/OFFLINE
  and streams pos/vel/torque; Damiao read LIMP so joints are backdrivable.
- Deployment gaps identified then: the IMU module as a hard dependency (since solved),
  per-joint zero/sign calibration (still open), doc/code reconciliation of the safety spec
  (done the same day).

### 2026-06-18 — CAN migration implemented
Both motor types moved behind one config-driven controller over CAN, with the
`step()`/`get_observation()` hook: `odrive_can_setup.py` (USB tool that puts a drive on
CAN), `odrive_can_joint.py` (CAN Simple driver), the unified motor controller (later
ported to `motor_service.py`), a CAN bench demo, and `damiao_joint.py` recreated from spec
(the original was never committed). `odrive_position_control.py` stays the USB
gain-tuning tool.

---

## 5. Technical notes (settled — don't re-litigate)
- **SocketCAN gives every open socket its own copy of every frame**, so two motor types can
  share a bus with one filtered socket each. `damiao_motor` 1.0.6's
  `DaMiaoController.__init__` doesn't accept `can_filters`; apply them afterwards with
  `bus.set_filters(...)`. Its frame router dispatches 8-byte frames by `data[0] & 0x0F`,
  not by arbitration id, so on a shared bus the filter is mandatory.
- **CAN Simple:** `arb_id = (node_id << 5) | cmd_id`. In use: `0x07` Set_Axis_State, `0x0B`
  Set_Controller_Mode, `0x0C` Set_Input_Pos, `0x01` Heartbeat (RX), `0x09`
  Get_Encoder_Estimates (RX), `0x1C` Get_Torques (RX). Axis states IDLE=1,
  CLOSED_LOOP_CONTROL=8; POSITION_CONTROL=3; input mode PASSTHROUGH=1.
- **Node ids on a shared bus:** 3/4/7/8 are collision-safe; node 0 (`0x000–0x01F`) swallows
  every Damiao id, and node 63 collides with `0x7FF`.
- **ODrive tuning (USB):** a buzz means `vel_gain` too high; slow oscillation means
  `pos_gain` too high. Recipe: zero `vel_integrator_gain`, raise `vel_gain` to just before
  the buzz then halve it, raise `pos_gain` to just before overshoot then back off, restore
  the integrator ≈ 0.5 × `vel_gain`. Starting point: `pos_gain≈15–20`, `vel_gain≈0.05`,
  `vel_integrator_gain≈0.1`. `save_configuration()` reboots the drive and drops USB
  (handled in `OdriveJoint.save()`).
- **The ODrive web GUI can't use the USB-CAN adapters**: `gs_usb` owns them ("Could not
  claim interface 0"), and the GUI defaults to 125 kbps anyway. Use SocketCAN.
- **Nicla:** USB-CDC doesn't reset on port open, so the first command after opening is
  reliably dropped (`query_version` retries). At 115200 baud the poll capped at ~41 Hz;
  v2 runs at 921600.
- **Feedback is only as good as its age.** Both drivers cache their last frame forever;
  check `feedback_age()` against `fleet_config.FEEDBACK_FRESH_S`, never the cached
  numbers alone. A Damiao only answers frames (enable, MIT command, limp probe, register
  read), so it is silent while disabled by design. `damiao_motor`'s polling thread exits
  silently on any bus exception. ODrives stream cyclic frames whether enabled or not.
- References that were on the OLD robot PC (not on `TUM770035243-11`):
  `~/Downloads/can_simple_utils.py`, `~/Desktop/dodo_ros2/dodo/dodo/odrive_interface.py`,
  `~/Documents/ros_odrive-main/odrive_node/src/odrive_can_node.cpp` (full cmd-id table).

## 6. Environment / gotchas
- Robot machine (`TUM770035243-11`): Python 3.12, repo `.venv` from `requirements.txt`
  (pyzmq, mujoco, onnxruntime, python-can, `damiao_motor==1.0.6`, pyserial).
  `odrive==0.6.11.post0` only for the USB tools. `damiao_motor`'s source, for reading its
  internals: `.venv/lib/python3.12/site-packages/damiao_motor/core/`. The user needs to be
  in `dialout` for the Nicla (README *Machine setup*).
- Old robot PC, for the record: system `python3` 3.10, an editable `damiao_motor` 1.0.7b2
  in the uv cache next to the regular 1.0.6.
- CAN bitrates are set at OS level, never by python-can: `setup/99-dodo-can.rules`
  brings both buses up at 1 Mbit when the adapters appear (by hand: `ip link`).
- `can0`/`can1` are enumeration order, not identity: use the udev names
  `can_dm`/`can_od`.
- The direct-CAN tools (`read_motor_state`, `calibrate_joints`, `visualize_motor_state`,
  `odrive_can_scan`) and `motor_service` all open the buses: run one at a time.
- Pausing the policy runner (a debugger, a viewer) stops the command stream, and
  `motor_service` e-stops after 0.5 s by design. `dodo_ctl estop_reset` recovers.
