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
git clone https://github.com/Atarilab/dodo_assets.git ../dodo_assets   # the robot model
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt   # Python >= 3.10
.venv/bin/python -m pytest   # 181 passed, 1 known failure, 1 skip (2026-11-05)
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
`dodo_rl`. The next one is trained by Shafeef Omar from `Atarilab/dodo_assets`.

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
  ODrives. The test suite runs anywhere: 174 passed, 1 known failure (the joint-order
  guard), 1 skip (stale flat run); without `dodo_rl` beside the repo, 3 more
  training-side skips. Python 3.12 (new robot machine); before 2026-11-04, 3.10.
- **Never run on hardware:** `motor_service`, the safety layer (including the silent-joint
  watchdog, fixed 2026-11-04), any commanded motion.

**Next steps, in order** (roadmap step 1 is 1–4 here):
1. ~~Knee/ankle gear ratios~~ — done 2026-11-05: knees 25.6, ankles 28.5
   (`fleet_config.GEAR_RATIO`; log).
2. ~~Persistence~~ — done 2026-11-05: Damiao zeros survive a power cycle, ODrive zeros
   don't (log). The startup reference pose follows from that (§3, open question there).
3. *(2026-11-05: main pass + `--limits` done and reviewed (log); `hip_right` sign fixed;
   zero pose confirmed against the URDF by photo. The reference pose is now a sitting
   pose (§3). Next per the user: the PD stand in sim, then on the robot, then a policy
   (log 11-05 (c)).)*
   **Calibrate** (README bring-up step 3): the main pass at the zero pose (the default:
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
5. ~~Wire the calibration into `motor_service`~~ — done 2026-11-05 (log 11-05 (g)).
   Smoke-tested on the real buses, disabled; never enabled on a motor yet.
6. **PD standing controller** (roadmap 3). *In sim: done 2026-11-05* (`sim_service.py
   --scene --gantry`, README *Rehearse the PD stand in sim first*; log 11-05 (e)). On the
   robot, on the gantry: README *Bring-up* steps 4–7 (calibrate `straight`, one joint
   at a time at gain scale 0.3, `home` in the air raising the gains live, then lower).
   The stand has little margin forward (log 11-05 (e)): keep the gantry close to taut,
   and watch which way it wants to tip. Re-take the weight before ending `dodo_startup`
   (its park ends limp).
   **Open before step 7 (standing): the ODrives' `vel_limit` caps the knee torque.** In
   position control the drive's velocity setpoint saturates at `vel_limit`, so the most
   torque it holds at standstill is vel_gain × vel_limit. kd 0.8 fixes vel_gain at
   0.0077 Nm/(turn/s) (knee), so at the 10 turns/s fleet_config lists that's 0.077 Nm at
   the motor, 2.0 Nm at the knee; `home` needs 2.9 Nm (sim). The drives' real
   `vel_limit`, current limit and `torque_constant` were set by WS25 and are unknown (CAN
   Simple can't read them; the `odrive` USB package isn't installed). **Plan (user,
   2026-11-05: no USB, the ports sit badly in the body): the push test** in step 6
   (`tools/pd_push_test.py`, log 11-05 (h)) measures the cap on each knee/ankle in the
   air. If the cap at full gains is under ~1.3 × the stand's need (knee 2.9, ankle 1.4
   Nm), raise the ODrive kd in `fleet_config.JOINT_KD` (cap = kd · 2π · vel_limit / N):
   kd 2 gives 4.9 Nm at vel_limit 10, and the sim stand is unchanged by it (3.8° lean,
   5.3° max vs 5.8°). If the plateau doesn't scale with the gains, it's the current
   limit instead, which only USB (or Set_Limits with a guessed current) can raise.
7. **Shafeef's standing policy** (roadmap 4): adapt `rl_env.py` to its contract, add the
   run to `policies/`, rehearse on `sim_service`, then the gantry. Before it runs on the
   robot, settle the IMU's pitch (below).
8. **Merge `merge-zmq-dev` into `main`** once the team has looked at it (on origin; a
   fast-forward). Until then, fresh clones land on the old docs.

**IMU, for step 7:** the exact-fit record is in `dodo_assets` once the PR from
`victor-hucklenbroich/dodo_assets` branch `imu-exact-fit` is merged (2026-11-05); until
then Atarilab's `main` still has the snapped one, and `imu_locate.py check` fails on
AXIS_REMAP against it.
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
- **ODrive drive calibration every power-up, over CAN** (2026-11-05): `dodo_ctl
  odrive_calibrate` (motors move 8-16°, ~16 s, legs free), before the joint calibration.
- **Drive faults while disabled are reported, not e-stops** (2026-11-05): nothing to stop,
  and the e-stop's IDLE would cancel the other drives' calibration. `enable` refuses.
- ODrive transport → **CAN Simple**. USB only for configuration. **The PD gains go over
  CAN** (2026-11-05; Set_Pos_Gain 0x1A / Set_Vel_Gains 0x1B, firmware 0.5 and 0.6;
  RAM only), set by `motor_service` at every enable from `fleet_config.JOINT_KP/JOINT_KD`
  (the earlier "gains can't be set over CAN" was wrong).
- CAN → **two buses**, one adapter each, pinned by serial to `can_dm` / `can_od`.
  ODrive nodes 3/4/7/8 (measured on the live bus).
- **No powered homing, ever** (2026-07-06): it could overload the 3D-printed parts and
  belts. Power on with the ODrive joints folded at their stops, then `dodo_ctl calibrate`.
  `motor_service` never auto-enables.
- **Zeros (2026-11-05, measured):** the Damiao zeros are stored (they survive a power
  cycle); the ODrive zeros are re-established every session from a reference pose
  against the hard stops (the user's "tuck"), then a short startup move to a working pose.
- **ODrive zero per session from a named reference pose** (2026-11-05,
  `fleet_config.REFERENCE_POSES`, `dodo_ctl calibrate --pose <name>`; no default, so
  nobody calibrates against a pose the robot isn't in). On the gantry: `straight`. The
  placeholder `BOOT_POSE_RAD` (URDF limits, 20° off the real knee stop) and July's
  raw-frame `JOINT_LIMITS_RAD` are gone; the stops come from `joint_calib.py`.
- **Reference pose = a bird SITTING pose** (user + Shafeef, 2026-11-05): Damiao readings
  saved there, the ODrive joints against physical stops. A stand-up from it is
  hardcoded first (or trained into the policy later). Still to confirm: whether the
  URDF's q = 0 moves with it. The recommendation is no: keep the URDF frames and give
  the sitting pose measured URDF angles (`BOOT_POSE_RAD`).
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
- **Robot model → `Atarilab/dodo_assets` only** (2026-11-05, user; replaces the
  2026-10-30 "self-contained" decision). Checked out next to this repo; `assets.py`
  spells every path (or `DODO_ASSETS`). `description/` is deleted. Changes to the model
  go to `dodo_assets` by PR (the user's fork: `victor-hucklenbroich/dodo_assets`).
  The reference policy stays in `policies/`. Everything is meant to move to Atarilab.
- Policy training → **Shafeef trains it** (2026-10-30). This repo owns sim2real + hardware.
- IMU pose → **one record, `imu_mount.json` in `dodo_assets`** (2026-11-01; moved
  2026-11-05). `imu_locate.py` generates the URDF's `imu_link`/`imu_joint`, the MJCF's
  `imu_link` body, `AXIS_REMAP` and the README's IMU rows from it; never edit those by
  hand. Orientation is snapped to the nearest square
  mounting unless `--exact`. Position is measured by hand, since it only affects raw
  accelerometer readings, which the policy doesn't observe.

---

## 4. Log (newest first)

### 2026-11-05 (k) — `tools/odrive_check.py`: testing a repaired motor connection
The user asked how to test the left knee's wiring after the repair (continuity alone
found last week's broken CAN lines, but a phase can conduct at milliamps and fail at
amps). README *Checking a knee/ankle motor connection*: power-off phase-to-phase
resistances (balanced, like another motor's, no short to the housing), then
`odrive_check.py` runs that one drive's calibration directly on the bus and logs the
steps, the DC bus current during the motor step (healthy ~0.12 A over ~4.2 s; the
open phase: 0.00 A, failed at 1.6 s), the rotor sweep and the result; `--repeat` to
wiggle the cable between runs, `--compare` a good drive first. It RTR-polls bus V/I,
Iq and Get_Error, which node 7 (that doesn't stream them) answers (checked on the robot,
read-only). Refuses while a ctl service answers. Tested against the fake drives
(`test_odrive_check.py`); not yet run on the robot as a tool (the same procedure as
the 11-05 (j) diagnosis, which was).

### 2026-11-05 (j) — robot: the LEFT KNEE's motor gets no current; the service fixed
- **User:** odrive_calibrate moved only the right leg noticeably, never ended in
  SUCCESS, and the service was always E-STOPPED.
- **Diagnosis (Claude, on the robot; the user had cleared it to run the process):**
  node 3 (`lower_leg_left`) reported `PHASE_RESISTANCE_OUT_OF_RANGE` + CALIBRATION_ERROR
  (0x40000000). Cleared and re-run alone: MOTOR_CALIBRATION, then that failure after
  1.6 s, rotor still, **DC bus current 0.00 A** throughout. The healthy left ankle
  (node 4), same step: 4.2 s at 0.12 A, then a 9.2 s encoder offset scan, SUCCESS. So
  **no current reaches the left knee motor: an open phase** (wires / connector between
  node 3's ODrive and its MN4004; cf. node 7's re-soldered connector, 11-04). Its
  encoder works (positions, the calibration's main pass). **Hardware fix needed.**
- The other three calibrate (measured, each alone): ankles 13.5 s, rotor 0.67 turns =
  8.4° at the ankle, back to the start; right knee (node 7, configured by
  odrive_can_setup.py: it also has an ENCODER_INDEX_SEARCH) 15.4 s, 1.17 turns = 16.4°,
  **ending 0.49 turns = 7° from its start**. Straighten the legs before `calibrate`.
- **Why always e-stopped:** a drive keeps its error latched (heartbeat axis_error) until
  Clear_Errors. motor_service e-stopped on that fault ~1 s after every start, disabled
  or not, and the e-stop's IDLE cancelled any calibration in flight. Confirmed on the
  robot before the fix.
- **Fixes:** a drive fault while the fleet is DISABLED is reported (status, log once),
  not an e-stop; enabled it still e-stops. `enable` refuses with any fault, and while
  any ODrive is busy (calibrating); `calibrate` refuses while one is busy (no joint
  zero mid-motion). `odrive_calibrate` and `estop_reset` send Clear_Errors (0x18)
  first. Error bits decoded (`odrive_can_joint.ODRIVE_ERRORS`, firmware 0.6), and
  PHASE_RESISTANCE_OUT_OF_RANGE says "no current flows: check its phase wires".
- **Checked on the robot through the fixed service:** no e-stop at start (the fault
  listed instead), odrive_calibrate → foot_left, foot_right SUCCESS at 13.8 s,
  lower_leg_right at 16.4 s, lower_leg_left failed at 1.8 s with the hint, nothing
  cancelled, never e-stopped; `enable` refused (joint calibration pending). Stopped
  there: `calibrate pose=straight` needs eyes on the legs.
- **Until the knee is fixed:** the other seven run with `motor_service.py --only
  hip_left,hip_right,upper_leg_left,upper_leg_right,foot_left,lower_leg_right,foot_right`
  (README *Bring-up* step 4): steps 5–6 in the air, the push test on three ODrives.
  Standing (step 7) needs the left knee.
- Tests: the fake drives fail like node 3 (open phase: PHASE_RESISTANCE_OUT_OF_RANGE,
  latched until Clear_Errors) and keep their last result through IDLE-while-IDLE
  (as the real ones did). New: a dead motor neither e-stops nor cancels the others,
  odrive_calibrate clears a latched error, the fleet runs without the dead joint,
  estop_reset clears drive errors, calibrate/enable refused while calibrating.

### 2026-11-05 (i) — robot: the ODrives refused closed loop (NOT_CALIBRATED); fixed
- **User at the robot, bring-up step 5:** with `demo_combined.py` no ODrive joint moved
  (even at --amp 0.95) and the push test read 0 Nm: the knees/ankles were limp.
- **Cause, read passively off can_od afterwards:** all four heartbeats: state IDLE,
  `procedure_result` 14 = NOT_CALIBRATED, axis_error 0 (Get_Error: active_errors 0,
  disarm_reason 0). The drives lose their motor/encoder calibration at every
  power-off (incremental encoders, which is also why their positions read 0 at boot)
  and refuse closed loop until it's redone. WS25's stack must have run it; our tools
  never did. `motor_service` never checked that a drive reached closed loop, so it
  reported "enabled" over four idle drives.
- **Fixes:** `OdriveCanJoint` parses the heartbeat's procedure_result and Get_Error,
  and has `request_calibration()` (FULL_CALIBRATION_SEQUENCE). `motor_service`:
  `enable` waits up to 1 s for every ODrive to REPORT closed loop, else disables
  everything and says why (NOT_CALIBRATED → run odrive_calibrate); an ODrive leaving
  closed loop while enabled is an e-stop; new ctl `odrive_calibrate` (disabled only,
  non-blocking, voids the joint zeros: calibrate pose=straight again); `status` shows
  each drive's state, procedure_result, errors. `dodo_startup.py --odrive-calibrate`,
  `dodo_ctl odrive_calibrate`; sim_service accepts it (no-op). README *Bring-up* step 4.
- **The calibration moves the motors:** motor R/L (a beep), then the encoder offset
  scan turns the rotor a fraction of a turn and back: ~10-15° at the knee/ankle (not
  measured yet). Legs hanging free.
- Tests: the fake drives refuse closed loop until calibrated, like the real ones.
  Enable fails closed (and disables the drives that did close the loop);
  odrive_calibrate → IDLE + SUCCESS → joint zeros void → calibrate → enable works;
  refused while enabled; a drive dropping out e-stops.
- Still unverified on the robot: everything after closed loop (gains over CAN, torque
  units, the vel_limit cap: the push test, §2 step 6).

### 2026-11-05 (h) — `tools/pd_push_test.py`: the ODrives' torque cap without USB
The user prefers not to use the drives' USB ports (they sit badly in the body). So the
cap (§2 step 6) is measured by hand: while motor_service holds a joint, push it slowly
and let `pd_push_test.py` read motor_states.
- **Stiffness** = torque/error at small errors vs the kp the service set: ~1 means the
  CAN gains took and the torque units are right; much stiffer means the drive's own
  gains (WS25's had an integrator) still run, or its torque is already joint-side (at
  ~the gear ratio); negative means an inverted torque sign.
- **Cap:** a torque plateau while the error grows = vel_gain × vel_limit (or the current
  limit), turned into the drive's vel_limit and the cap at full gains, against the
  stand's need. A second push at another gain scale tells the two apart (the velocity
  cap scales with √scale).
- **Fallback, checked in sim:** knee/ankle kd 1.5/2.0/3.0 instead of 0.8 doesn't change
  the stand (0.412 m, 3.8° lean; max 5.5/5.3/4.9° vs 5.8°) and raises the cap at
  vel_limit 10 to 3.7/4.9/7.4 Nm.
- `sim_service` reports kp/kd (and the gain scale) in `status` like motor_service, so
  the tool runs against the sim too: the knee pushed with up to 1.5 Nm on the gantry →
  stiffness 50.0 vs kp 50 (ratio 1.00), no plateau (the sim's PD has no velocity cap).
- Tests: `test_pd_push_test.py` (synthetic ODrive pushes: plateau → vel_limit 10 → 2.0
  Nm "NOT enough"; vel_limit 30 enough; lower bound without a plateau; joint-side
  torque; inverted sign; Damiao; moving samples ignored), and the tool against the sim
  in `test_sim_scene.py`. README *Bring-up* step 6 includes it.

### 2026-11-05 (g) — the calibration wired into `motor_service` (HANDOFF §2 step 5)
- **Joint units on the wire, raw units on the bus.** `_JointAdapter` maps every joint
  through `real2sim.JointCal`: positions, velocities, commands, clamps, the over-travel
  e-stop and the IDLE torque. The drivers' own software zero stays 0. Damiao: sign and
  zero from `joint_calib.py`, calibrated from boot. ODrive: sign from `joint_calib.py`,
  zero from `calibrate pose=<reference pose>` (`fleet_config.REFERENCE_POSES`, only
  `straight` so far); `set_zero` is now an alias for `calibrate pose=straight` (it used
  to re-zero the Damiaos by eye too). A joint whose `joint_calib` motor isn't the one
  of the same name is refused at startup.
- **Fail-closed enable:** refused unless every joint is calibrated (measured sign;
  ODrive calibrated this session) and has measured stops.
- **Clamps / e-stop:** commands 0.1 rad (`STOP_MARGIN_RAD`) inside the measured stops;
  e-stop 0.1 rad past them, judged only for calibrated joints. Velocity limit
  0.9 × 6 rad/s at the joint.
- **Gains:** `fleet_config.JOINT_KP/JOINT_KD` = scene.xml's 150/150/50/50 and
  1.5/1.5/0.8/0.8 (the Damiaos had 30/0.5). Damiao: in every MIT frame (own config
  copy per joint). ODrive: `real2sim.odrive_gains` → Set_Pos_Gain/Set_Vel_Gains over
  CAN at enable (knee pos_gain 62.5, vel_gain 0.0077; ankle 62.5/0.0062; integrator 0).
  `--gain-scale` and ctl `gains scale=` (kp × s, kd × √s; at most 1). `status` reports
  per joint `target_rad`, `err_rad`, `torque_nm`, `kp`, `kd`, `band_rad`, so
  err × kp ≈ torque checks both the gains and the torque units on the robot.
- **ODrive torque → joint Nm:** × gear ratio × sign (`ODRIVE_TORQUE_IS_MOTOR_SIDE`,
  assumed; if a drive's torque_constant included the gearing, joints would only IDLE
  too early).
- `fleet_config`: `BOOT_POSE_RAD` and `JOINT_LIMITS_RAD` removed (placeholders /
  July's raw frame); the node → joint comment now says the mapping is measured.
  `dodo_startup.py --ref-pose straight` (asks first unless `--yes`), `demo_combined.py
  --center current` (default: oscillates around the held target, no jump),
  `dodo_ctl.py gains`, `visualize_hardware.py`'s exit table compares with the measured
  stops.
- **On the real buses, disabled:** `motor_service` started with the real
  `joint_calib.py` (signs as measured, bands, kp/kd), 200 Hz with 0 overruns; `status`
  fine; `enable` refused ("no ctl `calibrate` this session" for the four ODrives).
  Nothing enabled, nothing calibrated (the legs' pose unknown).
- **Found: the ODrives' `vel_limit` caps the knee torque** (§2 step 6, open).
- Tests: the watchdog harness's fake fleet now reports positions/torques and records
  MIT frames, Set_Input_Pos and gains; `test_motor_watchdog.py` checks the wiring end
  to end on the virtual bus (angles, commands in raw units with signs, the clamp, gains
  and their scale, ODrive torque IDLE at the joint, enable refused until calibrated).
  `test_boot_calibration.py` → `test_motor_calibration.py` (the adapter, the gain
  conversion, the robot's own config is complete). Suite: 229 passed, 1 known failure,
  1 skip.
- `sim_service --scene` got ctl `gains` and `--gain-scale` (motor_service's rule), so
  README *Bring-up* steps 6–7 rehearse 1:1: gantry, scale 0.3, `home` in the air (0.005
  rad error), live to 0.6 and 1.0, lower 12 cm, release → 0.412 m, 3.5° (real CLI
  processes, no warnings).

### 2026-11-05 (f) — `visualize_hardware.py`: the visualizer shows the IMU too
`visualize_motor_state.py` → `visualize_hardware.py` (git mv; the user's request).
- The model's base got a free joint; its orientation follows the Nicla, polled in a
  thread like imu_service, through `imu_protocol` (firmware scales, AXIS_REMAP). Tilt
  only: roll/pitch from the fused quaternion's gravity direction (accelerometer
  without one, firmware v1). The heading is arbitrary without a magnetometer, so
  it's dropped unless `--imu-yaw`. Shown in the terminal (one line per source,
  rewritten in place) and in the viewer's overlay: nose-down / right-down degrees
  (imu_locate's words) and the gyro in body axes. Exit summary: readings, rate,
  largest tilt. `--no-imu` = the old tool; `--imu-only` opens no CAN bus (Mac too).
  An IMU that won't open (port held, permissions) is a warning, not a stop.
- Checked on the real Nicla (the hanging robot): firmware v2 ranges self-reported,
  67–69 Hz, |accel| 9.78 m/s², 1.6° nose-down / 0.9° left-down, gyro ~0. The main loop
  ran headless (stand-in viewer, `--imu-only`) against it: 60 Hz redraws, overlay,
  summary. Not yet looked at in the real viewer window with the motors.
- Tests: `test_visualize_hardware.py` (tilt math, heading dropped, accel fallback = quat,
  the fake Nicla mounted with the real AXIS_REMAP through `ImuFeed` → the model's body
  gravity = the robot's, a silent board). Suite: 215 passed, 1 known failure, 1 skip.

### 2026-11-05 (e) — the PD stand in sim, through the robot's own stack
Roadmap step 3 in sim (the user's plan: sim, then the robot, then a policy).
- **`sim_service.py --scene`:** `dodo_assets`' `scene.xml` as the virtual robot. Its
  MuJoCo position actuators (an implicit PD, like the drives' own loops) run at
  kp 150/150/50/50 and kd 1.5/1.5/0.8/0.8 (`--kp/--kd` per group). Physics is 1 ms × 5
  per 5 ms tick, so states stay at motor_service's 200 Hz; scene.xml says 2 ms. The IMU
  is the MJCF's chip-frame sensors through `imu_protocol`'s AXIS_REMAP (new public
  `board_to_base`, `board_quat_to_base`), so the deployment remap is exercised too.
  Keyframe start (`--start home|straight`); disabled = actuation off = limp. The
  default (stand-policy) model is unchanged apart from the two fixes below.
- **`--gantry`:** four ropes (limited spatial tendons) from fixed anchors to the
  body's top face, hanging the robot 3 cm above the floor with straight legs. ctl
  `gantry` lowers (`lower=<m>`), sets (`length=`) or `release`s them (`dodo_ctl.py
  --arg key=value`, new). `status` has a `sim` section: base height, tilt, gantry, and
  the worst height/tilt since enable.
- **Rehearsed with the real CLI processes:** hang limp (0.476 m, 4 ropes taut) →
  `dodo_startup.py --pose home` → `home` in the air (0.002 rad error) → lower 12 cm
  (ropes slack) → release → stands at 0.412 m, 3.6° forward lean (5.8° at most on
  landing), up to 0.05 rad knee sag under load. That is `pd_stand.py`'s result.
  The park after release ends limp on the floor (base 0.18 m): on the robot, re-take
  the weight first. `test_sim_scene.py` covers both rehearsals (floor and gantry).
- **Found: re-targeting a standing robot at its MEASURED angles tips it over.** Under
  load the joints sag below their PD targets (0.2–0.7°; ankles most). Commanding the
  sagged angles drops that preload, and the robot falls forward with no ramp at all.
  `dodo_startup` did exactly that (its ramp started from `states.pos`), and so did the
  sim's `enable` when already enabled. Fixed: both services report each joint's held
  setpoint as `target_rad` in `status` (motor_service: status-only, set where it
  already sets the setpoints, None while disabled/idled; tested on the virtual bus),
  `dodo_startup` ramps and parks from those, and a second `enable` is a no-op in the
  sim too (as in motor_service). On the gantry in the air the two agree, which is why
  the gantry procedure never showed it.
- **Found: the PD stand has little forward margin.** Settled, the whole-robot COM sits
  22 mm behind the toe edge in `straight` (foot 95 mm), and 38 mm from the toe / 57 mm
  from the heel in `home`. `home` survives a 0.1 s push of 6 N forward and 10 N back.
  +0.03 rad more toe-down at the ankles moves it to 56/39 mm (10 N forward, 6 N back);
  about +0.015 would center it. The real COM is CAD's (no cables; never weighed), so
  the real margin is unknown: `home` rather than `straight` for the first ground
  test, and watch which way it wants to tip.
- **Clamps (both models):** position commands now stop `fleet_config.STOP_MARGIN_RAD`
  (0.10) inside the measured stops, in sim radians (`real2sim.stops/command_band`). The
  old `JOINT_LIMITS_RAD × 0.9` was July's raw frame and clamped `home`'s
  upper_leg_right 0.45 to 0.26. motor_service still clamps the old way until step 5.
- `dodo_startup.py --pose <scene keyframe | default>` (default `home`, read from
  scene.xml by `assets.scene_pose`, no MuJoCo needed). Suite: 189 passed, 1 known
  failure, 1 skip.

### 2026-11-05 (d) — the robot model lives only in `dodo_assets`; `description/` deleted
Decided by the user: one robot model, in `Atarilab/dodo_assets`, as a step toward one
Atarilab repo.
- **`dodo_assets` (the user's fork, branch `imu-exact-fit`, commit `40033e4`; PR to
  Atarilab pending):** `imu_mount.json` is our 11-04 exact refit (it had the 11-02
  snapped record). The MJCF's `imu_link` body is now generated between `BEGIN/END
  imu_locate.py` markers like the URDF block, so one `imu_locate.py apply` writes the
  record's pose into both models, the README's two IMU rows (the hand-written
  square-mount matrix is gone) and our `AXIS_REMAP` (unchanged). Its
  `scripts/check_models.py`: OK. `pd_stand.py`: unchanged (the body-frame IMU readings
  are identical; only the chip-frame ones moved).
- **This repo:** `assets.py` holds every model path (sibling `../dodo_assets`, or
  `DODO_ASSETS`), with a clear error if it's missing. Nothing checks it at import, so
  `motor_service` and the bench tools don't need the checkout. `sim_env.URDF_PATH`
  re-exports it. `imu_locate.py` writes and checks the MJCF too (new `MJCF` check). Its
  MJCF/README defaults come from next to the URDF it's given, so a copy of the URDF
  can't write into another checkout's files. That nearly happened in this session: a
  stale test run wrote a fake mount into the real MJCF before the commit was checked;
  restored from the commit, and a test now guards it. `description/` deleted (git rm);
  its README had nothing `dodo_assets`' README lacks, except a "not verified on hardware"
  list that was out of date. Docs: README (*Machine setup*, *Tests*, Files),
  `docs/imu_locate.md`.
- Tests: `test_imu_locate.py` works on copies of `dodo_assets`' layout and passes every
  path. New: a stale MJCF is caught, the URDF and MJCF compile to the same IMU frame
  (to 1e-9, as `check_models.py` demands), the defaults stay in the URDF's checkout,
  and missing markers are refused. Suite: 181 passed, 1 known failure, 1 skip.
  `dodo_assets` stays clean through the full suite.
- The sim still builds its model from the URDF with the old stand policy's actuators
  (KP 32/KD 3, 5 ms). Loading `scene.xml`'s PD actuators for the stand is the next step
  (log 11-05 (c)).

### 2026-11-05 (c) — Shafeef's `dodo_assets`; PD stand in sim; plan to the robot
- **`Atarilab/dodo_assets`** (Shafeef), cloned next to this repo: the same URDF and
  meshes as `description/`, plus an MJCF, `scene.xml` (floor, PD position actuators,
  `home`/`straight` keyframes) and `scripts/pd_stand.py`. Its `imu_mount.json` is the
  11-02 *snapped* record, not our 11-04 exact refit (4.4° apart): irrelevant for the PD
  stand, sync before a policy. Its joint order is URDF order (right leg first), ours
  is Isaac Lab's.
- **PD stand in sim works** (`pd_stand.py --headless`, MuJoCo 3.17): kp 150/150/50/50,
  kd 1.5/1.5/0.8/0.8 (hip, upper_leg, knee, ankle), armature 0.005 (placeholder),
  implicit position actuators at 2 ms. `home` = 0, 0.45, −1.0, 0.55: stays up, 5.8° max
  tilt, knee tracking −2.8°. Peak torques: knee 2.9 Nm, ankle 1.3, hips ~1. With a
  5 N push: 6.8°. `straight`: 3.4° tilt, knee only 0.65 Nm. The `home` pose is inside
  every measured stop.
- **Our `sim_service` can't run those gains as is:** a software PD at a 5 ms step
  (KP 32/KD 3). `pd_stand.py`'s note says a Python-side PD at these gains is unstable.
  The motors run their PD inside the drive at kHz rates, so implicit position
  actuators are also the closer model of the hardware.
- **Gains on the hardware:** Damiao MIT kp/kd are output-side Nm/rad and Nm·s/rad, so
  150/1.5 go in as they are (range 0–500 / 0–5). The ODrive is a cascaded P-PI. For a
  PD, set `vel_integrator_gain` 0; then joint K = N²·pos_gain·vel_gain/2π and D =
  N²·vel_gain/2π. For K 50 / D 0.8: knee (N 25.6) vel_gain 0.0077 Nm/(turn/s), ankle
  (28.5) 0.0062, pos_gain 62.5 for both. That assumes `motor.config.torque_constant` =
  8.27/300. Whether pos_gain 62.5 runs without buzz depends on the encoder: find out
  in the air. To verify: firmware 0.6's CAN Simple may have Set_Pos_Gain/Set_Vel_Gains
  (0x1A/0x1B), which would let `motor_service` set the gains instead of USB.
- **Thermal:** 2.9 Nm at the knee is ~0.11 Nm at the MN4004, ~4 A held continuously.
  Check the drives' `current_lim` and the motor temperature on the first ground test.

### 2026-11-05 (b) — main pass + `--limits` done (user); the numbers reviewed
`joint_calib.py`: the main pass is in `b76ad25`, the stops were uncommitted when this was
written. Claude reviewed them against the URDF, the CAD meshes, July's raw ranges and a
passive bus snapshot. Nothing was commanded.
- **Consistent:** every joint is driven by the motor of the same name, so the ODrive
  node → joint map (3/4/7/8 = knee L, ankle L, knee R, ankle R) is now measured, not
  July's word. Every entry round-trips raw ↔ sim exactly and carries today's ratios.
  The ankles agree L/R within 0.6° at both stops and turn 13.32/13.28 motor turns stop
  to stop (July: 13.40/13.35), so those are repeatable metal stops; 168° of travel.
  The knee's + stop (foot back) agrees L/R within 1.7°. Knee and ankle ranges have the
  URDF's lopsided shape, so the front/back sense is right in absolute terms, not just
  L vs R. Hip pitch is +1/−1 (mirrored), as the mirrored CAD hip links predict, and
  the two ranges agree under those signs. The default crouch sits ≥ 32° inside every
  stop. Live (passive candump): all ODrives IDLE, axis_error 0; the hanging right leg
  reads ≈ 0 on all four joints. Suite: 174 passed, 1 skip (known failure deselected).
- **Suspect: `hip_right` sign −1.** Identical motors on a shared +x axis in mirrored hip
  assemblies (the CAD hip links are exact y-mirrors) should read with the same sign,
  and July measured +1/+1. Today's motor frames match July's (one stop each, within
  0.1° and 1.7°). July's raw ranges are mirror-symmetric only with equal signs (out
  34°/36°, in 58°/58°); with +1/−1 the right hip would reach 58° out against the
  left's 34°. With equal signs, the stops that repeat since July are both outward,
  and the ones that changed are both inward (today 42°/36°: the legs meeting?).
  Today's right-hip range is symmetric about zero (±36°), so it can't decide, and
  `--verify` can't either (it uses the same mental model as the nudge). Robot-left vs
  viewer-left already tripped the 11-04 IMU check. **Confirmed by the user in the
  visualizer; fixed:** `hip_right` sign +1, stops negated and swapped (−0.631 … +0.634).
  The zero and the raw stops are unchanged: q = 0 at the pose, so `zero` doesn't
  depend on the sign. Written through `calibrate_joints.write_calib`.
- **Bug in the zero-pose instructions (Claude, 11-04): URDF q = 0 is NOT a straight
  leg.** The knee axis sits 2 cm in front of the hip–ankle line: the thigh leans 5.7°
  forward and the shank 5.7° back (the meshes' long axes: 4.8°/3.8°). `POSE_HOWTO`
  said "hip, knee and ankle axes on ONE vertical line". That pose is upper_leg +0.100,
  knee −0.200, ankle +0.100 with soles level. A zero taken there biases those
  readings by that much (the crouch comes out deeper; the feet stay flat, since the
  errors sum to 0). **Resolved:** the user's side photo of the zero pose matches a
  render of the URDF at q = 0 (straight thigh and shank sections parallel, the knee
  housing jogged forward), not the axes-in-line render. The zero is good. The
  instruction text still needs fixing.
- **Asymmetries to look at:** knee fold stop L −159° vs R −169° (18.28 vs 19.11 motor
  turns); thigh forward stop L −51° vs R −63°. The knee fold stop is the planned
  per-session ODrive reference (BOOT_POSE knees at `sim_lo`), so test its
  repeatability before relying on it. The knee's + stop agrees L/R and is what `--home`
  uses; pick one convention.
- The real stops are tighter than the training URDF in two places: thigh forward
  (−0.90/−1.10 vs −1.57) and knee fold (−2.78/−2.96 vs −3.14). Give Shafeef the measured
  stops once the two checks above are settled.
- `fleet_config.JOINT_LIMITS_RAD`/`BOOT_POSE_RAD` still hold raw-frame/URDF numbers.
  Don't copy the sim-frame stops in before `motor_service` applies sign/zero (§2 step
  5); do both in one change.

### 2026-11-05 — gear ratios measured; persistence verdict (robot session, user)
- **"IMU not responding" = permissions, not the IMU.** None of the user's terminals had
  `dialout` (the desktop login predates `usermod -aG dialout`; a new terminal doesn't
  pick it up, only a fresh login). The tools said "no IMU response… is something holding
  the port?". `open_imu` now says "permission denied … log out and back in / `newgrp
  dialout`". The board itself passed `imu_check` (0.39 % off 1 g, 67 Hz). ModemManager
  is active but doesn't probe it (CDC protocol 0, no modem found).
- **Firmware v2's range readback is intermittently 0** ("VER 2 0 1000", then "VER 2 4
  1000" on the next query), long after boot, not only right after it as its comment
  says. `query_version` gave up on a 0, and every tool then used the v1 guesses (2× too
  large for v2, gyro included). It now retries a 0, and if it persists uses the ranges v2
  configures (±4 g / ±1000 °/s), flagged as not read back (`imu_check`'s SCALE verifies).
- Tests no longer depend on the live `GEAR_RATIO` (they broke once real ratios were in).
  Suite: 174 passed, 1 known failure, 1 skip.
- **Gear ratios** (`calibrate_joints.py --ratio`, phone on the moving link): knees
  **25.6**, ankles **28.5**, in `fleet_config.GEAR_RATIO` (same value left and right). In
  the visualizer the knees/ankles now turn by about the right amount (full `--verify`
  pending). Against July's stop-to-stop turns that means ~169° of real ankle travel
  (URDF: 150°) and ~200° for the left knee (URDF: 260°).
- **Persistence (`--check-persistence`): Damiao zeros survive a power cycle; ODrive
  zeros don't** (boot-relative), as expected. So the ODrive joints need a reference pose
  every session; the user proposes a "tuck" against the hard stops (§3).
- **The left leg reads inverted** in the visualizer (identity map, no `joint_calib.py`
  yet): expected to come out as sign −1 in the main pass. Mind hip roll: July found +1 on
  both hips, and their ranges are symmetric with +1/+1 (`fleet_config.py`).
- **To check before trusting the ODrive torque IDLE:** `OdriveCanJoint` reads Get_Torques'
  estimate, which is MOTOR-side Nm (Iq × torque_constant) unless the drives'
  `motor.config.torque_constant` was set to include the gearing (8.27/KV = 0.0276 Nm/A
  means motor-side). `motor_service` compares it with 0.8 × 9 Nm, a JOINT torque: at
  25–28:1 that check can't fire. Fix with the calibration wiring (§2 step 5).

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
- Group changes (`dialout` for the Nicla) apply to sessions started after a fresh
  login: log out of the desktop and back in. Until then `newgrp dialout` per terminal.
- CAN bitrates are set at OS level, never by python-can: `setup/99-dodo-can.rules`
  brings both buses up at 1 Mbit when the adapters appear (by hand: `ip link`).
- `can0`/`can1` are enumeration order, not identity: use the udev names
  `can_dm`/`can_od`.
- The direct-CAN tools (`read_motor_state`, `calibrate_joints`, `visualize_hardware`,
  `odrive_can_scan`) and `motor_service` all open the buses: run one at a time. The
  Nicla's port, likewise: `imu_service`, `imu_check`, `imu_locate`, `imu_sniff` and
  `visualize_hardware` (unless `--no-imu`).
- Pausing the policy runner (a debugger, a viewer) stops the command stream, and
  `motor_service` e-stops after 0.5 s by design. `dodo_ctl estop_reset` recovers.
