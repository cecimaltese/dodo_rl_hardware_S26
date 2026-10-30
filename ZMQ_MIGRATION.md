# ZMQ Real-Time Framework — Topic Spec + Migration Roadmap

> Status: **PHASES 0–2 IMPLEMENTED & TESTED ON DEV (2026-07-06).** Written after a
> full deep-dive of `dodo_rl` + `dodo_rl_hardware_S26`; implemented the same day.
> We deliberately chose **ZMQ over ROS2** — this was a transport port, not a
> redesign: the safety layer, joint drivers, and Env/policy contract survived
> unchanged. The old ROS2 nodes are frozen in `legacy_ros2/`.
>
> **Implemented:** `dodo_ipc.py` (schemas/sockets/rate loop, THE contract file),
> `motor_service.py`, `imu_service.py`, `zmq_env.py` (+ `run_policy.py --backend
> zmq`), `sim_service.py`, `demo_combined.py` (ZMQ bench client),
> `tools/{zmq_echo,zmq_hz,dodo_ctl}.py`, tests `test_ipc.py` + `test_e2e_zmq.py`
> (18 tests, all green on macOS dev; 1 skip = URDF acceptance, runs on Ubuntu).
>
> **Measured on dev (separate processes, localhost tcp):** motor_states 200.0 Hz /
> imu 100.0 Hz, 0 drops; policy loop exactly 50.0 Hz, 0 overruns; state age at
> read avg 0.21 ms (max 0.66 ms); sim time == wall time over 30 s with 0.08 %
> overruns (worst 12.8 ms). All inside the Phase-2 budget.
>
> **Still open:** Phase 3 (hardware bring-up on the Ubuntu box: bench run of
> motor_service, limits/gains) and Phase 4 (walking retrain). §7/§8 below
> updated accordingly.
>
> **Update 2026-07-06 (later):** IMU firmware is now COMMITTED (`imu.cpp`,
> Nicla Sense ME) and `imu_service.py` parses its real protocol (poll `'L'` →
> `ALL` line, raw LSB → SI); an upgraded streaming firmware with the BHI260's
> fused quaternion is provided in `firmware/imu_stream_nicla/`. **Homing is
> officially replaced by boot-pose calibration** (power on with the ODrive
> joints folded at their stops → `ctl calibrate`), with a bumpless fail-closed
> `enable` + auto-unpark in motor_service and a pre-start ramp to the default
> pose in ZmqEnv (`run_policy --startup-ramp`, rehearsal: `tools/dodo_startup.py`).
> Tests: 37 green + 1 URDF skip (`test_imu_service.py`, `test_boot_calibration.py`
> added).

---

## 1. Why ZMQ, and what stays

- No ROS2 install needed on the robot (or on dev laptops — the current rclpy files
  don't even import on the Mac venv). `pip install pyzmq` and done.
- Same mental model: named topics, publishers, subscribers. PUB/SUB sockets replace
  ROS topics; one small REQ/REP socket replaces "ros2 service call" for rare ops
  (enable/disable/zero/e-stop-reset).
- **Unchanged:** `DamiaoJoint`, `OdriveCanJoint`, the CAN topology (one filtered
  socket per motor type on `can0`), the entire safety layer in `motor_controller.py`,
  the `Env` base + policy contract in `rl_env.py`, `run_policy.py`'s loop, and the
  sim2sim consistency tests.

## 2. Target architecture

Every box is its **own OS process** (own GIL — this alone helps real-time). Services
bind their sockets; clients connect. Swap the two robot services for `sim_service.py`
and *nothing else changes* — that is the sim2sim = sim2real guarantee.

```
              REAL ROBOT                                    POLICY SIDE
┌─────────────────────────────────────┐
│ motor_service.py  (owns can0)       │   PUB  motor_states  ─────►  ┌──────────────────────┐
│  Damiao hips/upper (rad, hw zero)   │   SUB  motor_commands ◄────  │ run_policy.py        │
│  ODrive knees/feet (turns, bootcal) │   REP  ctl            ◄───►  │  --backend zmq       │
│  safety: clamps, torque→IDLE, estop │                              │  (ZmqEnv: assembles  │
├─────────────────────────────────────┤                              │   OBS_STAND, 50 Hz,  │
│ imu_service.py    (owns serial)     │   PUB  imu           ─────►  │   ONNX policy)       │
│  gyro + accel (+ quat if fused)     │                              └──────────────────────┘
└─────────────────────────────────────┘                                   ▲ tools connect too:
              — OR —                                                      │ echo / hz / record
┌─────────────────────────────────────┐                                   ▼
│ sim_service.py (MuJoCo, real-time)  │   same 4 endpoints          zmq_echo.py, zmq_hz.py
└─────────────────────────────────────┘
```

## 3. Topics / endpoints

One endpoint **per topic** (this is what makes latest-value delivery trivial, see §5).
Defaults below; all overridable via `DODO_ZMQ_HOST` / per-topic env vars. Everything
is defined once in **`dodo_ipc.py`** — the single shared module both sim and real import.

| Topic            | Pattern  | Binds            | Default endpoint    | Rate    | Content |
|------------------|----------|------------------|---------------------|---------|---------|
| `motor_states`   | PUB→SUB  | motor/sim service| `tcp://*:6001`      | 200 Hz  | pos/vel/torque per joint (rad, rad/s, Nm) + per-joint & global flags |
| `motor_commands` | PUB→SUB  | motor/sim service (SUB binds) | `tcp://*:6002` | 50 Hz | absolute position targets (rad), canonical joint order |
| `imu`            | PUB→SUB  | imu/sim service  | `tcp://*:6003`      | ≥100 Hz | gyro (rad/s), accel (m/s²), quat (optional, flag-gated) |
| `ctl`            | REQ↔REP  | motor/sim service| `tcp://*:6000`      | on demand | `calibrate`, `enable`, `disable`, `set_zero`, `estop_reset`, `status` |
| `telemetry`      | PUB→SUB  | (reserved)       | `tcp://*:6004`      | —       | future: debug/diag stream, recorder |

**Convention: services bind, clients connect** — including `motor_commands`, where the
*subscriber* (motor_service) binds and the policy runner connects its PUB. The robot is
the stable address; runners/tools/laptops come and go. (PUB-connect/SUB-bind is fully
supported by ZMQ.)

## 4. Message schemas (wire format)

Binary `struct`-packed, single-part messages (required for latest-value mode, §5).
No new deps beyond `pyzmq`. Exact struct strings live in `dodo_ipc.py`; field-level spec:

**Common header (every message):** `schema_version:u16`, `seq:u32`, `t_mono_ns:i64`
(sender monotonic clock — for age/latency checks, never for control math).

**`motor_states`** — all joint arrays in **canonical joint order** (§4a), length 8:
- `pos[8]:f32` rad · `vel[8]:f32` rad/s · `torque[8]:f32` Nm
- `joint_flags[8]:u8` — bit0 `has_feedback`, bit1 `enabled`, bit2 `idled_by_safety`,
  bit3 `calibrated` (zero valid — see §7 boot-pose calibration), bit4 `fault`
- `global_flags:u8` — bit0 `estopped`, bit1 `all_calibrated`

**`motor_commands`:**
- `target_pos[8]:f32` rad, canonical order. Absolute targets — the env already resolves
  `default_pos + ACTION_SCALE·action` (keep policy knowledge OUT of the controller).

**`imu`:**
- `gyro[3]:f32` rad/s · `accel[3]:f32` m/s² · `quat[4]:f32` wxyz · `flags:u8`
  (bit0 `quat_valid` — when 0, ZmqEnv falls back to accel gravity, same logic as
  today's `real_env._on_imu`).

**`ctl`** (REQ/REP): tiny JSON (`{"cmd": "enable"}` → `{"ok": true, ...}`) — rare,
human-triggered, debuggability beats bytes here.

### 4a. Canonical joint order — one constant, zero remapping

Joint names go **off the wire**. `dodo_ipc.JOINT_ORDER` = the existing
`POLICY_JOINT_ORDER` (type-grouped, left-before-right — empirically pinned by
`test_sim2sim_consistency`):

```
hip_left, hip_right, upper_leg_left, upper_leg_right,
lower_leg_left, lower_leg_right, foot_left, foot_right
```

`rl_env.py` imports it from `dodo_ipc` instead of defining its own; motor_service maps
its fleet into this order once at startup. The by-name remapping in `real_env._on_states`
disappears. `schema_version` bumps if the order ever changes, so a stale process fails
loudly instead of scrambling legs.

## 5. Latest-value semantics (the ZMQ gotcha that matters)

A 50 Hz control loop must always act on the **newest** state, never on a backlog.
Rules, encoded once in `dodo_ipc.Subscriber`:

- One topic per socket + **single-part** messages → safe to set `ZMQ_CONFLATE=1` on
  SUBs (kernel keeps only the newest message). *(CONFLATE is broken with multipart
  and multi-topic sockets — that's why §3 gives every topic its own endpoint.)*
- Belt-and-suspenders: `Subscriber.latest()` also drains with `NOBLOCK` until empty.
- Every consumer checks message **age** via `t_mono_ns` (clocks: same host → directly
  comparable; cross-host → compare arrival-time deltas instead). Stale `motor_states`
  or `imu` (> ~100 ms) → ZmqEnv refuses to step / motor_service watchdog e-stops,
  exactly like today's `FB_WATCHDOG_S` / `CMD_WATCHDOG_S`.
- Publishers set small `SNDHWM`, `LINGER=0`.

## 6. The sim2sim real-time fix

Today `sim_env.py` is **lockstep**: `step()` blocks, physics advances only when the
policy calls it, and wall-clock pacing exists only when the viewer is open. That is
not how the robot behaves — motors move and the IMU streams whether or not the policy
is on time. Hence "sim2sim doesn't feel real time" and, worse, it can't rehearse
latency/jitter at all.

**Fix: `sim_service.py`** — MuJoCo as a *virtual robot* process:

- Own clock: absolute-deadline loop (the `_wall_next += dt` pattern already in
  `sim_env.py`) stepping physics at `SIM_DT`=5 ms wall time, applying the software PD
  (KP 32 / KD 3, effort clamps) toward the **latest** `motor_commands` target each step.
- Publishes `motor_states` @200 Hz and `imu` @100 Hz from the sim sensors; answers `ctl`.
- Logs overruns (an 8-DOF MuJoCo model steps in ~µs — real-time is easy; if it ever
  overruns we *see* it instead of silently slowing down).
- Optional viewer; rendering must never block the physics/publish loop (render thread).

`run_policy.py --backend zmq` then cannot tell sim from robot. Keep the in-process
`--backend sim` for fast headless checks and the consistency tests — it's not the
real-time path and doesn't pretend to be.

## 7. Roadmap (fastest path to standing, then walking)

### Phase 0 — `dodo_ipc.py` + tools ✅ DONE 2026-07-06 (all tests green)
1. `dodo_ipc.py`: endpoints registry, schemas + pack/unpack, `Publisher` /
   `Subscriber(latest())` / `CtlServer` / `CtlClient`, `JOINT_ORDER`, rate-loop helper
   (absolute deadlines, monotonic clock, overrun counter).
2. `tools/zmq_echo.py` (decode + print any topic), `tools/zmq_hz.py` (rate + age +
   jitter). These are our `ros2 topic echo/hz` — build them first, debug everything else with them.
3. `pytest test_ipc.py`: pack/unpack round-trip, conflate/latest behavior, loop timing.
- **Accept:** two dummy processes exchange 200 Hz states / 50 Hz commands on one
  machine; `zmq_hz` shows stable rates and age < 5 ms.

### Phase 1 — port the three nodes ✅ CODE DONE 2026-07-06 (bench HIL still pending)
1. `motor_controller.py` → **`motor_service.py`**: delete rclpy; main loop = 200 Hz
   rate-helper; SUB `motor_commands` (latest), PUB `motor_states`, REP `ctl`. Safety
   layer (`_JointAdapter`, clamps, torque→IDLE, e-stop, watchdogs) moves over verbatim.
   Drop the dead commented-out `step()` block (that logic lives in `rl_env.Env` now).
   **Do not auto-enable on start** — require `ctl enable` (bring-up order becomes explicit).
2. `observer.py` → **`imu_service.py`**: same poll-serial→publish structure,
   `parse_imu_line()` stays the single firmware-specific edit point. *(Ask Jim to
   commit the IMU firmware; see §8.4 — streaming beats `'L'`-polling.)*
3. `real_env.py` → **`zmq_env.py`** (`ZmqEnv(Env)`): SUB states+imu (latest, with age
   check), PUB commands, `control_dt` pacing via the rate helper instead of bare
   `time.sleep`. `run_policy.py`: `--backend zmq` (alias `real`).
4. Fix `demo_combined.py` (currently imports `MotorControllerConfig`/`PolicyJoint`,
   which no longer exist — broken today): rewrite as a ZMQ client bench demo
   (`ctl enable` → publish slow sine targets → echo states). `read_motor_state.py`
   stays direct-CAN (pre-ZMQ bring-up tool, that's its job).
5. `requirements.txt`: add `pyzmq`, drop the ROS2 note.
- **Accept:** py_compile + Phase 0 tests green; bench (1 DM + 1 ODrive): `motor_service`
  + `demo` move both motors; `zmq_echo motor_states` shows sane rad values; unplugging
  a motor trips the feedback watchdog e-stop.

### Phase 2 — real-time sim2sim ✅ DONE on dev 2026-07-06 (URDF acceptance test
### ready in `test_e2e_zmq.py::test_stand_policy_over_zmq`, auto-skips until
### ../dodo_files exists — run it on the Ubuntu box)
1. **`sim_service.py`** per §6 (reuse `_build_model` + PD from `sim_env.py`).
2. End-to-end: `sim_service` + `run_policy --backend zmq --task stand` → robot stands
   in MuJoCo **through the network stack**.
3. Measure with `zmq_hz`: command→state round trip (< 10 ms target on one host),
   policy-loop jitter (< ±2 ms of the 20 ms period), zero missed deadlines over 60 s.
4. Optional but cheap and very useful: latency/drop injection flags in `sim_service`
   (`--extra-latency-ms 20 --drop 0.05`) — if the stand policy survives that, hardware
   timing noise won't be what kills it.
- **Accept:** 60 s stable stand via ZMQ pipeline + the timing numbers above. This is
  the go/no-go gate for hardware.

### Phase 3 — hardware bring-up + safety completion (bench/gantry days)
1. **Calibration — boot-pose convention (IMPLEMENTED 2026-07-06; replaces homing).**
   Decision: NO homing routine — driving the ODrive joints into their stops under
   power could overload the 3D-printed structure + belts. Instead:
   - **Power the robot ON with the ODrive joints folded all the way back against
     their mechanical stops** (`BOOT_POSE_RAD` in motor_service.py: knees −3.1416,
     ankles +1.57 — *placeholders until bench-verified*). `ctl calibrate` maps
     turns-since-boot onto those angles; joints report `calibrated` after it, and
     ZmqEnv refuses to run until `all_calibrated`.
   - `ctl enable` is bumpless + fail-closed: refuses silent joints, presets every
     setpoint at the measured position, then slowly *unparks* out-of-band joints to
     the safety-band edge (1.5 s). The pre-start routine (`run_policy
     --startup-ramp 5`, rehearse with `tools/dodo_startup.py`) then ramps to the
     default pose inside the commander process, so the watchdog stays fed.
   - Damiao (hips/upper): persistent hardware zero → verify zero = sim zero pose
     once, `set_zero_hardware()`, done. (`ctl set_zero` = jig fallback, bench only.)
   - **Bench TODO:** verify each `BOOT_POSE_RAD` sign/value (calibrate → `status`
     must show the stop angles; nudge each joint, check the sign), and whether the
     printed stops sit exactly at the URDF limits (±0.10 rad margin is covered).
   - **Verify ODrive gearing**: all turns↔rad math assumes direct drive (1 turn = 2π rad).
     If geared, commands AND limits divide by the ratio — check before trusting clamps
     (with wrong gearing the boot-pose *offset* is still right, but every *move*
     scales wrong — first unpark/ramp at low speed, hand on the kill switch).
2. **Damiao joint limits** (the open safety item): `JOINT_LIMITS_RAD` already carries
   URDF values for all 8 joints; knee/ankle (ODrive) are considered confirmed. Measure
   the true mechanical range of hip roll/pitch about the calibrated zero (limp mode via
   `read_motor_state.py`, move by hand, note extremes), update `JOINT_LIMITS_RAD`, and
   record the measured values here.
3. **Gains:** DM_JOINTS still kp=30/kd=0.5; trained policy expects the behavior of
   stiffness 32 / damping 3.0 (Damiao MIT units ≠ IsaacLab units — calibrate on the
   bench: command steps, match the response, don't literal-copy).
4. Full fleet on the gantry: `read_motor_state` sign/direction check per joint vs sim
   +angle convention → stand policy, hands hovering, then mild pushes.
- **Accept:** robot stands on the gantry via the identical pipeline that passed Phase 2.

### Phase 4 — walking + nice-to-haves
1. **Walking needs a retrain, not just ZMQ:** `dodo_flat` observes `base_lin_vel`,
   which is banned on hardware. Train a walk task with the stand-style obs **plus**
   `velocity_commands` (obs 33: ang_vel, gravity, commands, joint terms) — start now
   in `dodo_rl`, it runs in parallel with Phases 1–3. Deployment side is then just a
   small `velocity_commands`/teleop publisher (reserved topic, §3) and `OBS_WALK` in
   `rl_env.py`.
2. `tools/zmq_record.py` / `zmq_play.py` (dump topics to `.npz`) — post-mortems of
   every hardware run; replay IMU/state logs through the obs pipeline offline.
3. Later robustness retrain: `DelayedPDActuatorCfg` with 0–2 step delay (see
   `assets/dodo.py` note) once we've *measured* the real loop latency in Phase 3.

## 8. Known gaps & risks (carried from the deep dive)

1. ~~IMU firmware is uncommitted~~ **RESOLVED 2026-07-06:** `imu.cpp` (Nicla Sense
   ME, poll `'L'` → `ALL` line) is committed and `parse_imu_line()` matches it,
   converting the raw int16 LSB values to SI (BHI260AP defaults: 4096 LSB/g,
   16.384 LSB/°/s — **verify on the bench:** |accel| ≈ 9.81 at rest, 90° rotation
   integrates to 1.57 rad; also verify the mounting axes). Remaining recommendation:
   flash `firmware/imu_stream_nicla/` — continuous 100 Hz stream + the BHI260's
   **fused quaternion**, so `projected_gravity` stays correct while moving instead
   of falling back to quasi-static accel tilt.
2. ~~ODrive zero state~~ **RESOLVED (design) 2026-07-06:** boot-pose calibration,
   see Phase 3.1. Remaining risk is only the unverified `BOOT_POSE_RAD` stop
   angles/signs — wrong values shift every ODrive position by a constant; the
   over-travel e-stop (±0.10 rad) and the clamp band catch gross errors, the
   `status`-after-`calibrate` check catches the rest.
3. **Gain mismatch** (Phase 3.3) — known cause of the earlier sim2sim failure; will
   bite on hardware identically if skipped.
4. `dodo_files/urdf/dodo_daimao.urdf` is expected as a sibling of the repos
   (`../dodo_files/`) and is missing on at least one dev machine — commit it or
   document where it lives; `sim_env`/`sim_service`/tests need it.
5. Watchdog vs. viewer pauses: with `CMD_WATCHDOG_S = 0.5`, pausing the policy runner
   e-stops the fleet (by design). `ctl estop_reset` exists so recovery doesn't require
   restarting the service.
