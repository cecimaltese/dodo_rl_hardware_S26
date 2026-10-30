# The ZMQ framework: topics, schemas, timing

The services, `ZmqEnv` and the tools talk over ZMQ. This is the spec. Everything in it is
defined once in **`dodo_ipc.py`**, which every side imports. Built on `zmq_dev`
(2026-07-06), reconciled with main's July work on 2026-10-30. Verified in sim (numbers in §6), not yet on
the robot; status and next steps are in the README and HANDOFF.

## 1. Why ZMQ, and what it replaced

- No ROS2 install needed on the robot or on dev laptops (the rclpy nodes didn't even import
  on the Mac venv). `pip install pyzmq` and done.
- Same mental model as ROS: named topics, publishers, subscribers. PUB/SUB sockets replace
  ROS topics; one small REQ/REP socket replaces `ros2 service call` for rare operations
  (enable, disable, calibrate, e-stop reset).
- It was a transport port, not a redesign: `DamiaoJoint`, `OdriveCanJoint`, the per-type
  CAN sockets, the safety layer, the `Env` base and policy contract in `rl_env.py`,
  `run_policy.py`'s loop and the sim2sim consistency tests carried over unchanged.

## 2. Architecture

Every box is its own OS process (its own GIL, which alone helps real-time). Services bind
their sockets; clients connect. Swap the two robot services for `sim_service.py` and
nothing else changes. That is the sim2sim = sim2real guarantee.

```
              REAL ROBOT                                    POLICY SIDE
┌─────────────────────────────────────┐
│ motor_service.py (can_dm + can_od)  │   PUB  motor_states  ─────►  ┌──────────────────────┐
│  Damiao hips/upper (rad, hw zero)   │   SUB  motor_commands ◄────  │ run_policy.py        │
│  ODrive knees/feet (turns, bootcal) │   REP  ctl            ◄───►  │  --backend zmq       │
│  safety: clamps, torque→IDLE, estop │                              │  (ZmqEnv: assembles  │
├─────────────────────────────────────┤                              │   OBS_STAND, 50 Hz,  │
│ imu_service.py    (owns serial)     │   PUB  imu           ─────►  │   ONNX policy)       │
│  gyro + accel + fused quat          │                              └──────────────────────┘
└─────────────────────────────────────┘                                   ▲ tools connect too:
              — OR —                                                      │ echo / hz / ctl
┌─────────────────────────────────────┐                                   ▼
│ sim_service.py (MuJoCo, real-time)  │   same 4 endpoints          zmq_echo.py, zmq_hz.py
└─────────────────────────────────────┘
```

## 3. Topics and endpoints

One endpoint **per topic**; this is what makes latest-value delivery trivial (§5).
Defaults below, all overridable via `DODO_ZMQ_HOST` and per-topic environment variables.

| Topic            | Pattern  | Binds            | Default endpoint    | Rate    | Content |
|------------------|----------|------------------|---------------------|---------|---------|
| `motor_states`   | PUB→SUB  | motor/sim service| `tcp://*:6001`      | 200 Hz  | pos/vel/torque per joint (rad, rad/s, Nm) + per-joint & global flags |
| `motor_commands` | PUB→SUB  | motor/sim service (SUB binds) | `tcp://*:6002` | 50 Hz | absolute position targets (rad), canonical joint order |
| `imu`            | PUB→SUB  | imu/sim service  | `tcp://*:6003`      | ≤100 Hz | gyro (rad/s), accel (m/s²), quat (flag-gated) |
| `ctl`            | REQ↔REP  | motor/sim service| `tcp://*:6000`      | on demand | `calibrate`, `enable`, `disable`, `set_zero`, `estop_reset`, `status` |
| `telemetry`      | PUB→SUB  | (reserved)       | `tcp://*:6004`      | —       | future: debug/diag stream, recorder |

**Services bind, clients connect**, including `motor_commands`, where the *subscriber*
(motor_service) binds and the policy runner connects its PUB. The robot is the stable
address; runners, tools and laptops come and go. (PUB-connect/SUB-bind is fully supported
by ZMQ.)

## 4. Message schemas (wire format)

Binary `struct`-packed, single-part messages (required for latest-value mode, §5). No
dependencies beyond `pyzmq`. The exact struct strings live in `dodo_ipc.py`.

**Common header (every message):** `schema_version:u16`, `msg_type:u16`, `seq:u32`,
`t_mono_ns:i64` (the sender's monotonic clock: for age/latency checks, never for control
math).

**`motor_states`**, all joint arrays in **canonical joint order** (§4a), length 8:
- `pos[8]:f32` rad · `vel[8]:f32` rad/s · `torque[8]:f32` Nm
- `joint_flags[8]:u8`: bit0 `has_feedback`, bit1 `enabled`, bit2 `idled_by_safety`,
  bit3 `calibrated` (zero valid, see the boot-pose calibration), bit4 `fault`
- `global_flags:u8`: bit0 `estopped`, bit1 `all_calibrated`, bit2 `enabled`

**`motor_commands`:** `target_pos[8]:f32` rad, canonical order. Absolute targets: the env
already resolves `default_pos + ACTION_SCALE·action`, keeping policy knowledge out of the
controller. NaN means "no command" for that joint.

**`imu`:** `gyro[3]:f32` rad/s · `accel[3]:f32` m/s² · `quat[4]:f32` wxyz ·
`lin_vel[3]:f32` · `flags:u8` (bit0 `quat_valid`: when 0, ZmqEnv falls back to the
accelerometer's gravity direction; bit1 `lin_vel_valid`: set by sim_service only, never by
hardware).

**`ctl`** (REQ/REP): tiny JSON (`{"cmd": "enable"}` → `{"ok": true, ...}`). Rare and
human-triggered, so debuggability beats bytes here.

### 4a. Canonical joint order

Joint names stay **off the wire**. `dodo_ipc.JOINT_ORDER` equals `rl_env.POLICY_JOINT_ORDER`
(type-grouped, left before right, empirically pinned by `test_sim2sim_consistency.py`);
`test_ipc.py` fails if the two ever diverge:

```
hip_left, hip_right, upper_leg_left, upper_leg_right,
lower_leg_left, lower_leg_right, foot_left, foot_right
```

motor_service maps its fleet into this order once at startup. `schema_version` bumps if
the order ever changes, so a stale process fails loudly instead of scrambling legs.

## 5. Latest-value semantics (the ZMQ gotcha that matters)

A 50 Hz control loop must always act on the **newest** state, never on a backlog. The rules,
encoded once in `dodo_ipc.Subscriber`:

- One topic per socket plus **single-part** messages make it safe to set `ZMQ_CONFLATE=1`
  on SUBs (the kernel keeps only the newest message). CONFLATE is broken with multipart and
  multi-topic sockets, which is why §3 gives every topic its own endpoint.
- Belt and braces: `Subscriber.latest()` also drains with `NOBLOCK` until empty.
- Consumers check message **age**. ZmqEnv stamps a message when it drains it (sender clocks
  aren't comparable across hosts), so stale `motor_states` or `imu` (> 0.25 s) make it
  refuse to step. A message still parked in the conflated queue when a service dies is
  stamped fresh, which under continuous 50 Hz stepping delays detection by one step.
  motor_service e-stops on its own watchdogs (`CMD_WATCHDOG_S`, `FB_WATCHDOG_S`).
- Publishers set a small `SNDHWM` and `LINGER=0`.
- Pausing the policy runner (a debugger, a viewer) stops the command stream, and
  motor_service e-stops after 0.5 s, by design. `dodo_ctl estop_reset` recovers without a
  restart.

## 6. Real-time sim2sim

`sim_env.py` is **lockstep**: `step()` blocks, and physics advances only when the policy
calls it. The robot doesn't behave like that: motors move and the IMU streams whether or
not the policy is on time, so lockstep sim can't rehearse latency or jitter at all.

**`sim_service.py`** runs MuJoCo as a *virtual robot* process instead:

- Its own clock: an absolute-deadline loop stepping physics at `SIM_DT` = 5 ms wall time,
  applying the software PD (KP 32 / KD 3, effort clamps) toward the **latest**
  `motor_commands` target each step, with the same position clamps as motor_service.
- Publishes `motor_states` at 200 Hz and `imu` at 100 Hz from the sim sensors; answers `ctl`.
- Logs overruns instead of silently slowing down.
- Optional viewer that never blocks the physics/publish loop; optional latency and drop
  injection (`--extra-latency-ms`, `--drop`) and `--imu-no-quat`.

`run_policy.py --backend zmq` then cannot tell sim from robot. The in-process
`--backend sim` stays for fast headless checks and the consistency tests; it is not the
real-time path and doesn't pretend to be.

**Measured on the dev Mac (2026-07-06), separate processes over localhost TCP:**
`motor_states` 200.0 Hz and `imu` 100.0 Hz with 0 drops; the policy loop at exactly
50.0 Hz with 0 overruns; state age at read 0.21 ms on average (max 0.66 ms); sim time equal
to wall time over 30 s, with 0.08 % overruns (worst 12.8 ms). On 2026-10-30 the stand
policy held the robot model upright for 10 s through this stack
(`test_e2e_zmq.py::test_stand_policy_over_zmq`).
