<p align="center">
    <img <img width="1879" height="252" alt="TUM_mirmi" src="https://github.com/user-attachments/assets/b1440892-0ecc-4c05-9875-69f6e070998a" />
</p>

# 🦤 dodo_rl — Teaching a Dodo to Walk (and Backflip)

**Reinforcement learning locomotion for the Dodo bipedal robot, built on NVIDIA IsaacLab.**

> *"The original dodo went extinct because it couldn't run. We're fixing that — and adding a backflip for good measure."*

RL locomotion for the Dodo bipedal robot, built as an external IsaacLab project for year S26 team Maltese Invrea, Ghonim, Hucklenbroich, Pickrell.

---

## What Is This?

An IsaacLab project that trains the 8-DOF Dodo bipedal robot to walk (and eventually do acrobatics) using reinforcement learning in simulation (Isaac Sim), with the goal of sim-to-real transfer to the physical robot.

The robot learns entirely from scratch — no hand-crafted gaits, no motion capture. Just a reward signal that says *"go forward, don't fall"*, and 256 parallel robots stumbling their way to competence.

### The Pipeline

```
URDF → USD (Isaac Sim import) → RL Environment → PPO Training (rsl_rl) → Trained Policy → Real Robot
```

## Robot Specs

| Property | Value |
|---|---|
| Name | Dodo |
| Type | Bipedal |
| Total DOF | 8 revolute + 2 fixed (foot soles) |
| Mass | ~4.7 kg |
| Standing height | ~0.45 m |
| Hip motors | 27 Nm max torque |
| Knee/ankle motors | 9 Nm max torque |

# 🔧 Hardware — Sim-to-Real Motor Control

This is the **real-robot** half of the project (`dodo_rl_hardware_S26/`). The goal is a
clean motor-control + feedback stack so the policy trained in IsaacLab can drive the
physical Dodo through the exact same `step(action) → observation` contract used in sim.

## The two motor types

The robot uses two kinds of actuators, each spoken to over the **same CAN bus** but with
a different protocol.

| | **Damiao DM-J4310** | **T-Motor Antigravity MN4004 (KV300)** |
|---|---|---|
| Role | Integrated CAN servo (joints) | Gimbal BLDC, driven by an **ODrive** controller |
| Driver | Built-in (motor + driver in one) | External ODrive (FOC) + encoder |
| Protocol | Damiao MIT mode (CAN std frame) | ODrive **CAN Simple** |
| Native units | **radians** | **turns** (1 turn = 2π rad) |
| Rated / peak torque | 3 Nm / 7 Nm | low (gimbal motor) — gearing/ODrive dependent |
| Gear ratio | 10:1 | n/a (depends on assembly) |
| Feedback | dual 14-bit magnetic encoders; pos/vel/torque/temp over CAN | encoder estimates + torque over CAN |
| CAN bitrate | 1 Mbit/s | 1 Mbit/s |
| Datasheet | [DM-J4310](https://files.seeedstudio.com/products/Damiao/DM-J4310-en.pdf) | [MN4004 KV300](https://store.tmotor.com/de/product/mn4004-kv300-motor-antigravity-type.html) |

The Damiao is an all-in-one servo: send it an MIT-mode frame and it closes the loop
internally. The T-Motor is a "dumb" BLDC, so an **ODrive** does the field-oriented
control; we talk to the ODrive over CAN Simple. The key design choice is that **both
motor types expose the identical Python joint interface**, so one control loop drives
them uniformly — only the units differ.

The sim's 8 DOF map onto these actuators in two effort groups (see `dodo_rl/.../dodo.py`):
hip-roll + hip-pitch (27 Nm sim limit) and knee + ankle (9 Nm sim limit). Effort limits
are set per actuator group in the sim; the matching `rated_torque` per joint is used by
the (planned) real-robot safety layer to normalize torque.

## CAN bus architecture

Everything lives on one physical bus, **`can0` @ 1 Mbit/s**. SocketCAN hands each open
socket its *own copy* of every frame, so we give each motor type a **separately filtered
socket** — they never mis-ingest each other's frames, no driver patching needed:

```
                         ┌───────────────────────────── can0 @ 1 Mbit/s ─────────────────────────────┐
  MotorController         │                                                                            │
   ├─ DaMiaoController ───┤  socket #1: filter = {DM feedback ids, 0x7FF}   →  DamiaoJoint(s)  (rad)    │
   └─ can.Bus + reader ───┤  socket #2: filter = {node_id<<5, mask 0x7E0}   →  OdriveCanJoint(s)(turns) │
                          └────────────────────────────────────────────────────────────────────────────┘
```

CAN Simple arbitration id = `(node_id << 5) | cmd_id`; the reader thread routes each
inbound frame to the right joint by `node_id`. ODrive node ids are kept at **7/8** to
avoid colliding with Damiao command/feedback/register ids.

Bring the bus up first (OS level — not python-can):

```bash
sudo ip link set can0 up type can bitrate 1000000
```

## Files

The stack is **ZMQ-based** (we deliberately chose ZMQ over ROS2) — full topic
spec, rationale and status in **`ZMQ_MIGRATION.md`**.

| File | What it is |
|---|---|
| `dodo_ipc.py` | **The contract file**: topics/ports, binary schemas, canonical joint order, Publisher/Subscriber (latest-value), ctl REQ/REP, RateLoop. Everything imports this. |
| `motor_service.py` | The motor fleet as a ZMQ service: PUB `motor_states` @200 Hz, SUB `motor_commands`, REP `ctl` (calibrate/enable/disable/set_zero/estop_reset/status). Safety layer inside. Bumpless fail-closed enable + boot-pose calibration (`BOOT_POSE_RAD`). No auto-enable. |
| `imu_service.py` | IMU serial → PUB `imu`. Parses the real `imu.cpp` protocol (poll `'L'`, raw LSB → SI) and the stream-firmware format. |
| `imu.cpp` | The IMU firmware as flashed today (Nicla Sense ME, poll-based). |
| `firmware/` | `imu_stream_nicla/` — recommended firmware upgrade: 100 Hz stream + BHI260 fused quaternion (see `firmware/README.md`). |
| `sim_service.py` | **MuJoCo as a real-time virtual robot** on the same topics — sim2sim = sim2real rehearsal, incl. optional latency/drop injection. |
| `zmq_env.py` | `ZmqEnv(Env)` — the deployment backend for `run_policy.py --backend zmq`; can't tell sim_service from the real robot. |
| `rl_env.py` / `sim_env.py` / `run_policy.py` | Policy contract + in-process MuJoCo (fast checks) + the Env-agnostic runner. |
| `tools/` | `zmq_echo.py`, `zmq_hz.py` (rate/age/jitter/drops), `dodo_ctl.py` (bring-up CLI), `dodo_startup.py` (rehearse the full power-on ritual: calibrate → enable → ramp to default pose → hold → fold back). |
| `damiao_joint.py` | `DamiaoJoint` — one Damiao motor over CAN MIT mode (radians). |
| `odrive_can_joint.py` | `OdriveCanJoint` — one ODrive axis over CAN Simple (turns), runtime path. |
| `odrive_can_setup.py` | **USB, run once** — put the ODrive on CAN (node id, 1 Mbit, cyclic feedback) and save. Gains can't be set over CAN. |
| `odrive_position_control.py` | **USB** — interactive ODrive gain-tuning / config tool. |
| `demo_combined.py` | Bench demo over ZMQ: sine-drive any joint subset through the full service stack. |
| `read_motor_state.py` | **Read-only direct-CAN diagnostic** (pre-ZMQ bring-up): who's ONLINE + live pos/vel/torque, Damiao limp. |
| `dm_position_control.py` | Original flat single-Damiao sample script (reference). |
| `test_ipc.py` / `test_e2e_zmq.py` / `test_sim2sim_consistency.py` | Message layer tests, end-to-end ZMQ pipeline tests, IsaacLab↔MuJoCo contract tests. |
| `legacy_ros2/` | Frozen pre-ZMQ ROS2 nodes (reference only). |
| `HANDOFF.md` | Full context / progress recap. |

Dependencies: `pip install -r requirements.txt` (pyzmq, mujoco, onnxruntime,
python-can, damiao_motor, pyserial — **no ROS**).

## Quick start (bench, Ubuntu robot box)

**Startup convention (no homing — it could overload the printed parts/belts):
POWER ON with the ODrive joints folded all the way back against their stops.**
`calibrate` then maps turns-since-boot onto those known angles (`BOOT_POSE_RAD`).

```bash
sudo ip link set can0 up type can bitrate 1000000   # 1) bus up
python3 odrive_can_setup.py                          # 2) once: ODrive → CAN (node 7, 1M), saves+reboots
candump can0                                          # 3) sanity: DM feedback + ODrive heartbeat/encoder
python3 read_motor_state.py                          # 4) read-only: who is ONLINE + live states
python3 motor_service.py --only hip_left,lower_leg_left   # 5) service up (subset for the bench)
python3 tools/dodo_ctl.py status                     # 6) what does the service see?
python3 tools/dodo_ctl.py calibrate                  # 7) robot was powered on folded at the stops!
python3 tools/dodo_ctl.py status                     # 8) VERIFY the boot angles match reality
python3 tools/dodo_startup.py                        # 9) enable → unpark → ramp to default pose → hold → fold back
python3 demo_combined.py --joints hip_left,lower_leg_left #    or: sine-drive through the stack
python3 imu_service.py                               #    IMU on /dev/ttyACM0 (see firmware/README.md)
python3 tools/zmq_hz.py motor_states                 #    (anytime) rate/latency/drops
```

## Quick start (sim2sim, any machine — no hardware)

```bash
python3 sim_service.py &                             # real-time MuJoCo "robot" (needs ../dodo_files URDF)
python3 run_policy.py --backend zmq --task stand \
    --policy ../dodo_rl/.../dodo_stand/<run>/exported/policy.onnx \
    --startup-ramp 0        # sim spawns already posed; on the robot keep the default 5 s pre-start ramp
```

## Sim-to-real hook

`ZmqEnv` (over the `dodo_ipc` topics) is the single substitution point for the
IsaacLab policy — the runner's loop is identical against MuJoCo (`sim_service`)
and the real robot (`motor_service` + `imu_service`).

> **Deployment constraint:** the policy must **not** use `base_lin_vel` — base linear
> velocity can't be estimated reliably from the IMU without heavier state-estimation. The
> first target is a **stand-and-balance** policy (hold two-leg balance, resist mild
> pushes), which needs neither linear velocity nor velocity commands. The IsaacLab env is
> configured to drop both terms so the trained obs layout matches the hardware exactly.

- **Action** — per-joint position target. `step(action)` applies
  `target = default_pos + action_scale · action` (radians, sim convention), converts to
  each motor's native units, and sends it.
- **Observation** — `get_observation()` fills the **motor-derived** terms
  (`joint_pos_rel`, `joint_vel`, `last_action`) in policy-joint order, in radians. The two
  used **IMU** terms (`base_ang_vel` from the gyro, `projected_gravity` from
  accel/orientation) come from the FeedbackAggregator / IMU module and are stitched in via
  `to_policy_vector(base_ang_vel, projected_gravity)`, which concatenates the full vector
  in canonical order:

  ```
  [ base_ang_vel(3), projected_gravity(3), joint_pos_rel(n), joint_vel(n), last_action(n) ]
  ```

  Excluded on purpose: `base_lin_vel`, `velocity_commands`.

Keep `POLICY_JOINTS` in the **same order as the sim's articulation joints** so the final
swap (point the trained policy at `step()`/`get_observation()`) is a one-liner, not a
rewrite.

## Roadmap

Phases 0–2 of the ZMQ framework (message layer, service ports, real-time
sim2sim) are **done and tested on dev** — see `ZMQ_MIGRATION.md` for status,
measured timing, and what remains:

1. **Phase 3 — hardware bring-up** (Ubuntu box): bench-run `motor_service` +
   `tools/dodo_startup.py` + `demo_combined`, verify the `BOOT_POSE_RAD` stop
   angles/signs and the IMU scale constants + mounting axes, measure the Damiao
   hip/upper joint limits, verify ODrive gearing, calibrate gains toward the
   trained 32/3.0 behavior. (IMU firmware is committed now — `imu.cpp`; consider
   flashing `firmware/imu_stream_nicla/` for the fused quaternion.)
2. **Phase 4 — walking**: retrain flat *without* `base_lin_vel` (stand-style
   obs + `velocity_commands`) — `dodo_flat` as-is cannot deploy.

See `HANDOFF.md` for the full technical context.

