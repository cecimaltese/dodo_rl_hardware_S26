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

| File | What it is |
|---|---|
| `damiao_joint.py` | `DamiaoJoint` — one Damiao motor over CAN MIT mode (radians). |
| `odrive_can_joint.py` | `OdriveCanJoint` — one ODrive axis over CAN Simple (turns), runtime path. |
| `odrive_can_setup.py` | **USB, run once** — put the ODrive on CAN (node id, 1 Mbit, cyclic feedback) and save. Gains can't be set over CAN. |
| `odrive_position_control.py` | **USB** — interactive ODrive gain-tuning / config tool. |
| `motor_controller.py` | `MotorController` — owns the whole fleet on one bus; `enable_all`/`disable_all`/`set_zero_all`/`shutdown` + the sim-to-real `step(action)` / `get_observation()`. |
| `demo_combined.py` | Bench demo (1 Damiao + 1 ODrive over CAN) driving everything through `MotorController`. |
| `dm_position_control.py` | Original flat single-Damiao sample script (reference). |
| `HANDOFF.md` | Full context / roadmap recap. |

Dependencies: `python-can`, `damiao_motor==1.0.6`, `odrive==0.6.11.post0`.

## Quick start (bench)

```bash
sudo ip link set can0 up type can bitrate 1000000   # 1) bus up
python3 odrive_can_setup.py                          # 2) once: ODrive → CAN (node 7, 1M), saves+reboots
candump can0                                          # 3) sanity: see DM feedback + ODrive heartbeat/encoder
python3 demo_combined.py                              # 4) both motors move through MotorController
```

## Sim-to-real hook

`MotorController` is the single substitution point for the IsaacLab policy.

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

1. **Safety layer** in `MotorController`: per-joint position limits, max-velocity limit,
   and torque cutoff — at **0.8 of rated torque → motor to IDLE** (normalized 0–1).
2. **IMU / body-state module** → produces base velocity, acceleration, pitch, yaw.
3. **Two-class consolidation**: one control class (`MotorController`) + one feedback class
   (`FeedbackAggregator` = motor feedback + IMU).
4. **IsaacLab substitution**: replace the sim env's step/observation with the real ones.

See `HANDOFF.md` for the full technical context.

