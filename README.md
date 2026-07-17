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

## 📌 Last update — 2026-07-16 · next session 2026-07-17

**Where we are:** `calibrate_joints.py` is written and works; the calibration itself is
**not done**. A first run produced a `joint_calib.py` that is **incomplete and must be
redone** — do not command motion from it.

**What that run told us (the useful part):** the knee/ankle drives are **geared, not
direct**. `foot_left` and `foot_right` independently measured **13.402** and **13.346**
motor turns between their hard stops — agreeing to 0.4 %, so these are real, repeatable
stops on a symmetric mechanism. But 13.4 turns interpreted as `1 turn = 2π rad` implies an
**84 rad ankle**, which is impossible. Against the URDF's ankle range that's a ratio of
roughly **32:1** (`lower_leg_left` implies ~19.7 — the knee's real stops likely sit outside
its URDF soft limits, which is why the ratio is **not yet trustworthy**). This confirms the
warning already sitting in `fleet_config.py:117-120`. **The readings are good; the
conversion is wrong.**

> ⚠️ The gearing error makes the ODrive joints **under-travel by ~32×** — a "nothing moves"
> bug, not a "drives into the stop" bug. Knees/ankles are 9 Nm. Not dangerous; just wrong.

**Known bad in the current `joint_calib.py`:**
- `upper_leg_left` / `upper_leg_right` — **missing entirely** (only 6 of 8 joints present).
- `lower_leg_right` — **never moved**: its two "hard stops" are 0.0301 turns apart. Either
  the joint wasn't moved or the ODrive feedback went stale. The script caught it and warned
  that the sim's default `-0.50` is outside the measured stops.
- Both hips look **under-measured on one side**: `hip_left.raw_hi=+0.641` matches the
  recorded `0.641` exactly, but `raw_lo=-0.503` stops short of `-0.966`; `hip_right`'s
  `+0.699` stops short of `1.104`. Eased off before the metal.
- Both hips came back **`sign=+1`, i.e. no mirroring** — which *contradicts* this README's
  central suspicion. Either a real (welcome) result, or both hips were moved the same way by
  hand rather than the sim's `+` way. **Re-measure and watch this one.**
- `--check-persistence` was run but **the verdict was not recorded** — re-run and write down
  the per-motor `ABSOLUTE` / `BOOT-RELATIVE` line. It decides whether `--home` is a
  per-boot ritual or optional insurance.

### TODO — 2026-07-17

1. **Get the knee/ankle gear ratio.** From the build / CAD / WS25 notes if anyone has it — a
   nameplate number beats inference. Otherwise measure it: motors limp, rotate one ankle
   through a known angle (90° is easy to judge), read the turn delta,
   `ratio = turns × 2π ÷ angle`.
2. **Wire the ratio into `fleet_config.py`** as a per-joint `GEAR_RATIO`, threaded through
   `to_rad()` / `to_native_units()` in `calibrate_joints.py` **and** `_JointAdapter` in the
   control path. Until this lands, every ODrive `sim_lo`/`sim_hi`/`zero` is meaningless —
   and `--home` back-solves the zero *from* `sim_hi`, so homing is wrong too.
3. **Re-run the calibration** — `joint_calib.py` from 2026-07-16 should be deleted, not
   patched. The flow is now **zero-first** and much shorter (17 prompts, no stop-hunting):
   ```bash
   python3 calibrate_joints.py            # pose the crouch once, then nudge each joint
   python3 calibrate_joints.py --verify   # prove the signs BEFORE anything is commanded
   ```
   Watch the summary for `** NOT CALIBRATED` — that's the check that would have caught
   yesterday's missing `upper_leg_*` joints.
4. **`python3 calibrate_joints.py --limits`** — the hard stops, now an opt-in second pass.
   Do it deliberately and *reach the metal*: this is where yesterday went wrong. Needed for
   position clamps and for `--home`.
5. **Re-run `--check-persistence`** and record the verdict here.
6. Only then `--home`, and only then anything that commands motion.

## Status at a glance

| Subsystem | State |
|---|---|
| **Sim → sim (MuJoCo)** | ✅ **Verified.** The `dodo_stand` policy stands 30 s+. `test_sim2sim_consistency.py`: 17 pass / 1 fail / 1 skip. |
| **IMU** | ✅ **Verified on hardware.** `imu_check.py` passes: scale 0.5 % off 1 g, gyro bias ~1 °/s, 60 Hz. |
| **Safety layer** | ✅ Implemented (pos/vel clamps, 0.8·rated → IDLE, e-stop). ⚠️ Never exercised on hardware. |
| **Motor control** | ⚠️ Code complete, `py_compile`-clean, **never run on a motor. Not once.** |
| **Sign / zero calibration** | ⚠️ **Tool ready, calibration NOT done. Still the blocker.** `calibrate_joints.py` works; the first run was incomplete (6/8 joints, one never moved) and **must be redone** — see *Last update* above. |
| **ODrive gear ratio** | ❌ **Geared, ratio UNKNOWN — blocks all knee/ankle numbers.** Measured 2026-07-16: ~13.4 motor turns per ankle stop-to-stop, so `1 turn = 2π rad` is wrong. Confirms `fleet_config.py:117`. |
| **`AXIS_REMAP` (IMU→base)** | ❌ Unmeasured — needs the IMU bolted to the frame. |

> ### ⛔ Do not run the policy on the robot yet
> The real Damiao hips measure **left/right mirrored** relative to the URDF
> (`hip_left (-0.966, 0.641)` vs `hip_right (-0.544, 1.104)` — mirror images), but the
> URDF gives both sides identical symmetric limits and the same axis. **There is no sign
> or zero-offset layer in the control path** — `_JointAdapter.to_native()` only converts
> rad↔turns. Running the policy now drives the right leg *the wrong way into its hard
> stop* under a 27 Nm motor. A gantry stops it falling; it does not stop it breaking.
> Do the LIMP calibration (bring-up step 3) first.

## The two motor types

The robot uses two kinds of actuators, each on **its own CAN bus** with a different protocol.

| | **Damiao DM4340P** | **T-Motor Antigravity MN4004 (KV300)** |
|---|---|---|
| Role | Integrated CAN servo — hips (roll + pitch) | Gimbal BLDC — knees + ankles, driven by an **ODrive** |
| Driver | Built-in (motor + driver in one) | External ODrive (FOC) + encoder |
| Protocol | Damiao MIT mode (CAN std frame) | ODrive **CAN Simple** |
| Bus | `can0` | `can1` |
| Native units | **radians** | **turns** (1 turn = 2π rad) |
| Limits (`damiao_motor` preset `4340P`) | pos ±12.5 rad, vel ±10 rad/s, **torque ±28 Nm** | ODrive `vel_limit` / `rated_torque` config |
| Sim effort limit | **27 Nm** (hip_ + upper_leg_) | **9 Nm** (lower_leg_ + foot_) |
| Feedback | dual 14-bit magnetic encoders; pos/vel/torque/temp over CAN | encoder estimates + torque over CAN |
| CAN bitrate | 1 Mbit/s | 1 Mbit/s |

The Damiao is an all-in-one servo: send it an MIT-mode frame and it closes the loop
internally. The T-Motor is a "dumb" BLDC, so an **ODrive** does the field-oriented
control; we talk to the ODrive over CAN Simple. The key design choice is that **both
motor types expose the identical Python joint interface**, so one control loop drives
them uniformly — only the units differ.

> ⚠️ **ODrive gearing is UNVERIFIED.** `to_native()` assumes **direct drive** (1 turn =
> 2π rad). If the knees/ankles are geared, every ODrive command *and* every safety clamp
> is wrong by the gear ratio. Confirm before commanding motion.

## CAN bus architecture

**Two buses, one USB-CAN adapter each** — the same split last semester's WS25 stack used.
Separate electrical buses mean no cross-talk and no arbitration-id collisions.

```
  can0 @ 1 Mbit/s ── Damiao ── hip_left 0x01, upper_leg_left 0x02,      (feedback 0x11/0x12/0x15/0x16)
                               hip_right 0x05, upper_leg_right 0x06
  can1 @ 1 Mbit/s ── ODrive ── nodes 3, 4, 7, 8   (MEASURED off the live bus)
```

> ⚠️ **Which node drives which leg is UNVERIFIED.** `odrive_can_scan.py` proves the four
> nodes exist and answer; it cannot say which is a knee and which an ankle. WS25 never
> recorded a mapping either (it used raw `can1:3` as the joint name). The assignment in
> `fleet_config.OD_JOINTS` is a placeholder — confirm physically before commanding motion.

CAN Simple arbitration id = `(node_id << 5) | cmd_id`; a reader thread routes each
inbound frame to the right joint by `node_id`.

> **Node ids 3/4/7/8 are collision-safe even on a shared bus** (windows `0x060-0x07F`,
> `0x080-0x09F`, `0x0E0-0x0FF`, `0x100-0x11F` never touch the Damiao ids), so there's no
> reason to renumber them. **Node 0 would NOT be** — its window `0x000-0x01F` swallows
> every Damiao command *and* feedback id.

Both types *can* share one bus (SocketCAN gives each socket its own copy of every frame,
so each gets its own filtered socket). Set `od_channel` = `dm_channel` to do that; the
Damiao filter is then applied via `bus.set_filters(...)`, because the installed
`damiao_motor` 1.0.6 `DaMiaoController.__init__` does not accept `can_filters`.

Bring the buses up first (OS level — not python-can):

```bash
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can1 up type can bitrate 1000000
```

> ⚠️ **`can0`/`can1` are enumeration order, not device identity** — and this HAS bitten us:
> adapter `0038..` was `can0` with one adapter plugged in and became `can1` once the second
> was added. A replug or reboot can silently swap the Damiao and ODrive buses, and nothing
> errors — both are valid CAN interfaces. **Fix: `99-dodo-can.rules`** pins each adapter to
> `can_dm`/`can_od` by serial. Install it (instructions in the file), then set
> `DM_CHANNEL`/`OD_CHANNEL` in `fleet_config.py` to the stable names.

> ⚠️ **The ODrive web GUI cannot use these adapters.** Its USB-CAN option needs WebUSB to
> claim the device, but the `gs_usb` kernel driver already owns it — that's *why* `canX`
> exists. You get `Could not claim interface 0 on USB device`. Don't fight it: SocketCAN +
> `odrive_can_scan.py` is the native path. (The GUI also defaults to 125 kbps; the bus is 1 M.)

## The IMU

An **Arduino Nicla Sense ME** (BHI260AP) on USB serial, running `imu_firmware_v2/`.
`observer.py` polls it and publishes `sensor_msgs/Imu` on `/imu_states`.

It supplies **6 of the 30 obs dims** — `base_ang_vel` (gyro) and `projected_gravity`
(fused quaternion, or the accelerometer as fallback). Without it the policy is blind to
tilt and cannot balance, so `RealEnv` now **hard-aborts** if `/imu_states` is missing or
goes stale (>200 ms) rather than flying on a frozen default.

Firmware v2 (preferred) over v1:
- **Explicit sensor ranges + a `'V'` handshake** — the device reports the ranges it read
  back off the chip, so the host never guesses. (`SensorXYZ::x()` returns **raw int16
  counts**, not m/s² — an unknown range silently mis-scales `base_ang_vel`.)
- **Fused orientation via `SENSOR_ID_GAMERV`** — accel+gyro, deliberately **not**
  `SENSOR_ID_RV`, which fuses the magnetometer (worthless next to 8 BLDC motors). GAMERV's
  yaw drifts, but `projected_gravity` is yaw-invariant, so it doesn't matter.
- **921600 baud** — 115200 capped polling at 41 Hz, *below* the 50 Hz control loop.

`observer.py` auto-detects v1/v2 and baud, so flashing is optional and reversible.

Build + flash (needs `arduino-cli`; the `arduino:mbed_nicla` core does the rest):
```bash
arduino-cli compile --fqbn arduino:mbed_nicla:nicla_sense imu_firmware_v2
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:mbed_nicla:nicla_sense imu_firmware_v2

# rollback to Jim's original if ever needed:
# arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:mbed_nicla:nicla_sense imu_firmware_v1
```

## Files

### Policy / sim-to-real
| File | What it is |
|---|---|
| `rl_env.py` | **`Env` ABC + the IsaacLab policy contract** (joint order, default pose, action scaling, obs layout) + `OnnxPolicy`. The load-bearing definitions. |
| `sim_env.py` | `SimEnv(Env)` — MuJoCo backend. The sim-to-sim verification gate. |
| `real_env.py` | `RealEnv(Env)` — ROS backend. Publishes `/motor_commands`, subscribes `/motor_states` + `/imu_states`. Hard-aborts on missing/stale feedback. |
| `run_policy.py` | Env-agnostic runner: `--backend sim\|real --task flat\|stand --policy <onnx>`. |
| `test_sim2sim_consistency.py` | 19 checks tying `params/env.yaml` ↔ `rl_env`/`sim_env` ↔ URDF/USD ↔ onnx, plus a behavioural stand test. Run it after touching anything. |

### Motors
| File | What it is |
|---|---|
| `99-dodo-can.rules` | udev rule pinning each USB-CAN adapter to a stable name by serial. Install it — see the header. |
| `fleet_config.py` | **Single source of truth** — motor ids/node ids, CAN buses, joint limits, rated torques. ROS-free so the bench tools import it too. **Edit fleet config here, nowhere else.** |
| `motor_controller.py` | **`MotorController`** — ROS2 node owning all 8 joints (Damiao + ODrive) with the safety layer. Sub `/motor_commands`, pub `/motor_states`. |
| `damiao_joint.py` | `DamiaoJoint` — one Damiao motor over CAN MIT mode (radians). |
| `odrive_can_joint.py` | `OdriveCanJoint` — one ODrive axis over CAN Simple (turns), runtime path. |
| `odrive_can_setup.py` | **USB, once PER DRIVE** — `--node-id N` (required) puts that ODrive on CAN (node id, 1 Mbit, cyclic feedback) and saves. Gains can't be set over CAN. It configures whichever drive is on USB, so plug in one at a time. |
| `odrive_can_scan.py` | Find live nodes on a CAN bus — passive listen + active RTR sweep of ids 0–63. "Is anything on the wire?" |
| `odrive_position_control.py` | **USB** — interactive ODrive gain-tuning / config tool. |
| `calibrate_joints.py` | **Guided sign / zero / limit calibration.** Read-only — you move the robot by hand, it watches all 8 and auto-detects which motor moved (you can't mislabel a joint). **Main pass is zero-first:** pose the crouch once → snapshot all 8 → nudge each joint from there for mapping+sign → `zero` falls out. ~17 prompts, no hard stops. `--verify` proves the signs by hand before anything is commanded; `--limits` is an opt-in stop-walking pass (the slow, risky half) that produces `sim_lo`/`sim_hi` and enables `--home`; `--home` re-zeros against a stop at startup; `--check-persistence` says whether that ritual is needed at all. Writes **merge**, so `--only <joint>` patches one joint without dropping the rest. Output splits into a **durable** half (`motor`/`sign`/`sim_lo`/`sim_hi` — mechanical) and a **perishable** half (`zero`/`raw_lo`/`raw_hi` — dies on every power cycle). |
| `read_motor_state.py` | **Read-only bench diagnostic.** Marks each motor ONLINE/OFFLINE and streams pos/vel/torque. Damiao read **LIMP** (kp=kd=0, backdrivable); commands no motion. **The tool for the sign/zero calibration.** |
| `demo_combined.py`, `dm_position_control.py` | Older bench demos (reference). |

### IMU
| File | What it is |
|---|---|
| `observer.py` | `Observer` ROS2 node — polls the Nicla, publishes `/imu_states`. Owns `parse_imu_line`, the `'V'` handshake, and `AXIS_REMAP`. |
| `imu_check.py` | **One-command acceptance test.** No ROS. Checks port/format/scale/gyro/quat/gravity/rate. Exit 0 = the IMU is sane. |
| `imu_sniff.py` | Deeper diagnostic — raw byte dump, `--gyro-cal` (pins the gyro range on v1), **`--axes`** (builds `AXIS_REMAP`). |
| `imu_firmware_v2/` | **The Nicla firmware** (Arduino sketch) — explicit ranges + `'V'` handshake + fused GAMERV quaternion, 921600 baud. This is what's flashed. |
| `imu_firmware_v1/` | Jim's original, verbatim — the rollback. Flashable as-is. |

Dependencies: `python-can`, `damiao_motor==1.0.6`, `odrive==0.6.11.post0`, `pyserial`,
`numpy`, `mujoco`, `onnxruntime`, and **ROS2 Humble** (`rclpy`) for the `real` backend.

## Bring-up sequence

Do these **in order**. Each one is a prerequisite for the next.

```bash
# 1) IMU — bench, no robot, no ROS needed
python3 imu_check.py                        # must print RESULT: PASS

# 2) CAN — buses up, motors visible
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can1 up type can bitrate 1000000
python3 odrive_can_scan.py --channel can1   # <- ODrive nodes 0-3 alive? (default is can0!)
candump can0                                # DM feedback frames
# If a drive isn't on CAN yet: python3 odrive_can_setup.py --node-id N   (USB, one at a time)

# 3) SIGN / ZERO CALIBRATION  <-- THE BLOCKER. Robot on the gantry, motors LIMP.
python3 read_motor_state.py                       # first: are all 8 ONLINE?
python3 calibrate_joints.py --check-persistence   # is a stored zero valid AT ALL?
python3 calibrate_joints.py                       # main pass: pose crouch, nudge each joint
python3 calibrate_joints.py --verify              # prove the signs before commanding anything
python3 calibrate_joints.py --limits              # opt-in: hard stops -> clamps + --home
python3 imu_sniff.py --axes                       # IMU now mounted -> AXIS_REMAP

# 3b) AFTER EVERY POWER CYCLE, before anything is commanded:
python3 calibrate_joints.py --home                # push each joint to its hard stop; re-zeros
#    The ODrives are boot-relative: they zero their position estimate at power-up, so the
#    stored `zero` describes an origin that no longer exists — readings can be a full radian
#    or more off. What survives is the METAL: the hard stops don't move. --home pushes each
#    joint to a known stop and back-solves the zero from the stored sim_hi. Repeatable to the
#    stop, not to your eye. Skip it and the policy commands from a wrong origin.

# 4) One joint, a few degrees. Prove direction, clamps, and that the e-stop fires.
# 5) Hold the default pose, NO policy: hip 0 / upper_leg 0.2 / lower_leg -0.5 / foot 0.3.
#    If it takes that crouch, the whole action path is validated.
# 6) Only then, the policy. Three processes, three terminals — plain python3,
#    no `ros2 run` needed (these are ROS nodes, but you launch them directly):
python3 motor_controller.py --ros-args -p move_duration:=0.0
python3 observer.py
python3 run_policy.py --backend real --task stand --policy <path>/policy.onnx
```

> The robot is a biped: IDLE-ing one leg motor drops it. **Run on the gantry.**
> And launch `MotorController` with `move_duration:=0.0` so streamed 50 Hz targets track
> directly instead of each one starting a 1 s ramp.

## The sim-to-real substitution point

The swap happens at the **`Env` interface** (`rl_env.py`) — *not* inside `MotorController`,
which is a pure ROS node. `run_policy.py` drives `SimEnv` and `RealEnv` with the identical
loop; only `--backend` changes. That's why sim-to-sim in MuJoCo is a real gate: if the
policy behaves there, the obs/action plumbing and joint mapping are correct and `RealEnv`
is a drop-in.

> **Deployment constraint:** the policy must **not** use `base_lin_vel` — it can't be
> estimated from the IMU without heavier state estimation. The deployable target is the
> **stand-and-balance** policy (obs **30**), which needs neither linear velocity nor
> velocity commands. `dodo_flat` (walking, obs 36) is **sim-only**.

- **Action** — `target_rad = default_pos + 0.5 · action`, 8 dims, applied in policy joint
  order, converted to each motor's native units.
- **Observation** — concatenated in canonical order:

  ```
  [ base_ang_vel(3), projected_gravity(3), joint_pos_rel(8), joint_vel(8), last_action(8) ]
  ```
  Excluded on purpose: `base_lin_vel`, `velocity_commands`.

- **Joint order** — type-grouped BFS, **left-first**:
  `[hip_l, hip_r, upper_leg_l, upper_leg_r, lower_leg_l, lower_leg_r, foot_l, foot_r]`.
  Everything maps **by name**, so MuJoCo's own (different) order doesn't matter.
- **Gains** — the policy is trained with `ImplicitActuator` **kp=32 / kd=3.0**. Any change
  must stay lockstep across `dodo.py` ↔ `sim_env.KP/KD` ↔ hardware, and **requires a retrain**.

> ⚠️ `motor_controller.py` currently sends the Damiao **kp=30 / kd=0.5**, but the policy
> was trained at 32/3. Damiao MIT gain units differ from IsaacLab's, so treat 32/3 as a
> calibration *target*, not a literal copy. Open item.

## Roadmap

1. ~~**Safety layer**~~ ✅ done — pos/vel clamps at 0.9·limit, `|torque| > 0.8·rated` →
   that one motor to IDLE, full e-stop on fault / over-speed / hard-limit / stalled
   command stream / silent feedback.
2. ~~**IMU / body-state module**~~ ✅ done — `observer.py` + firmware v2, verified on hardware.
3. **Sign + zero calibration layer** ← **next, and blocking everything else.**
4. **First hardware run** — bring-up steps 4–6 above.
5. **Match the real Damiao gains** to the trained kp≈32 / kd≈3.0.
6. **Retrain `dodo_flat`** to the current contract for walking (it's stale: trained on the
   old default pose).

**Open architecture question:** ROS2 isn't load-bearing here — `MotorController` talks CAN
via python-can and `Observer` talks pyserial; both are plain Python underneath. `RealEnv`
could call them directly in one process. ROS buys process isolation and `ros2 topic echo`/
`bag`; it costs 3 processes and DDS hops on an already-tight latency budget. Worth a
deliberate decision.

See `HANDOFF.md` for the full technical context.

