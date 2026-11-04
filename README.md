<p align="center">
    <img width="1879" height="252" alt="TUM_mirmi" src="https://github.com/user-attachments/assets/b1440892-0ecc-4c05-9875-69f6e070998a" />
</p>

# 🦤 Dodo hardware: the sim-to-real stack

> *"The original dodo went extinct because it couldn't run. We're fixing that — and adding a backflip for good measure."*

This repo runs trained policies on the physical Dodo, an 8-DOF biped from TUM MIRMI. It
holds the motor and IMU services, a real-time MuJoCo stand-in for the robot, the robot
description, a reference policy, and the bench and calibration tools. Policies are trained
elsewhere: so far in the S26 Isaac Lab project
[`dodo_rl`](https://github.com/cecimaltese/dodo_rl), next by Shafeef Omar.

New here? Read **Status**, then **Bring-up**. `HANDOFF.md` has the history and the
decisions behind it.

| Property | Value |
|---|---|
| Joints | 8 revolute: hip roll, hip pitch, knee, ankle per leg |
| Mass | 4.72 kg (CAD; the robot has not been weighed) |
| Height | base ~0.44 m in the default crouch |
| Hip roll + pitch | Damiao DM4340P, 27 Nm |
| Knee + ankle | T-Motor MN4004 via ODrive, geared, 9 Nm in sim |

## Status (2026-11-04)

| Subsystem | State |
|---|---|
| **Hardware** | ✅ Every motor driver powered, CAN daisy chain continuous (repaired 2026-10-30). |
| **ZMQ stack** | ✅ Reconciled with main's July work 2026-10-30. In sim, the stand policy balances 10 s through `sim_service` + `ZmqEnv`. ⚠️ Never run against `motor_service` on the robot. |
| **Sim → sim (MuJoCo)** | ✅ The `dodo_stand` policy stands 30 s+. |
| **IMU** | ✅ Verified on hardware (firmware v2): scale 0.5 % off 1 g, gyro bias ~1 °/s, ~60 Hz. ✅ Pose measured on the robot 2026-11-02 (`description/imu_mount.json` → URDF `imu_link`, `AXIS_REMAP`). The board sits off square, so the record is the exact fit (2026-11-04). ⚠️ The first live check still read 3.8° nose-down on a phone-levelled robot: pitch unresolved (HANDOFF §2 step 1, `imu_locate.py level`). |
| **Safety layer** | ✅ In `motor_service`: position/velocity clamps, 0.8 × rated torque → that motor IDLE, e-stop, and a silent-joint watchdog that works (it couldn't fire before 2026-11-04). ⚠️ Never exercised on hardware. |
| **Motor control** | ⚠️ Never run on a motor. |
| **Sign / zero calibration** | ❌ **The blocker.** Tooling ready (`real2sim.py`, `calibrate_joints.py`, the meshed visualizer as the check); the calibration itself is still to do (July's run was deleted), and `motor_service` doesn't apply measured signs or stops yet. |
| **ODrive gear ratio** | ❌ Unknown (~32:1 inferred). Blocks every knee/ankle number. Measure: `calibrate_joints.py --ratio <joint>` → `fleet_config.GEAR_RATIO`. |
| **CAN udev rule** | ✅ `setup/99-dodo-can.rules` names the adapters `can_dm`/`can_od` by serial and brings them up at 1 Mbit (2026-11-04, *Machine setup*). |

> ### ⛔ Do not run a policy on the robot yet
> `motor_service` has no sign or zero-offset layer: `_JointAdapter` only converts
> rad ↔ turns. The measured hip-pitch ranges are left/right mirrored
> (`upper_leg_left (-0.32, 2.38)` vs `upper_leg_right (-2.30, 0.29)`), which points to one
> hip-pitch motor reading inverted relative to the URDF, and July's incomplete calibration
> never measured that pair. (The hip-roll ranges look mirrored too, but that's expected:
> see `fleet_config.py`.) The knee/ankle gear ratio is also unknown. Until calibration is
> done and wired in, a policy can drive a joint the wrong way into its hard stop under a
> 27 Nm motor. A gantry stops the robot falling; it does not stop it breaking.

## Architecture

Every box is its own process, and they talk over ZMQ. `sim_service.py` publishes the same
topics as the two robot services, so the policy runner can't tell sim from robot.

```
  ROBOT                                                        POLICY
  motor_service.py   owns both CAN buses  ── motor_states ───►  run_policy.py --backend zmq
    Damiao hips (rad), ODrive knees/ankles ◄── motor_commands ──   ZmqEnv: observations,
    safety layer, boot-pose calibration    ◄──────── ctl ──────►   50 Hz pacing, ONNX policy
  imu_service.py     owns the Nicla serial ─────────── imu ────►
            — or —
  sim_service.py     MuJoCo in real time, the same four endpoints
```

| Topic | Pattern | Rate | Content |
|---|---|---|---|
| `motor_states` | PUB → SUB | 200 Hz | position, velocity, torque per joint (rad, rad/s, Nm) + per-joint and global flags |
| `motor_commands` | PUB → SUB | 50 Hz | absolute position targets (rad); NaN leaves a joint alone |
| `imu` | PUB → SUB | up to 100 Hz | gyro (rad/s), accel (m/s²), fused quaternion |
| `ctl` | REQ ↔ REP | on demand | `status`, `calibrate`, `enable`, `disable`, `set_zero`, `estop_reset` |

`dodo_ipc.py` defines the endpoints, the binary schemas and the canonical joint order;
[`docs/zmq.md`](docs/zmq.md) has the full spec.

**Why ZMQ and not ROS2** (decided 2026-07-06): ROS2 only carried messages here, since the
motors talk python-can and the IMU pyserial. It also needed a ROS install on the robot and
couldn't run on the dev Macs. ZMQ sockets with one topic each give the 50 Hz loop the
newest message, never a backlog. The port kept the drivers, safety layer and policy contract
unchanged, and made the real-time sim stand-in on the same topics possible.

## Hardware

| | **Damiao DM4340P** | **T-Motor Antigravity MN4004 (KV300) + ODrive** |
|---|---|---|
| Joints | hip roll (`hip_*`), hip pitch (`upper_leg_*`) | knee (`lower_leg_*`), ankle (`foot_*`) |
| Driver | integrated servo; Damiao MIT mode over CAN | external ODrive (FOC + encoder); CAN Simple |
| Native units | radians | turns |
| Limits | ±12.5 rad, ±10 rad/s, ±28 Nm (`4340P` preset); sim 27 Nm | set on the drive over USB; sim 9 Nm |
| Zero | hardware zero, assumed to persist (`calibrate_joints.py --check-persistence` decides) | boot-relative: the encoder zeroes at power-up |
| Bus | `can_dm` @ 1 Mbit/s; ids 0x01/0x02/0x05/0x06, feedback 0x11–0x16 | `can_od` @ 1 Mbit/s; nodes 3/4/7/8 |

> ⚠️ **The ODrive joints are geared, ratio unknown.** Every conversion assumes 1 motor turn
> = 2π rad at the joint. In July both ankles measured ~13.4 motor turns stop to stop, which
> implies roughly 32:1. Until the ratio is in `fleet_config.py`, every knee/ankle command,
> clamp and velocity check is off by that factor.

**CAN.** Two buses, one USB-CAN adapter each. `can0`/`can1` are enumeration order, not
identity: a replug can silently swap them. A udev rule pins each adapter by serial to
`can_dm` / `can_od`, the names `fleet_config.py` uses, and brings both up at 1 Mbit:
`setup/99-dodo-can.rules` (install: *Machine setup*).
- The node → joint assignment for the ODrives is a placeholder; `calibrate_joints.py`
  measures the real one.
- Both motor types can share one bus (pass the same channel twice). Nodes 3/4/7/8 stay
  clear of the Damiao ids; node 0 would not.
- The ODrive web GUI can't use these adapters (the `gs_usb` driver owns them). Use
  SocketCAN and `odrive_can_scan.py`.

**IMU.** Arduino Nicla Sense ME (BHI260AP) on USB serial running `firmware/imu_firmware_v2`:
gyro, accelerometer and an on-chip accel+gyro fusion quaternion (no magnetometer next to
eight motors). It supplies 6 of the 30 policy inputs, so `ZmqEnv` refuses to run on a
missing or stale IMU stream. `AXIS_REMAP` in `imu_protocol.py` rotates the board axes into
the body frame and must match the URDF's IMU pose. `imu_locate.py` measures both on the
robot and writes them from one record, `description/imu_mount.json`
([docs/imu_locate.md](docs/imu_locate.md)); not measured yet. Flashing: `firmware/README.md`.

## Machine setup

Once per robot machine (Ubuntu). Everything after this runs as your normal user.

```bash
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt   # Python >= 3.10

# CAN: stable names can_dm / can_od, up at 1 Mbit whenever the adapters are plugged in
sudo cp setup/99-dodo-can.rules /etc/udev/rules.d/
sudo udevadm control --reload
# then unplug + replug both USB-CAN adapters (or reboot)
ip -details link show can_dm    # UP, "bitrate 1000000"; same for can_od

# IMU: the Nicla is /dev/ttyACM0, group dialout
sudo usermod -aG dialout $USER  # log out and back in to take effect
```

The rule matches each adapter by its USB serial. A new or swapped adapter needs its
serial in the rule: `udevadm info -q property /sys/class/net/can0 | grep SERIAL_SHORT`.
Which bus is which is a wiring fact: the ODrives stream cyclic frames on theirs, the
Damiaos stay silent until spoken to (`candump can_od` vs `candump can_dm`).

## Bring-up

In order: each step is a prerequisite for the next. Robot on the gantry from step 4.
`python3` below means the repo's venv (`source .venv/bin/activate`).

```bash
# 1) IMU (bench, no robot needed)
python3 imu_check.py                             # must print RESULT: PASS

# 2) CAN: buses up (the udev rule does it), every motor answers
ip -details link show can_dm                     # and can_od: UP, bitrate 1000000
python3 odrive_can_scan.py                       # nodes 3/4/7/8 alive on can_od?
candump can_od                                   # ODrive cyclic frames, ~100 Hz each
# a drive not on CAN yet: python3 odrive_can_setup.py --node-id N   (USB, one drive at a time)

# 3) Measure, motors LIMP. These tools open the buses directly: stop motor_service first.
python3 read_motor_state.py                      # all 8 ONLINE?
python3 calibrate_joints.py --ratio foot_left    # each knee/ankle -> fleet_config.GEAR_RATIO
python3 calibrate_joints.py --check-persistence  # do stored zeros survive a power cycle?
python3 calibrate_joints.py                      # legs straight down once, nudge each joint
python3 calibrate_joints.py --verify             # prove the signs by hand
python3 visualize_motor_state.py                 # THE check: the model follows your hands
python3 calibrate_joints.py --limits             # hard stops -> sim_lo/sim_hi
python3 imu_locate.py measure                    # IMU mounted -> URDF imu_link + AXIS_REMAP

# 4) Services. Power on with knees and ankles folded back against their stops.
python3 motor_service.py                         # starts DISABLED
python3 imu_service.py
python3 tools/dodo_ctl.py status
python3 tools/dodo_ctl.py calibrate              # ODrive turns-since-boot -> BOOT_POSE_RAD
python3 tools/dodo_ctl.py status                 # VERIFY the stop angles match reality

# 5) One joint, a few degrees: prove direction, clamps and the e-stop
python3 demo_combined.py --joints hip_left --amp 0.05

# 6) The default pose without a policy: enable, ramp to the crouch, hold, fold back on exit
python3 tools/dodo_startup.py

# 7) Only then, a policy
python3 run_policy.py --backend zmq --task stand \
    --policy policies/dodo_stand/2026-06-30_18-44-06/exported/policy.onnx
```

### Calibration: how the pieces fit

- **Damiao (hips):** hardware zero, assumed persistent. `motor_service` treats them as
  calibrated from boot.
- **ODrive (knees, ankles):** zeroed at power-up. Instead of a homing drive, which could
  overload the 3D-printed parts and belts, the robot is powered on with these joints folded
  against their stops. `dodo_ctl calibrate` then maps turns-since-boot onto `BOOT_POSE_RAD`
  in `fleet_config.py`. `ZmqEnv` refuses to run until every joint is calibrated.
- **`real2sim.py`** is what a reading means: `q = sign · (raw − zero) · scale`, scale 1 for
  a Damiao, 2π / gear ratio for an ODrive (`fleet_config.GEAR_RATIO`). Every tool that
  turns a reading into an angle goes through it.
- **`calibrate_joints.py`** measures what that needs: each knee/ankle's gear ratio
  (`--ratio`), which motor drives which joint, each joint's sign and zero (the main pass,
  from the legs-straight-down zero pose by default), and the stop angles (`--limits`). It
  writes `joint_calib.py`, recording the ratio each entry was made with. Its `--home`
  re-zeros that file by pushing each joint to a stop by hand.
- **`visualize_motor_state.py`** is the acceptance check: the meshed model follows your
  hands, the right way and by the right amount, or something above is wrong. On exit it
  lists the range each joint reached.

**Not wired yet (the blocker):** `motor_service` uses the gear ratio (through `real2sim`),
but neither the measured signs (it assumes +1) nor the measured stops (`BOOT_POSE_RAD`
holds URDF placeholders). Its ODrive node → joint mapping in `fleet_config.py` is the July
calibration's word only. There is no `joint_calib.py` yet: July's was incomplete and was
deleted on 2026-11-04.

## The sim-to-real contract

`rl_env.py` holds everything the policy knows: joint order, default pose, action scaling,
observation layout. `run_policy.py` runs the same loop against either backend:
- `--backend sim`: MuJoCo in-process, lockstep (`sim_env.py`). Fast checks.
- `--backend zmq`: through the services, in real time. Against `sim_service.py` this is the
  full rehearsal; against `motor_service` + `imu_service` it is the robot. (`real` is an
  alias.)

The current contract is the `dodo_stand` policy's:
- **Action:** `target = default_pos + 0.5 · action` (rad, 8 dims, policy joint order).
- **Observation (30):** `[base_ang_vel(3), projected_gravity(3), joint_pos − default(8),
  joint_vel(8), last_action(8)]`. There is no `base_lin_vel`, because the robot can't
  measure it.
- **Joint order:** `hip_l, hip_r, upper_leg_l, upper_leg_r, lower_leg_l, lower_leg_r,
  foot_l, foot_r` (Isaac Lab's breadth-first order). The services use the same order on the
  wire (`dodo_ipc.JOINT_ORDER`).
- **Default pose:** hip 0, upper_leg 0.2, lower_leg −0.5, foot 0.3 rad.
- **Gains:** trained with an implicit PD actuator, kp 32 / kd 3. The Damiao hips currently
  run kp 30 / kd 0.5 in MIT units, which are not Isaac Lab's units; matching the behavior is
  an open bench task.

A policy with a different contract (Shafeef's, for instance) means updating `rl_env.py`
and adding its run to `policies/`. It then has to pass the tests below before it goes near
the robot.

## Files

| Path | What it is |
|---|---|
| `dodo_ipc.py` | The ZMQ contract: endpoints, binary schemas, joint order, latest-value subscriber, ctl client/server, rate loop. |
| `motor_service.py` | The motor fleet as a service: safety layer, bumpless fail-closed `enable`, boot-pose `calibrate`. |
| `imu_service.py` | IMU serial → `imu` topic. |
| `sim_service.py` | MuJoCo as a real-time virtual robot on the same topics, with optional latency/drop injection. |
| `zmq_env.py` | `ZmqEnv`, the deployment backend. Refuses to run on stale, e-stopped, disabled or uncalibrated streams. |
| `rl_env.py` | The policy contract (`Env` base, joint order, default pose, observation layout) and `OnnxPolicy`. |
| `sim_env.py` | `SimEnv`: in-process MuJoCo. Also `URDF_PATH`. |
| `run_policy.py` | The policy runner, for either backend. |
| `fleet_config.py` | **The** robot config: motors, ids, buses, joint limits, torques, boot pose. Edit it here only. |
| `damiao_joint.py`, `odrive_can_joint.py` | One joint each, behind an identical interface. |
| `imu_protocol.py` | Nicla wire protocol: line formats, scaling, `AXIS_REMAP`, baud detection. |
| `firmware/` | Nicla firmware: v2 (flashed) and v1 (rollback). |
| `real2sim.py` | Motor reading ↔ sim joint angle (sign, zero, gear ratio), for every tool. |
| `calibrate_joints.py`, `joint_calib.py` | Guided gear-ratio/sign/zero/limit calibration (read-only, by hand) and its output (not yet made). |
| `read_motor_state.py`, `visualize_motor_state.py` | Read-only fleet diagnostics: a live table, or the meshed MuJoCo model following the robot (+ reached ranges). |
| `odrive_can_scan.py`, `odrive_can_setup.py`, `odrive_position_control.py` | ODrive: find nodes on a bus; put a drive on CAN (USB, once per drive); tune gains (USB). |
| `imu_check.py`, `imu_sniff.py` | IMU acceptance test; raw dump, range cross-check, `--axes`. |
| `imu_locate.py` | Measures the IMU mount (three poses + a ruler) and writes the URDF's `imu_link`, `AXIS_REMAP` and the IMU table from `description/imu_mount.json`; `refit [--exact]` re-solves from the stored poses; `check --live` verifies on the robot. |
| `setup/99-dodo-can.rules` | udev rule: the two USB-CAN adapters → `can_dm` / `can_od` by serial, up at 1 Mbit. |
| `tools/` | `dodo_ctl.py` (ctl CLI), `dodo_startup.py` (power-on ritual without a policy), `zmq_echo.py` and `zmq_hz.py` (topic echo and rate). |
| `demo_combined.py` | Sine-drive any joints through the full service stack. |
| `description/` | URDF + meshes of the robot as built, with a README for whoever trains on it. |
| `policies/` | The reference `dodo_stand` run the tests use. |
| `docs/zmq.md` | ZMQ topic and schema spec. |
| `docs/imu_locate.md` | Bench procedure for measuring the IMU pose. |
| `HANDOFF.md` | Current state, decisions, history. |
| `dm_position_control.py`, `odrive_joint.py` | Early single-motor scripts, kept for reference; unused. |

## Tests

```bash
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt   # any OS, Python >= 3.10
.venv/bin/python -m pytest
```

Everything runs on any machine without hardware: the message layer (`test_ipc.py`), the
service stack against a synthetic robot plus the stand policy through it
(`test_e2e_zmq.py`), `motor_service`'s silent-joint watchdog with the real drivers on
python-can's virtual bus against a fake fleet (`test_motor_watchdog.py`),
boot-calibration math, IMU parsing, the IMU mount measurement against a simulated IMU
(`test_imu_locate.py`), the fleet config, and the Isaac Lab ↔ MuJoCo contract
(`test_sim2sim_consistency.py`). There is one known failure:
`test_behavior_wrong_order_falls`. The stand policy also stands with left and right swapped,
so that guard can't detect a joint-order regression.

## Next steps

The roadmap (HANDOFF §1): joints real → sim, then the joint limits, a PD standing
controller, Shafeef's standing policy, and later walking (onboard compute + battery).

1. Measure the knee/ankle gear ratios (`calibrate_joints.py --ratio <joint>`) into
   `fleet_config.GEAR_RATIO`.
2. Calibrate (bring-up step 3) and check it with `visualize_motor_state.py`; then the
   limits (`--limits`) into `fleet_config.py`.
3. Wire the signs, measured stops and motor mapping into `motor_service`.
4. First motion and the PD stand (bring-up steps 5–6, `tools/dodo_startup.py`). Then match
   the Damiao gains to the kp 32 / kd 3 behavior.
5. Shafeef's standing policy: adapt `rl_env.py`, rehearse it on `sim_service`, settle the
   IMU's ~3° pitch question (HANDOFF §2), then the gantry.
6. Merge `merge-zmq-dev` into `main`.

---

Supervised by Dr. Shafeef Omar and Dr. Hoan Quang Le (TUM MIRMI). S26 team: Maltese Invrea,
Ghonim, Hucklenbroich, Pickrell. Builds on the WS25 team's DoDo Alive! ROS2 stack (CAN
layout, ODrive node ids).
