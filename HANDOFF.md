# Dodo Motor Control — Handoff / Context Recap

> Purpose: hand this to another Claude session so it can pick up exactly where we left off.
> Covers the **short-term plan (CAN migration — in progress)** and the **long-term roadmap**.
> Working directory for all motor-control files: `/home/dodo/Documents/dodo/`

---

## 1. Project goal (big picture)
Build a real-hardware motor-control + feedback stack for a robot ("dodo") whose final
purpose is **sim-to-real**: a trained policy (currently in an Isaac Lab simulation repo)
should drive the real motors through an interface that matches the sim's
`env.step(action) -> observation` contract.

Final hardware fleet: **4 Damiao motors + 4 ODrive motors + 1 Tmotor**, all on CAN.
Right now on the bench: **1 Damiao + 1 ODrive**.

Target architecture (two classes the user wants):
```
RobotInterface (sim-to-real bridge)
  step(action)      -> MotorController -> [Damiao joints, ODrive joints]   (CONTROL class)
  get_observation() <- FeedbackAggregator <- motor states + IMU            (FEEDBACK class)
```
- **MotorController** = control: owns all joints, enforces safety/limits, applies actions.
- **FeedbackAggregator** = feedback: motor states (pos/vel/torque) + IMU (body lin/ang vel,
  pitch, yaw). The IMU/body-state part is being written separately by a colleague ("Jim").
- `step()` + `get_observation()` are the exact hook the Isaac Lab substitution plugs into.

---

## 2. Current state of the code (what already exists)
All in `/home/dodo/Documents/dodo/`:

| File | Status | What it is |
|------|--------|-----------|
| `dm_position_control.py` | working | Original flat script: single Damiao motor sample move over CAN (MIT mode). |
| `damiao_joint.py` | working | `DamiaoJoint` + `DamiaoJointConfig`: clean class wrapper around one Damiao motor. Interface: `enable/disable`, `set_zero`, `get_position/get_state`, `send_position`, `goto` (blocking ramp), `start_move`/`is_moving`/`update` (non-blocking). Uses the `damiao_motor` pip library. Units: **radians**. |
| `odrive_position_control.py` | working (USB) | `OdriveJoint` + `OdriveJointConfig`: ODrive over **USB** (`odrive.find_sync`). Same interface as DamiaoJoint. Units: **turns**. Has gain config (`pos_gain`, `vel_gain`, `vel_integrator_gain`), software zero offset, and a reboot-tolerant `save()` for `save_configuration()`. **Going forward this file is repurposed as the USB gain-tuning / config tool**, NOT the runtime path. |
| `demo.py` | working (DM only) | Drives motors through a shared `spin_until` control loop. Currently DM-only; to be updated to use the new CAN `MotorController`. |

These were built collaboratively this session, mirroring each other so both motor types
expose an **identical interface** — that uniformity is the key design principle; keep it.

### What we learned tuning the ODrive (USB)
- It vibrated → that's controller gains. `vel_gain` too high = high-freq buzz; `pos_gain`
  too high = slow oscillation. Recipe: zero `vel_integrator_gain`, raise `vel_gain` to just
  before buzz then halve, raise `pos_gain` to just before overshoot then back off,
  restore integrator ≈ `0.5 * vel_gain`. Starting point used: `pos_gain≈15-20, vel_gain≈0.05,
  vel_integrator_gain≈0.1`.
- `save_configuration()` **reboots the drive and drops USB** → wrap in try/except and
  reconnect (this is implemented in `OdriveJoint.save()`).

---

## 3. SHORT-TERM PLAN — CAN migration (APPROVED, IN PROGRESS)
Full approved plan file: `/home/dodo/.claude/plans/ethereal-wondering-horizon.md`

**Goal of this milestone:** put BOTH motors on the same CAN bus (`can0`) behind one
unified `MotorController`, and expose the `step(action)`/`get_observation()` skeleton.

### Key technical findings (already settled — don't re-litigate)
- **One CAN bus works for both.** SocketCAN raw sockets each receive their *own copy* of
  every frame. We give each motor type its **own filtered socket** so they don't cross-talk:
  - `DaMiaoController.__init__` forwards `**kwargs` to `can.interface.Bus`
    (`Damiao Control/python-damiao-driver/damiao_motor/core/controller.py:145-156`), so we
    pass `can_filters=...` to it.
  - The DM frame router (`controller.py:365`) dispatches 8-byte frames by `data[0] & 0x0F`
    (NOT by arbitration id) → without a filter it could mis-ingest ODrive frames. The
    per-socket filter prevents this. **No patching of `damiao_motor` needed.**
- **ODrive over CAN ≠ the USB `odrive` library.** It uses the **"CAN Simple"** protocol via
  raw `python-can`. Arbitration id encoding: `arb_id = (node_id << 5) | cmd_id`.
- **Bus params:** `can0` @ **1 Mbit/s**. Bring up: `sudo ip link set can0 up type can bitrate 1000000`.
- **ODrive gains cannot be set over CAN Simple** — gains/modes are USB config persisted on
  the drive. Hence the USB file stays as the config tool; CAN is runtime-only.
- **CAN Simple command IDs in use:** `0x07` Set_Axis_State, `0x0B` Set_Controller_Mode,
  `0x0C` Set_Input_Pos, `0x01` Heartbeat (RX), `0x09` Get_Encoder_Estimates (RX),
  `0x1C` Get_Torques (RX). Axis states: IDLE=1, CLOSED_LOOP_CONTROL=8.
  Control mode: POSITION_CONTROL=3; input mode: PASSTHROUGH=1.
- **Node-id allocation (avoid arb-id collisions):** keep ODrive **node_id 7** (and 8 later) —
  windows `0xE0–0xFF` / `0x100–0x11F` don't collide with DM cmd `0x02`, fb `0x12`, or
  register id `0x7FF`. Avoid node 0 (low DM ids) and node 63 (collides with `0x7FF`).
- **Reference implementations already on disk (reuse these):**
  - `/home/dodo/Downloads/can_simple_utils.py` — clean `CanSimpleNode` (arb-id encoding, state set).
  - `/home/dodo/Desktop/dodo_ros2/dodo/dodo/odrive_interface.py` — raw python-can ODrive setup.
  - `/home/dodo/Documents/ros_odrive-main/odrive_node/src/odrive_can_node.cpp` — full cmd-id table.

### Files to create / change (the actual work)
1. **`odrive_can_setup.py`** (NEW, USB) — inspect + configure ODrive for CAN: print current
   CAN config; set `axis.config.can.node_id=7`, `can.config.baud_rate=1_000_000`, enable
   cyclic feedback (`heartbeat_msg_rate_ms`, `encoder_msg_rate_ms`, `torque_msg_rate_ms ≈ 10`),
   ensure POSITION/PASSTHROUGH + gains, `save_configuration()`. *Verify exact attribute paths
   against odrive 0.6.11 at implementation time.*
2. **`odrive_can_joint.py`** (NEW) — `OdriveCanJoint` + `OdriveCanJointConfig`. Mirrors the
   `OdriveJoint` interface but speaks CAN Simple via a **shared `can.Bus`** passed in.
   `send_position` → Set_Input_Pos `struct.pack('<fhh', pos, 0, 0)`; `enable/disable` →
   Set_Axis_State; `set_zero` → software offset; `process_frame(msg)` decodes Heartbeat
   (`<IBBBB`), Encoder_Estimates (`<ff`), Torques (`<ff`) into cached state. Copy the
   identical `_ramp_profile`/`goto`/`start_move`/`update` logic from `odrive_position_control.py:126-175`.
3. **`motor_controller.py`** (NEW) — `MotorController`. DM via `DaMiaoController(..., can_filters=<DM fb ids + 0x7FF>)`
   wrapped in `DamiaoJoint`; ODrive via a second `can.Bus(..., can_filters=[{"can_id": node_id<<5,
   "can_mask": 0x7E0}])` + a small daemon reader thread that drains `bus.recv()` into
   `OdriveCanJoint.process_frame`. Config-driven (lists of DM + ODrive configs). API:
   `enable_all/disable_all/set_zero_all/shutdown`, plus `step(action)` and `get_observation()`.
4. **`demo.py`** (UPDATE) — rebuild on `MotorController` (1 DM + 1 ODrive), zero/enable/sample
   move via the shared loop, shutdown.

### Verification (bench, user-run)
1. `sudo ip link set can0 up type can bitrate 1000000`
2. `python3 odrive_can_setup.py` (USB) → confirm node 7, 1M baud, feedback rates; let it save+reboot.
3. `candump can0` → see DM feedback + ODrive heartbeat (`0xE1`) and encoder (`0xE9`) frames.
4. Build `MotorController`, call `get_observation()` → positions read for both, no cross-talk.
5. `python3 demo.py` → both motors move, `get_position()` tracks; watch ODrive heartbeat error field.

### Progress / todo state at handoff
A task list exists (TaskCreate) with these items, **all still `pending` (no code written yet)**:
1. Create `odrive_can_setup.py` (USB config tool)
2. Create `odrive_can_joint.py` (CAN Simple driver)
3. Create `motor_controller.py` (unified MotorController)
4. Update `demo.py` to use MotorController over CAN
5. Syntax-check all new CAN files

### Decisions captured from the user (don't re-ask)
- ODrive transport → **move to CAN** (same `can0` bus as DM). Same interface, different ids.
- ODrive CAN config state → **unknown** → include the USB inspect/config step.
- Start order → **CAN migration first** (before safety layer).
- Bench now → **1 DM + 1 ODrive**; classes must be config-driven for the full fleet.
- Torque safety threshold "0.8" → **normalized 0–1 of rated torque** (for the later safety step).

---

## 4. LONG-TERM ROADMAP (sequenced, after CAN migration)
These are the user's next steps in order. NOT part of the current approved plan; the
`step()`/`get_observation()` skeleton from the CAN milestone is the hook they build on.

1. **Safety layer** (into `MotorController`):
   - Joint position limits (already have `pos_min`/`pos_max` per joint).
   - **Max joint torque threshold: 0.8 of rated torque reached → motor goes to IDLE.**
     (Normalized 0–1 of each motor's rated torque. ODrive torque via Get_Torques `0x1C`;
     DM torque via `get_states()["torq"]`.)
   - **Max velocity** limit (ODrive has `vel_limit` config; DM needs a software check).
   - General safety functions / e-stop → disable-all.
2. **CAN connection to both motors + the input/output function** — largely the current
   milestone; finalize the unified `step(action) -> observation` once safety is in.
3. **IMU / body-state module (colleague "Jim")** — a Python module producing body
   velocity, acceleration, raw pitch and yaw. Integrates into the FeedbackAggregator.
4. **Two-class consolidation:** finalize **1 class for motor control** + **1 class for
   feedback** (feedback comprehends both motor feedback AND IMU sensor feedback).
5. **Isaac Lab substitution (final step):** in the simulation repo, replace the Isaac Lab
   env/classes so they match this real-hardware env + classes — i.e. point the trained
   policy at `MotorController.step()` / `FeedbackAggregator.get_observation()`. Will need the
   sim's exact observation/action layout (order + units) — match it from the start so this
   step is a swap, not a rewrite. **Open item:** which Isaac Lab repo/env, and its obs/action spec.

---

## 5. Environment / gotchas
- Python: system `python3` (3.10) uses `~/.local/lib/python3.10/site-packages`. Installed:
  `odrive` 0.6.11.post0, `python-can`, `damiao_motor` 1.0.6. (There's also an editable
  `damiao_motor` 1.0.7b2 in the uv cache — the regular 1.0.6 is what `python3` imports.)
- `damiao_motor` editable source (for reading internals):
  `/home/dodo/Documents/dodo/Damiao Control/python-damiao-driver/damiao_motor/`
- Damiao units = **radians**; ODrive units = **turns** (1 turn = 2π rad). Keep native per
  joint for now; the rad/turns convention for the *policy vector* gets fixed in the Isaac Lab step.
- `can0` bitrate is set at OS level (`ip link`), NOT by python-can, for socketcan.
- The user separately asked about turning these files into a standalone git repo — answer was
  yes, the 4 files are self-contained (only cross-import by name + pip deps); just copy them
  into one folder and `git init`. requirements: `odrive==0.6.11.post0`, `python-can`, `damiao_motor==1.0.6`.

---

## 6. How to continue (for the next Claude)
1. Read the approved plan: `/home/dodo/.claude/plans/ethereal-wondering-horizon.md`.
2. Skim the existing classes (`damiao_joint.py`, `odrive_position_control.py`) to match style/interface.
3. Skim the references in §3 (`can_simple_utils.py`, `odrive_interface.py`, `odrive_can_node.cpp`).
4. Implement files 1→4 from §3 (CAN migration), syntax-check, then hand the bench
   verification steps to the user (hardware-in-the-loop, they run them).
5. Keep both motor types on the **identical interface** — that's the load-bearing principle.
