"""
fleet_config.py — the SINGLE SOURCE OF TRUTH for the robot's motors and their limits.

Why this file exists: `motor_controller.py` and `read_motor_state.py` each used to carry
their own hand-copied fleet config with a "kept in sync with motor_controller.py" comment
on top. They drifted — the ODrive node ids were updated in one and not the other, so the
diagnostic tool would have hunted for nodes 7-10 that no longer exist and reported every
ODrive OFFLINE. A comment is not a mechanism. Import from here instead. (motor_service.py,
the ZMQ port, grew a third copy that went stale the same way — node ids 7-10 and a single
shared bus — until it was switched to this module.)

Deliberately ROS-free: the bench tools (`read_motor_state.py`, `odrive_can_scan.py`) must
run without a ROS install, so nothing here may import rclpy.

Joint NAMES match the URDF / IsaacLab articulation, so the whole stack (sim, policy, ROS
topics, motors) shares ONE naming scheme with no remapping:
    hip_*       = hip roll     (Damiao)
    upper_leg_* = hip pitch    (Damiao)
    lower_leg_* = knee         (ODrive)
    foot_*      = ankle        (ODrive)
"""

from damiao_joint import DamiaoJointConfig
from odrive_can_joint import OdriveCanJointConfig

# --- CAN buses ------------------------------------------------------------------
# Two buses, one USB-CAN adapter each (the real robot wiring, and the same split the
# WS25 stack used). Separate electrical buses = no cross-talk, no id collisions, and
# no Damiao frame filter needed.
#
# These are the STABLE names from 99-dodo-can.rules, pinned to each adapter's serial.
# Do NOT go back to can0/can1: those are enumeration order, not device identity — adapter
# 0038.. was observed being renamed can0 -> can1 simply by plugging the second one in,
# which silently swaps the Damiao and ODrive buses with no error.
#   sudo ip link set can_dm up type can bitrate 1000000
#   sudo ip link set can_od up type can bitrate 1000000
DM_CHANNEL = "can_dm"
OD_CHANNEL = "can_od"
BUSTYPE = "socketcan"
BITRATE = 1_000_000   # set at OS level: sudo ip link set canX up type can bitrate 1000000

# --- Damiao hips (roll + pitch) -------------------------------------------------
# ids are already set on the physical motors. motor_type MUST be "4340P": the
# DamiaoJointConfig default is "4310", a much weaker motor (+/-10 Nm vs +/-28 Nm), and
# the preset drives the MIT-mode P/V/T encoding — a wrong one mis-scales every command.
# TODO(bench): kp/kd 30/0.5 are the legacy values; the trained policy expects the
# behavior of IsaacLab stiffness 32 / damping 3.0 — calibrate on the bench
# (Damiao MIT units are NOT IsaacLab units; match step response, don't copy).
DM_JOINTS = [
    DamiaoJointConfig(name="hip_left",        motor_id=0x01, feedback_id=0x11,
                      motor_type="4340P", kp=30.0, kd=0.5),  # left hip roll
    DamiaoJointConfig(name="upper_leg_left",  motor_id=0x02, feedback_id=0x12,
                      motor_type="4340P", kp=30.0, kd=0.5),  # left hip pitch
    DamiaoJointConfig(name="hip_right",       motor_id=0x05, feedback_id=0x15,
                      motor_type="4340P", kp=30.0, kd=0.5),  # right hip roll
    DamiaoJointConfig(name="upper_leg_right", motor_id=0x06, feedback_id=0x16,
                      motor_type="4340P", kp=30.0, kd=0.5),  # right hip pitch
]

# --- ODrive knees (lower_leg_*) + ankles (foot_*) -------------------------------
# T-Motor via ODrive, CAN Simple.
#
# NODE IDS ARE 3, 4, 7, 8 — MEASURED OFF THE LIVE BUS 2026-07-16 with
# `odrive_can_scan.py --channel can1`: all four answer RTR and stream cyclic feedback
# at ~100 Hz. These were briefly set to 0-3 here on the belief the web GUI had
# reassigned them; it had NOT (its USB-CAN connect failed with "could not claim
# interface 0" — the gs_usb kernel driver owns the adapter, so the browser never
# reached the drives). The drives are still on last semester's WS25 ids, which
# `dodo_robot_ROS2_WS25/.../canbus_node.launch.py` confirms: can1:3/4/7/8.
# TRUST THE BUS, NOT THE GUI. Re-run the scan after any ODrive reconfiguration.
#
# 3/4/7/8 are also collision-safe even on a SHARED bus (windows 0x060-0x07F, 0x080-
# 0x09F, 0x0E0-0x0FF, 0x100-0x11F never touch the Damiao ids 0x01-0x06 / 0x11-0x16 /
# 0x7FF), so there is no reason to renumber them. Node 0 would NOT be safe — its
# window 0x000-0x01F swallows every Damiao command and feedback id.
#
# !! THE NODE -> JOINT ASSIGNMENT BELOW IS A GUESS !!
# The scan proves WHICH nodes exist, not which leg each one drives. WS25 never
# recorded a mapping either — it used the raw "can1:3" as the joint name throughout.
# Ordering here is ascending-id = left-knee/left-ankle/right-knee/right-ankle purely
# as a placeholder. VERIFY PHYSICALLY (command one node a few degrees on the gantry,
# see which joint moves) before commanding motion — a swapped knee/ankle is exactly
# as destructive as the L/R sign mirroring.
#
# TODO(user): set soft pos_min/pos_max (TURNS) per joint. ODrive position is otherwise
# unclamped except by JOINT_LIMITS_RAD below, which assumes direct drive (see NOTE).
OD_JOINTS = [
    OdriveCanJointConfig(name="lower_leg_left",  node_id=3, vel_limit=10.0, rated_torque=9.0),
    OdriveCanJointConfig(name="foot_left",       node_id=4, vel_limit=10.0, rated_torque=9.0),
    OdriveCanJointConfig(name="lower_leg_right", node_id=7, vel_limit=10.0, rated_torque=9.0),
    OdriveCanJointConfig(name="foot_right",      node_id=8, vel_limit=10.0, rated_torque=9.0),
]

# --- Physical constraints of the legs/body --------------------------------------
# Mechanical range of each joint, in RADIANS — the single source of truth for position
# safety. _JointAdapter converts to each motor's native units (Damiao=rad, ODrive=turns).
#
# The Damiao rows are REAL MEASURED ranges (2026-07-06), NOT the URDF's. They are
# asymmetric and left/right MIRRORED, while the URDF gives both sides identical
# symmetric limits (hip +/-0.35, upper_leg +/-1.57) and the same axis. That mismatch is
# an open BLOCKER: it implies the real motor frames are mirrored vs the sim, and there is
# NO per-joint sign/zero layer in the control path. Calibrate with read_motor_state.py
# (LIMP) before commanding motion. The ODrive rows are still the URDF values = UNMEASURED.
JOINT_LIMITS_RAD = {
    # Damiao — measured on the robot
    "hip_left":        (-0.966,  0.641),  "hip_right":       (-0.544, 1.104),   # hip roll
    "upper_leg_left":  (-0.324,  2.379),  "upper_leg_right": (-2.303, 0.293),   # hip pitch
    # ODrive — URDF values, NOT yet measured
    "lower_leg_left":  (-3.1416, 1.3963), "lower_leg_right": (-3.1416, 1.3963), # knee
    "foot_left":       (-1.05,   1.57),   "foot_right":      (-1.05,   1.57),   # ankle
}

# Boot pose: where the ODrive joints physically ARE at power-on. The startup convention
# (no homing drive — it could overload the 3D-printed structure/belts): switch the robot
# on with the knees and ankles folded ALL THE WAY BACK against their mechanical stops;
# motor_service's ctl `calibrate` then maps each joint's turns-since-boot onto these
# angles. These are the URDF limits = PLACEHOLDERS. `calibrate_joints.py --limits`
# measures the real stops (its sim_lo/sim_hi) — put those here once measured. Verify
# after `calibrate`: `dodo_ctl status` must show these values, and nudging each joint by
# hand must change it with the expected sign. The e-stop over-travel margin covers ±0.10
# rad of error.
BOOT_POSE_RAD = {
    "lower_leg_left": -3.1416, "lower_leg_right": -3.1416,  # knee folded back (min limit)
    "foot_left": 1.57,         "foot_right": 1.57,          # foot folded against shank (max limit)
}

# Rated torque per joint (URDF effort limits, Nm): hips/upper 27, knee/ankle 9.
# Matches the damiao_motor "4340P" preset (+/-28 Nm) for the hips.
RATED_TORQUE_NM = {
    "hip_left": 27.0, "hip_right": 27.0, "upper_leg_left": 27.0, "upper_leg_right": 27.0,
    "lower_leg_left": 9.0, "lower_leg_right": 9.0, "foot_left": 9.0, "foot_right": 9.0,
}

JOINT_VEL_LIMIT_RAD = 6.0    # URDF velocity limit (rad/s), same for all joints.

# NOTE (ODrive gearing): unit conversion assumes 1 motor turn == 2*pi rad at the joint
# (direct drive). If the T-Motor/ODrive joints are geared, this scaling — and therefore
# BOTH the commands and the limits above — must be divided by the gear ratio. VERIFY the
# real ratio before trusting the ODrive position clamps.
