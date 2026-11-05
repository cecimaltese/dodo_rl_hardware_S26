"""
fleet_config.py — the SINGLE SOURCE OF TRUTH for the robot's motors and their limits.

Why this file exists: the motor controller (then a ROS2 node) and `read_motor_state.py`
each used to carry their own hand-copied fleet config with a "kept in sync" comment on
top. They drifted — the ODrive node ids were updated in one and not the other, so the
diagnostic tool would have hunted for nodes 7-10 that no longer exist and reported every
ODrive OFFLINE. A comment is not a mechanism. Import from here instead. (motor_service.py,
the ZMQ port, grew a third copy that went stale the same way — node ids 7-10 and a single
shared bus — until it was switched to this module.)

Import-only: nothing here opens a bus, so the services and every bench tool can share
it.

Joint NAMES match the URDF / IsaacLab articulation, so the whole stack (sim, policy,
services, motors) shares ONE naming scheme with no remapping:
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

# --- Joint-space PD gains -------------------------------------------------------
# The PD stand's gains, per joint group: dodo_assets' mjcf/scene.xml (Shafeef), which
# sim_service --scene runs. Joint-side units: kp [Nm/rad], kd [Nm s/rad].
#   * Damiao: MIT mode is the same law at the output shaft (tau = kp*(q*-q) +
#     kd*(qd*-qd)), so these go into every MIT frame as they are (damiao_motor:
#     kp 0-500, kd 0-5).
#   * ODrive: a cascaded P-PI on the MOTOR side. motor_service sets it over CAN at
#     enable (real2sim.odrive_gains): vel_integrator_gain 0 makes it a PD, and with
#     N = GEAR_RATIO, K = N^2 pos_gain vel_gain / 2pi, D = N^2 vel_gain / 2pi.
# motor_service --gain-scale / ctl `gains` scales kp by s and kd by sqrt(s) (same
# damping ratio) for soft first tests.
JOINT_KP = {"hip": 150.0, "upper_leg": 150.0, "lower_leg": 50.0, "foot": 50.0}
JOINT_KD = {"hip": 1.5, "upper_leg": 1.5, "lower_leg": 0.8, "foot": 0.8}


def joint_group(joint: str) -> str:
    """'hip', 'upper_leg', 'lower_leg' or 'foot' for a joint name."""
    return joint.rsplit("_", 1)[0]


# --- Damiao hips (roll + pitch) -------------------------------------------------
# ids are already set on the physical motors. motor_type MUST be "4340P": the
# DamiaoJointConfig default is "4310", a much weaker motor (+/-10 Nm vs +/-28 Nm), and
# the preset drives the MIT-mode P/V/T encoding — a wrong one mis-scales every command.
# kp/kd: the PD stand's (JOINT_KP/JOINT_KD above). They were 30/0.5 until 2026-11-05.
DM_JOINTS = [
    DamiaoJointConfig(name="hip_left",        motor_id=0x01, feedback_id=0x11,
                      motor_type="4340P", kp=JOINT_KP["hip"], kd=JOINT_KD["hip"]),  # left hip roll
    DamiaoJointConfig(name="upper_leg_left",  motor_id=0x02, feedback_id=0x12,
                      motor_type="4340P", kp=JOINT_KP["upper_leg"], kd=JOINT_KD["upper_leg"]),  # left hip pitch
    DamiaoJointConfig(name="hip_right",       motor_id=0x05, feedback_id=0x15,
                      motor_type="4340P", kp=JOINT_KP["hip"], kd=JOINT_KD["hip"]),  # right hip roll
    DamiaoJointConfig(name="upper_leg_right", motor_id=0x06, feedback_id=0x16,
                      motor_type="4340P", kp=JOINT_KP["upper_leg"], kd=JOINT_KD["upper_leg"]),  # right hip pitch
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
# The node -> joint assignment below was a placeholder (ascending id = left knee, left
# ankle, right knee, right ankle) until 2026-11-05: the calibration's main pass, which
# picks the motor that MOVED, measured exactly this mapping for all four (joint_calib's
# `motor` fields), and --ratio re-checked the knees and ankles.
#
# Position limits: motor_service clamps in joint radians against the measured stops
# (STOP_MARGIN_RAD, real2sim.command_band); pos_min/pos_max (turns) stay unset.
OD_JOINTS = [
    OdriveCanJointConfig(name="lower_leg_left",  node_id=3, vel_limit=10.0, rated_torque=9.0),
    OdriveCanJointConfig(name="foot_left",       node_id=4, vel_limit=10.0, rated_torque=9.0),
    OdriveCanJointConfig(name="lower_leg_right", node_id=7, vel_limit=10.0, rated_torque=9.0),
    OdriveCanJointConfig(name="foot_right",      node_id=8, vel_limit=10.0, rated_torque=9.0),
]

# --- Physical constraints of the legs/body --------------------------------------
# The MECHANICAL STOPS are measured, not configured: joint_calib.py's sim_lo/sim_hi
# (calibrate_joints.py --limits, 2026-11-05), in sim radians, read through
# real2sim.stops(). motor_service e-stops a joint measured past them (+ its
# POS_OVERTRAVEL_MARGIN) and clamps commands STOP_MARGIN_RAD inside them
# (real2sim.command_band). The July JOINT_LIMITS_RAD table that used to be here was in
# the raw motor frame (no sign/zero) and went with the calibration wiring.

# Reference poses for motor_service's ctl `calibrate`: where the ODrive joints (knees,
# ankles) physically ARE when it runs, in sim radians. An ODrive zeroes its encoder at
# power-up, so this is how its angle is re-established every session (no homing drive:
# it could overload the 3D-printed parts and belts). The Damiaos keep their measured
# zero (joint_calib.py) across power cycles and aren't touched.
#   straight  legs hanging straight down on the gantry: thigh and shank as in the
#             calibration's zero pose (the URDF's q = 0: the knee housing jogs forward,
#             the soles level). Set by eye, so good to a few degrees.
# TODO: the bird SITTING pose (HANDOFF §3), with the knees/ankles against measured
# stops, once it's measured on the robot.
REFERENCE_POSES = {
    "straight": {"lower_leg_left": 0.0, "lower_leg_right": 0.0,
                 "foot_left": 0.0, "foot_right": 0.0},
}

# Joint torque the ODrives report: Get_Torques' estimate is Iq x
# motor.config.torque_constant, i.e. MOTOR-side Nm when torque_constant is 8.27/KV
# (0.0276 Nm/A for the KV300 MN4004), as configured by WS25 (assumed, not read back).
# motor_service multiplies by GEAR_RATIO for joint Nm (gearbox losses ignored). If a
# drive's torque_constant was set to include the gearing, this must become False.
ODRIVE_TORQUE_IS_MOTOR_SIDE = True

# Rated torque per joint (URDF effort limits, Nm): hips/upper 27, knee/ankle 9.
# Matches the damiao_motor "4340P" preset (+/-28 Nm) for the hips.
RATED_TORQUE_NM = {
    "hip_left": 27.0, "hip_right": 27.0, "upper_leg_left": 27.0, "upper_leg_right": 27.0,
    "lower_leg_left": 9.0, "lower_leg_right": 9.0, "foot_left": 9.0, "foot_right": 9.0,
}

JOINT_VEL_LIMIT_RAD = 6.0    # URDF velocity limit (rad/s), same for all joints.

# Position commands are clamped this far INSIDE each joint's measured hard stops
# (joint_calib.py's sim_lo/sim_hi, in sim radians: real2sim.command_band). A fixed
# margin, unlike the old 0.9 x limit, which shrank toward 0 rad and gave lopsided
# margins. motor_service and sim_service both clamp this way (2026-11-05).
STOP_MARGIN_RAD = 0.10

# Feedback older than this is not "current". Both drivers cache their last frame
# forever, so every consumer must check feedback_age() against this instead of
# trusting the cached numbers. Healthy rates: ODrive cyclic encoder frames every 10 ms
# (odrive_can_setup.py), a Damiao reply to every command (200 Hz from motor_service).
# 0.1 s = ~10 missed frames. Silence beyond motor_service.FB_WATCHDOG_S is an e-stop.
FEEDBACK_FRESH_S = 0.1

# ODrive gearing: MOTOR turns per JOINT turn, per joint. Every rad <-> turns conversion
# (real2sim.py, used by motor_service, calibrate_joints and the visualizer) divides by
# it. MEASURED 2026-11-05 with `calibrate_joints.py --ratio` (phone on the moving link)
# and checked by eye in the visualizer. (None would mean unmeasured, read as direct
# drive.) A calibration (joint_calib.py) records the ratio it was made with, and the
# tools refuse an entry whose ratio no longer matches: change a number here and the
# knee/ankle calibration must be redone.
GEAR_RATIO = {
    "lower_leg_left": 25.6, "lower_leg_right": 25.6,   # knee
    "foot_left": 28.5,      "foot_right": 28.5,        # ankle
}
