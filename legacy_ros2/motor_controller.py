"""
motor_controller.py — unified ROS2 motor-control node for the dodo robot.

Owns the WHOLE motor fleet (Damiao + ODrive) behind one motor-type-agnostic
interface and exposes it purely over ROS topics:

        subscribe  /motor_commands  (sensor_msgs/JointState)  -> target positions [rad]
        publish    /motor_states    (sensor_msgs/JointState)  <- pos / vel / effort [rad, rad/s, Nm]

This is a *pure* control node: it has no knowledge of the policy. The sim-to-real
policy contract (joint order, default pose, action scaling, observation assembly)
lives in the Env layer (rl_env.py / real_env.py); RealEnv publishes resolved
position targets here and subscribes to /motor_states, so SimEnv and RealEnv apply
actions identically. (ODrive integration merged from dodo_rl_hardware_S26/.)

Fleet (8 DOF):
  * Damiao DM4340P  -> hips   (left/right hip_roll + hip_pitch), native units = radians
  * ODrive (CAN Simple) -> knees + ankles, native units = turns (1 turn = 2*pi rad)

CAN topology
------------
Both motor types can share ONE bus (can0 @ 1 Mbit/s): SocketCAN gives each open
socket its own copy of every frame, so each motor type gets its own *filtered*
socket and never mis-ingests the other's frames. The installed `damiao_motor`
1.0.6 `DaMiaoController.__init__` does NOT accept `can_filters`, so the Damiao
filter is applied to its bus afterwards via `bus.set_filters(...)` (only needed
when Damiao and ODrive share a channel). If you have two USB-CAN adapters, set
`od_channel` to the second bus (e.g. can1) and no Damiao filter is applied.

Bring the bus up first (OS level, not python-can):
    sudo ip link set can0 up type can bitrate 1000000

Safety: every position command is clamped, and measured feedback is monitored,
against SAFETY_FACTOR (= 0.9) of each motor's rated position / velocity / torque
limits. A breach (or a motor fault flag) triggers an e-stop: all motors are
disabled and commands are ignored until the node is restarted.
Sim-to-real (IsaacLab) alignment
--------------------------------
DEPLOYMENT CONSTRAINT (from the professor): do NOT use base_lin_vel in the policy
observation. Base *linear* velocity can't be estimated reliably from the IMU
without sophisticated state estimation, so the policy must not depend on it. The
first deployment target is a STAND-AND-BALANCE policy (hold two-leg balance, resist
mild pushes) — that needs neither linear velocity nor velocity commands.

So the observation we target is:

  action      = per-joint position target, applied as
                target = default_joint_pos + action_scale * action   (radians, sim)
  observation = concat[ base_ang_vel(3), projected_gravity(3),
                        joint_pos_rel(n), joint_vel(n), last_action(n) ]

  EXCLUDED on purpose: base_lin_vel (unreliable from IMU) and velocity_commands
  (no command for a pure balance policy). The IsaacLab env must drop these terms
  too (observations.policy.base_lin_vel = None, .velocity_commands = None) so the
  trained obs layout matches this exactly.

This file owns everything MOTOR-derived: joint_pos_rel, joint_vel, last_action,
and it applies actions to the right motor in that motor's native units (Damiao =
radians, ODrive = turns). The IMU terms that ARE used (base_ang_vel from the gyro,
projected_gravity from accel/orientation) come from the FeedbackAggregator / IMU
module (colleague "Jim") and are stitched in via to_policy_vector(). Keep
POLICY_JOINTS in the SAME order as the sim's articulation joint order so the final
Isaac Lab swap is a one-liner, not a rewrite.

Run with:
    python3 motor_controller.py
"""

import math
import threading
import time
from typing import Dict, List, Optional, Union

import can
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from damiao_motor import DaMiaoController

from damiao_joint import DamiaoJoint, DamiaoJointConfig
from odrive_can_joint import OdriveCanJoint, OdriveCanJointConfig


TWO_PI = 2.0 * math.pi
DM_REGISTER_ID = 0x7FF  # Damiao register read/write reply arbitration id

# --- Safety factors / thresholds ------------------------------------------------
SAFETY_FACTOR = 0.9          # margin applied to position-clamp + velocity limit
TORQUE_IDLE_FACTOR = 0.8     # |torque| > 0.8 * rated  ->  that motor goes to IDLE
                             # (professor's spec; per-joint, NOT a full e-stop)
CMD_WATCHDOG_S = 0.5         # once commands are streaming, a gap this long -> e-stop
FB_WATCHDOG_S = 0.5          # a joint that WAS reporting goes silent this long -> e-stop
POS_OVERTRAVEL_MARGIN = 0.10 # rad past the hard URDF limit (measured) -> e-stop
STARTUP_GRACE_S = 1.0        # ignore safety trips for the first second (transients)

# --- Physical constraints of the legs/body (from dodo_daimao.urdf) --------------
# The mechanical range of each joint, in RADIANS. This is the single source of
# truth for position safety; _JointAdapter converts to each motor's native units
# (Damiao=rad, ODrive=turns) for clamping. Keep in sync with the URDF <limit> tags.
JOINT_LIMITS_RAD = {
    "hip_left":        (-0.35,   0.35),   "hip_right":       (-0.35,   0.35),    # hip roll
    "upper_leg_left":  (-1.57,   1.57),   "upper_leg_right": (-1.57,   1.57),    # hip pitch
    "lower_leg_left":  (-3.1416, 1.3963), "lower_leg_right": (-3.1416, 1.3963),  # knee
    "foot_left":       (-1.05,   1.57),   "foot_right":      (-1.05,   1.57),    # ankle
}
# Rated torque per joint (URDF effort limits, Nm): hips/upper 27, knee/ankle 9.
RATED_TORQUE_NM = {
    "hip_left": 27.0, "hip_right": 27.0, "upper_leg_left": 27.0, "upper_leg_right": 27.0,
    "lower_leg_left": 9.0, "lower_leg_right": 9.0, "foot_left": 9.0, "foot_right": 9.0,
}
JOINT_VEL_LIMIT_RAD = 6.0    # URDF velocity limit (rad/s), same for all joints.

# NOTE (ODrive gearing): unit conversion assumes 1 motor turn == 2*pi rad at the
# joint (direct drive). If the T-Motor/ODrive joints are geared, this scaling —
# and therefore BOTH the commands and these limits — must be divided by the gear
# ratio. Verify the real ratio before trusting the ODrive position clamps.

# Damiao fault status names (anything other than DISABLED/ENABLED) -> e-stop.
DM_FAULT_STATUS = {
    "OVER_VOLTAGE", "UNDER_VOLTAGE", "OVER_CURRENT",
    "MOS_OVER_TEMP", "ROTOR_OVER_TEMP", "LOST_COMM", "OVERLOAD",
}

AnyJoint = Union[DamiaoJoint, OdriveCanJoint]


# ---------------------------------------------------------------------------
# Fleet configuration (real robot)
# ---------------------------------------------------------------------------
# Joint NAMES match the URDF / IsaacLab articulation (hip_*=hip roll,
# upper_leg_*=hip pitch, lower_leg_*=knee, foot_*=ankle) so the whole stack
# (sim, policy, ROS topics, motors) shares ONE naming scheme — no remapping.
#
# Damiao hips (roll + pitch) — ids already set on the physical motors (DM4340P).
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

# ODrive knees (lower_leg_*) + ankles (foot_*) — T-Motor via ODrive, CAN Simple.
# TODO(user): set the real CAN node ids (as configured with odrive_can_setup.py)
# and, ideally, soft pos_min/pos_max (turns) for each joint's mechanical range.
# rated_torque = the sim effort-limit for the knee/ankle group (9 Nm), used to
# normalize the torque safety check; vel_limit (turns/s) bounds the velocity check.
OD_JOINTS = [
    OdriveCanJointConfig(name="lower_leg_left",  node_id=7,  vel_limit=10.0, rated_torque=9.0),
    OdriveCanJointConfig(name="foot_left",       node_id=8,  vel_limit=10.0, rated_torque=9.0),
    OdriveCanJointConfig(name="lower_leg_right", node_id=9,  vel_limit=10.0, rated_torque=9.0),
    OdriveCanJointConfig(name="foot_right",      node_id=10, vel_limit=10.0, rated_torque=9.0),
]


# ---------------------------------------------------------------------------
# Per-joint adapter: unifies units (rad <-> native) + safety limits + state.
# ---------------------------------------------------------------------------
class _JointAdapter:
    """Wraps any joint so the node treats Damiao (rad) and ODrive (turns)
    uniformly: positions/velocities are exposed to ROS and the policy in
    radians, while clamping and feedback monitoring happen in the joint's
    native units."""

    def __init__(self, joint: AnyJoint, logger=None):
        self.joint = joint
        self.name = joint.cfg.name
        self.is_odrive = isinstance(joint, OdriveCanJoint)
        self.idled = False              # set True once this joint is IDLEd by safety
        self._last_fb_time: Optional[float] = None  # last time we saw feedback

        cfg = joint.cfg

        # Position limits: URDF mechanical range (rad) with SAFETY_FACTOR margin,
        # converted to native units. A per-joint cfg.pos_min/max (native) may
        # further tighten but never loosen it.
        lo_rad, hi_rad = JOINT_LIMITS_RAD.get(self.name, (None, None))
        self.pos_hard_lo_rad, self.pos_hard_hi_rad = lo_rad, hi_rad  # for over-travel check
        self.pos_lo = self.to_native(lo_rad * SAFETY_FACTOR) if lo_rad is not None else None
        self.pos_hi = self.to_native(hi_rad * SAFETY_FACTOR) if hi_rad is not None else None
        cpm, cpx = getattr(cfg, "pos_min", None), getattr(cfg, "pos_max", None)
        if cpm is not None:
            self.pos_lo = cpm if self.pos_lo is None else max(self.pos_lo, cpm)
        if cpx is not None:
            self.pos_hi = cpx if self.pos_hi is None else min(self.pos_hi, cpx)
        if logger and (self.pos_lo is None or self.pos_hi is None):
            logger.warn(f"{self.name}: position UNCLAMPED (no URDF limit found).")

        # Velocity limit (native) from the URDF, with margin.
        self.vel_limit = self.to_native(SAFETY_FACTOR * JOINT_VEL_LIMIT_RAD)
        # Torque: rated from the URDF group; IDLE this joint above 0.8 * rated.
        self.rated_torque = RATED_TORQUE_NM.get(
            self.name, getattr(cfg, "rated_torque", None) or 9.0)
        self.torque_idle_limit = TORQUE_IDLE_FACTOR * self.rated_torque

    # ── unit conversion (native is turns for ODrive, rad for Damiao) ──
    def to_native(self, rad: float) -> float:
        return rad / TWO_PI if self.is_odrive else rad

    def to_rad(self, native: float) -> float:
        return native * TWO_PI if self.is_odrive else native

    def clamp_native(self, q: float) -> float:
        if self.pos_lo is not None:
            q = max(q, self.pos_lo)
        if self.pos_hi is not None:
            q = min(q, self.pos_hi)
        return q

    # ── normalized state read ─────────────────────────────────────
    def read_state(self) -> dict:
        st = self.joint.get_state() or {}
        pos_native = float(st.get("pos", 0.0))
        vel_native = float(st.get("vel", 0.0))
        # Damiao reports torque under "torq"; ODrive under "torque".
        torque = float(st.get("torque", st.get("torq", 0.0)))

        fault = None
        if self.is_odrive:
            if st.get("axis_error"):
                fault = f"ODrive axis_error=0x{int(st['axis_error']):X}"
        else:
            if st.get("status") in DM_FAULT_STATUS:
                fault = f"status '{st.get('status')}'"

        return {
            "pos_rad": self.to_rad(pos_native),
            "vel_rad": self.to_rad(vel_native),
            "pos_native": pos_native,
            "vel_native": vel_native,
            "torque": torque,
            "fault": fault,
            "has_feedback": bool(st),
        }


class MotorController(Node):
    """Pure ROS2 control node owning the whole motor fleet."""

    def __init__(self):
        super().__init__("motor_controller")

        # ----- Parameters -----
        self.declare_parameter("dm_channel", "can0")
        self.declare_parameter("od_channel", "can0")  # set to e.g. can1 with a 2nd adapter
        self.declare_parameter("bustype", "socketcan")
        self.declare_parameter("command_topic", "/motor_commands")
        self.declare_parameter("state_topic", "/motor_states")
        self.declare_parameter("update_rate", 200.0)   # Hz
        self.declare_parameter("move_duration", 1.0)    # s, ramp time for topic commands

        dm_channel = self.get_parameter("dm_channel").value
        od_channel = self.get_parameter("od_channel").value
        bustype = self.get_parameter("bustype").value
        cmd_topic = self.get_parameter("command_topic").value
        state_topic = self.get_parameter("state_topic").value
        self.update_rate = float(self.get_parameter("update_rate").value)
        self.move_duration = float(self.get_parameter("move_duration").value)

        self._estopped = False
        self._stop = threading.Event()
        self._start_time = time.time()
        self._got_cmd = False          # have we ever received a /motor_commands msg?
        self._last_cmd_time = 0.0       # time of the last command (for the watchdog)

        # ── Damiao: one controller/socket ─────────────────────────
        self.dm_controller = DaMiaoController(channel=dm_channel, bustype=bustype)
        self.dm_joints = [DamiaoJoint(self.dm_controller, c) for c in DM_JOINTS]

        # ── ODrive: a second (filtered) socket + reader thread ─────
        self.od_bus: Optional[can.BusABC] = None
        self.od_joints: List[OdriveCanJoint] = []
        self._od_by_node: Dict[int, OdriveCanJoint] = {}
        self._reader_thread: Optional[threading.Thread] = None
        if OD_JOINTS:
            od_filters = [{"can_id": c.node_id << 5, "can_mask": 0x7E0} for c in OD_JOINTS]
            self.od_bus = can.Bus(channel=od_channel, interface=bustype,
                                  can_filters=od_filters)
            for c in OD_JOINTS:
                j = OdriveCanJoint(self.od_bus, c)
                self.od_joints.append(j)
                self._od_by_node[c.node_id] = j
            # When DM and ODrive share a bus, filter the DM socket so it doesn't
            # mis-ingest ODrive frames (installed lib can't take can_filters in
            # __init__, so apply to the bus directly — see module docstring).
            if od_channel == dm_channel:
                dm_filters = [{"can_id": c.feedback_id, "can_mask": 0x7FF} for c in DM_JOINTS]
                dm_filters.append({"can_id": DM_REGISTER_ID, "can_mask": 0x7FF})
                self.dm_controller.bus.set_filters(dm_filters)
            self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
            self._reader_thread.start()

        # Unified joint list + name lookup + adapters (Damiao first, then ODrive).
        self.joints: List[AnyJoint] = [*self.dm_joints, *self.od_joints]
        self.adapters = {j.cfg.name: _JointAdapter(j, self.get_logger()) for j in self.joints}

        self.enable_all()

        # ----- ROS interfaces -----
        self.cmd_sub = self.create_subscription(
            JointState, cmd_topic, self.command_callback, 10)
        self.state_pub = self.create_publisher(JointState, state_topic, 10)
        self.timer = self.create_timer(1.0 / self.update_rate, self.control_loop)

        self.get_logger().info(
            f"motor_controller up: {len(self.dm_joints)} Damiao (on {dm_channel}) + "
            f"{len(self.od_joints)} ODrive (on {od_channel}), loop {self.update_rate:.0f} Hz. "
            f"Listening on {cmd_topic}.")
        for name, a in self.adapters.items():
            lo = f"{a.pos_lo:+.2f}" if a.pos_lo is not None else "-inf"
            hi = f"{a.pos_hi:+.2f}" if a.pos_hi is not None else "+inf"
            vl = f"{a.vel_limit:.1f}" if a.vel_limit is not None else "n/a"
            tl = f"{a.torque_idle_limit:.1f}"
            unit = "turns" if a.is_odrive else "rad"
            self.get_logger().info(
                f"  {name}: pos[{lo},{hi}] {unit}, |vel|<={vl}, "
                f"torque->IDLE @{tl} Nm (0.8 x {a.rated_torque:.0f})")

    # ── ODrive reader thread ──────────────────────────────────────
    def _reader_loop(self):
        while not self._stop.is_set():
            msg = self.od_bus.recv(timeout=0.1)
            if msg is None:
                continue
            j = self._od_by_node.get(msg.arbitration_id >> 5)
            if j is not None:
                j.process_frame(msg)

    # ── fleet management ──────────────────────────────────────────
    def enable_all(self):
        self.dm_controller.enable_all()
        for j in self.od_joints:
            j.enable()
        time.sleep(0.1)

    def disable_all(self):
        try:
            self.dm_controller.disable_all()
        except Exception as e:
            self.get_logger().error(f"Damiao disable_all failed: {e}")
        for j in self.od_joints:
            try:
                j.disable()
            except Exception:
                pass

    def set_zero_all(self):
        for j in self.joints:
            j.set_zero()

    # ------------------------------------------------------------------ #
    # ROS path: command handling
    # ------------------------------------------------------------------ #
    def command_callback(self, msg: JointState):
        """Move joints to commanded target positions [radians] (smooth ramp)."""
        if self._estopped:
            self.get_logger().warn("Ignoring command: controller is e-stopped.")
            return
        if not msg.position:
            self.get_logger().warn("Command had no positions; ignored.")
            return

        self._got_cmd = True
        self._last_cmd_time = time.time()

        # Match by name when names are given, else by joint order.
        if msg.name:
            pairs = []
            for i, name in enumerate(msg.name):
                if i >= len(msg.position):
                    break
                a = self.adapters.get(name)
                if a is None:
                    self.get_logger().warn(f"Unknown joint '{name}' in command; skipped.")
                    continue
                pairs.append((a, msg.position[i]))
        else:
            pairs = list(zip(self.adapters.values(), msg.position))

        for a, target_rad in pairs:
            if a.idled:
                continue  # this joint was IDLEd by safety; don't re-command it
            native = a.clamp_native(a.to_native(float(target_rad)))
            if abs(a.to_rad(native) - float(target_rad)) > 1e-6:
                self.get_logger().warn(
                    f"{a.name}: target {float(target_rad):+.3f} rad clamped to "
                    f"{a.to_rad(native):+.3f} rad (safety limit).")
            a.joint.start_move(native, duration=self.move_duration)

    # ------------------------------------------------------------------ #
    # Control + safety loop
    # ------------------------------------------------------------------ #
    def control_loop(self):
        if self._estopped:
            return

        for joint in self.joints:
            if self.adapters[joint.cfg.name].idled:
                continue  # IDLEd by safety — leave it disabled, don't re-drive it
            try:
                joint.update()  # drive ramp / hold last target
            except Exception as e:
                self.get_logger().error(f"{joint.cfg.name}: update failed: {e}")

        states = {name: a.read_state() for name, a in self.adapters.items()}
        self._publish_states(states)
        self._safety_check(states)
        self._watchdog_check(states)

    def _publish_states(self, states: dict):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        for name, st in states.items():
            msg.name.append(name)
            msg.position.append(st["pos_rad"])   # radians
            msg.velocity.append(st["vel_rad"])   # rad/s
            msg.effort.append(st["torque"])      # Nm
        self.state_pub.publish(msg)

    def _safety_check(self, states: dict):
        """Per-cycle safety. Severe/systemic breaches (fault, over-speed, position
        over-travel) -> full e-stop; a torque breach on ONE joint -> IDLE just that
        joint (professor's spec: |torque| > 0.8*rated -> motor to IDLE)."""
        # Small startup grace so transient feedback doesn't nuisance-trip.
        if time.time() - self._start_time < STARTUP_GRACE_S:
            return
        for name, st in states.items():
            if not st["has_feedback"]:
                continue
            a = self.adapters[name]
            a._last_fb_time = time.time()
            if a.idled:
                continue  # already protected

            # (1) Motor-reported fault -> systemic -> e-stop everything.
            if st["fault"]:
                self._estop(f"{name} reported fault: {st['fault']}")
                return
            # (2) Over-speed -> instability -> e-stop everything.
            if a.vel_limit is not None and abs(st["vel_native"]) > a.vel_limit:
                self._estop(f"{name} velocity {abs(st['vel_native']):.2f} > "
                            f"{a.vel_limit:.2f} ({'turns/s' if a.is_odrive else 'rad/s'})")
                return
            # (3) Measured position past the hard mechanical limit -> e-stop.
            if a.pos_hard_lo_rad is not None:
                p = st["pos_rad"]
                if (p < a.pos_hard_lo_rad - POS_OVERTRAVEL_MARGIN or
                        p > a.pos_hard_hi_rad + POS_OVERTRAVEL_MARGIN):
                    self._estop(f"{name} position {p:+.3f} rad past mechanical limit "
                                f"[{a.pos_hard_lo_rad:+.3f},{a.pos_hard_hi_rad:+.3f}]")
                    return
            # (4) Torque over 0.8*rated -> IDLE just this motor (per professor).
            if abs(st["torque"]) > a.torque_idle_limit:
                self._idle_joint(name, f"torque {abs(st['torque']):.2f} > "
                                        f"{a.torque_idle_limit:.2f} Nm "
                                        f"(0.8 x {a.rated_torque:.0f} Nm rated)")

    def _watchdog_check(self, states: dict):
        """E-stop on a stalled command stream or a joint that went silent."""
        if time.time() - self._start_time < STARTUP_GRACE_S or self._estopped:
            return
        now = time.time()
        # Command watchdog: only after the stream has started (avoids tripping while idle).
        if self._got_cmd and (now - self._last_cmd_time) > CMD_WATCHDOG_S:
            self._estop(f"no /motor_commands for {now - self._last_cmd_time:.2f}s "
                        f"(> {CMD_WATCHDOG_S}s watchdog)")
            return
        # Feedback watchdog: a joint that was reporting has gone silent (lost comm).
        for name, a in self.adapters.items():
            if a.idled:
                continue
            if a._last_fb_time is not None and (now - a._last_fb_time) > FB_WATCHDOG_S:
                self._estop(f"{name} feedback silent for {now - a._last_fb_time:.2f}s "
                            f"(> {FB_WATCHDOG_S}s watchdog)")
                return

    def _idle_joint(self, name: str, reason: str):
        """Disable a single motor (send it to IDLE) without stopping the fleet."""
        a = self.adapters[name]
        if a.idled:
            return
        a.idled = True
        self.get_logger().error(f"IDLE {name}: {reason}. Disabling this motor.")
        try:
            a.joint.disable()
        except Exception as e:
            self.get_logger().error(f"{name}: disable failed: {e}")

    def _estop(self, reason: str):
        if self._estopped:
            return
        self._estopped = True
        self.get_logger().error(f"E-STOP: {reason}. Disabling all motors.")
        self.disable_all()

    '''
       # ── sim-to-real hook: step / observe ──────────────────────────
    def step(self, action: Sequence[float]) -> None:
        """ simulation runs at 100Hz and hardware at 50Hz -> MISSING decimation factor (?)
        Apply one policy action. 

        action[i] corresponds to policy_joints[i]; target (rad) =
        default_pos + action_scale * action[i], converted to the joint's native
        units and sent. Mirrors IsaacLab JointPositionAction(use_default_offset).
        """
        if len(action) != len(self.policy_joints):
            raise ValueError(
                f"action len {len(action)} != n joints {len(self.policy_joints)}")
        for i, (pj, j) in enumerate(zip(self.policy_joints, self._policy_objs)):
            target_rad = pj.default_pos + self.cfg.action_scale * float(action[i])
            j.send_position(self._rad_to_native(j, target_rad))
        self._last_action = [float(a) for a in action]

    def get_observation(self) -> dict:
        """Return the MOTOR-derived part of the observation, in policy order.

        joint_pos_rel and joint_vel are in RADIANS / rad·s (sim convention),
        regardless of each motor's native units. The used IMU term (base_ang_vel)
        is left None for the FeedbackAggregator (Jim) to fill; projected_gravity
        likewise. base_lin_vel is intentionally absent (see module docstring).
        Use to_policy_vector() to assemble the full balance-policy observation.
        """
        joint_pos_rel, joint_vel = [], []
        for pj, j in zip(self.policy_joints, self._policy_objs):
            joint_pos_rel.append(self._native_to_rad(j, j.get_position()) - pj.default_pos)
            vel_native = j.get_state().get("vel", 0.0)
            joint_vel.append(self._native_to_rad(j, vel_native))
        return {
            # motor-derived (filled here)
            "joint_pos_rel": joint_pos_rel,
            "joint_vel": joint_vel,
            "last_action": list(self._last_action),
            # IMU terms used by the balance policy — filled by FeedbackAggregator
            "base_ang_vel": None,
            "projected_gravity": None,
            # NOTE: base_lin_vel and velocity_commands are intentionally excluded.
        }

    def to_policy_vector(self,
                         base_ang_vel: Sequence[float],
                         projected_gravity: Sequence[float]) -> List[float]:
        """Assemble the stand-and-balance policy observation in canonical order:
        [base_ang_vel(3), projected_gravity(3),
         joint_pos_rel(n), joint_vel(n), last_action(n)].

        Caller supplies the two IMU terms (from Jim's module). base_lin_vel and
        velocity_commands are deliberately NOT part of this vector — the trained
        IsaacLab policy must be configured to match (no base_lin_vel, no commands).
        """
        obs = self.get_observation()
        return [
            *base_ang_vel,
            *projected_gravity,
                        *obs["joint_pos_rel"],
            *obs["joint_vel"],
            *obs["last_action"],
        ]
        '''

    # ------------------------------------------------------------------ #
    # Shutdown
    # ------------------------------------------------------------------ #
    def shutdown(self):
        self.get_logger().info("Shutting down: disabling motors.")
        self._stop.set()
        if self._reader_thread is not None:
            self._reader_thread.join(timeout=1.0)
        self.disable_all()
        try:
            self.dm_controller.shutdown()
        except Exception as e:
            self.get_logger().error(f"Damiao shutdown failed: {e}")
        if self.od_bus is not None:
            try:
                self.od_bus.shutdown()
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
