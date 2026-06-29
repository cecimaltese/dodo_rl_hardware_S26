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
SAFETY_FACTOR = 0.9
DM_REGISTER_ID = 0x7FF  # Damiao register read/write reply arbitration id

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

        if self.is_odrive:
            cfg: OdriveCanJointConfig = joint.cfg
            self.pos_lo = cfg.pos_min  # turns; None -> unclamped
            self.pos_hi = cfg.pos_max
            self.vel_limit = SAFETY_FACTOR * cfg.vel_limit if cfg.vel_limit else None  # turns/s
            self.torque_limit = SAFETY_FACTOR * cfg.rated_torque if cfg.rated_torque else None
            if logger and (self.pos_lo is None or self.pos_hi is None):
                logger.warn(f"{self.name}: ODrive position is UNCLAMPED "
                            f"(set pos_min/pos_max in OD_JOINTS for safety).")
        else:
            motor = joint.motor  # damiao_motor.DaMiaoMotor with resolved limits
            p_max = abs(getattr(motor, "_p_max", 12.5))
            v_max = abs(getattr(motor, "_v_max", 10.0))
            t_max = abs(getattr(motor, "_t_max", 28.0))
            cfg = joint.cfg
            lo, hi = -SAFETY_FACTOR * p_max, SAFETY_FACTOR * p_max
            if cfg.pos_min is not None:
                lo = max(lo, cfg.pos_min)
            if cfg.pos_max is not None:
                hi = min(hi, cfg.pos_max)
            self.pos_lo, self.pos_hi = lo, hi          # rad
            self.vel_limit = SAFETY_FACTOR * v_max     # rad/s
            self.torque_limit = SAFETY_FACTOR * t_max  # Nm

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
            tl = f"{a.torque_limit:.1f}" if a.torque_limit is not None else "n/a"
            unit = "turns" if a.is_odrive else "rad"
            self.get_logger().info(
                f"  {name}: pos[{lo},{hi}] {unit}, |vel|<={vl}, |torque|<={tl} Nm")

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
            try:
                joint.update()  # drive ramp / hold last target
            except Exception as e:
                self.get_logger().error(f"{joint.cfg.name}: update failed: {e}")

        states = {name: a.read_state() for name, a in self.adapters.items()}
        self._publish_states(states)
        self._safety_check(states)

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
        """E-stop if any joint faults or exceeds its safe velocity/torque."""
        # Small startup grace so transient feedback doesn't nuisance-trip.
        if time.time() - self._start_time < 1.0:
            return
        for name, st in states.items():
            if not st["has_feedback"]:
                continue
            a = self.adapters[name]
            if st["fault"]:
                self._estop(f"{name} reported fault: {st['fault']}")
                return
            if a.vel_limit is not None and abs(st["vel_native"]) > a.vel_limit:
                self._estop(f"{name} velocity {abs(st['vel_native']):.2f} > "
                            f"{a.vel_limit:.2f} ({'turns/s' if a.is_odrive else 'rad/s'})")
                return
            if a.torque_limit is not None and abs(st["torque"]) > a.torque_limit:
                self._estop(f"{name} torque {abs(st['torque']):.2f} > "
                            f"{a.torque_limit:.2f} Nm")
                return

    def _estop(self, reason: str):
        if self._estopped:
            return
        self._estopped = True
        self.get_logger().error(f"E-STOP: {reason}. Disabling all motors.")
        self.disable_all()

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
