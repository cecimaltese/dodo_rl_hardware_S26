"""
motor_controller.py — unified, config-driven controller for the whole Dodo motor
fleet on ONE CAN bus (can0), behind a single object.

Design principle (load-bearing): every motor — Damiao or ODrive — exposes the
*identical* joint interface (enable/disable, set_zero, get_position/get_state,
send_position, goto, start_move/is_moving/update). MotorController owns them all
and adds the sim-to-real hook: step(action) / get_observation().

CAN topology
------------
Both motor types live on the same physical bus (can0 @ 1 Mbit/s) but get their
OWN filtered SocketCAN socket, so they never mis-ingest each other's frames
(SocketCAN delivers a private copy of every frame to each open socket):

  * Damiao  -> DaMiaoController(..., can_filters=<DM feedback ids + 0x7FF>)
               (DaMiaoController forwards **kwargs to can.interface.Bus)
  * ODrive  -> a second can.Bus(..., can_filters=[per-node masks]) plus a daemon
               reader thread that drains bus.recv() into OdriveCanJoint.process_frame.

Bring the bus up first (OS level, not python-can):
    sudo ip link set can0 up type can bitrate 1000000

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
"""

import math
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Union

import can

from damiao_joint import DamiaoJoint, DamiaoJointConfig
from odrive_can_joint import OdriveCanJoint, OdriveCanJointConfig

try:
    from damiao_motor import DaMiaoController
except Exception:  # allow import / syntax-check without the lib present
    DaMiaoController = None  # type: ignore

TWO_PI = 2.0 * math.pi
DM_REGISTER_ID = 0x7FF  # Damiao register read/write responses

AnyJoint = Union[DamiaoJoint, OdriveCanJoint]


@dataclass
class PolicyJoint:
    """One entry in the policy's joint vector: which physical joint, and its
    default (home) position in RADIANS (sim convention)."""
    name: str                  # must match a DamiaoJointConfig / OdriveCanJointConfig name
    default_pos: float = 0.0   # radians


@dataclass
class MotorControllerConfig:
    dm_configs: List[DamiaoJointConfig] = field(default_factory=list)
    od_configs: List[OdriveCanJointConfig] = field(default_factory=list)
    channel: str = "can0"
    bustype: str = "socketcan"
    # Policy joint order + home pose (radians). Order MUST match the sim.
    policy_joints: List[PolicyJoint] = field(default_factory=list)
    action_scale: float = 0.5  # matches IsaacLab JointPositionAction default scale


class MotorController:
    def __init__(self, cfg: MotorControllerConfig):
        self.cfg = cfg
        self._by_name: Dict[str, AnyJoint] = {}

        # ── Damiao: one controller/socket, filtered to DM feedback ids ──
        self.dm_joints: List[DamiaoJoint] = []
        self.dm_controller = None
        if cfg.dm_configs:
            if DaMiaoController is None:
                raise RuntimeError("damiao_motor not installed but dm_configs given.")
            dm_filters = [{"can_id": c.feedback_id, "can_mask": 0x7FF} for c in cfg.dm_configs]
            dm_filters.append({"can_id": DM_REGISTER_ID, "can_mask": 0x7FF})
            self.dm_controller = DaMiaoController(
                channel=cfg.channel, bustype=cfg.bustype, can_filters=dm_filters,
            )
            for c in cfg.dm_configs:
                j = DamiaoJoint(self.dm_controller, c)
                self.dm_joints.append(j)
                self._by_name[c.name] = j

        # ── ODrive: a second filtered socket + reader thread ──
        self.od_joints: List[OdriveCanJoint] = []
        self.od_bus: Optional[can.BusABC] = None
        self._od_by_node: Dict[int, OdriveCanJoint] = {}
        self._reader_thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        if cfg.od_configs:
            od_filters = [{"can_id": c.node_id << 5, "can_mask": 0x7E0} for c in cfg.od_configs]
            self.od_bus = can.Bus(channel=cfg.channel, interface=cfg.bustype,
                                  can_filters=od_filters)
            for c in cfg.od_configs:
                j = OdriveCanJoint(self.od_bus, c)
                self.od_joints.append(j)
                self._od_by_node[c.node_id] = j
                self._by_name[c.name] = j
            self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
            self._reader_thread.start()

        # Resolve the policy joint order to live joint objects (if provided).
        self.policy_joints: List[PolicyJoint] = list(cfg.policy_joints)
        self._policy_objs: List[AnyJoint] = []
        for pj in self.policy_joints:
            if pj.name not in self._by_name:
                raise KeyError(f"policy_joint '{pj.name}' has no matching motor config")
            self._policy_objs.append(self._by_name[pj.name])

        self._last_action: List[float] = [0.0] * len(self.policy_joints)

    # ── ODrive reader thread ──────────────────────────────────────
    def _reader_loop(self) -> None:
        assert self.od_bus is not None
        while not self._stop.is_set():
            msg = self.od_bus.recv(timeout=0.1)
            if msg is None:
                continue
            node = msg.arbitration_id >> 5
            j = self._od_by_node.get(node)
            if j is not None:
                j.process_frame(msg)

    # ── fleet management ──────────────────────────────────────────
    @property
    def joints(self) -> List[AnyJoint]:
        return [*self.dm_joints, *self.od_joints]

    def enable_all(self) -> None:
        if self.dm_controller is not None:
            self.dm_controller.enable_all()
        for j in self.od_joints:
            j.enable()
        time.sleep(0.1)

    def disable_all(self) -> None:
        for j in self.joints:
            try:
                j.disable()
            except Exception:
                pass

    def set_zero_all(self) -> None:
        """Software-zero every joint at its current shaft position."""
        for j in self.joints:
            j.set_zero()

    def update_all(self) -> None:
        """Tick every joint's non-blocking move (call at the control rate)."""
        for j in self.joints:
            j.update()

    def shutdown(self) -> None:
        self._stop.set()
        if self._reader_thread is not None:
            self._reader_thread.join(timeout=1.0)
        self.disable_all()
        if self.dm_controller is not None:
            try:
                self.dm_controller.shutdown()
            except Exception:
                pass
        if self.od_bus is not None:
            try:
                self.od_bus.shutdown()
            except Exception:
                pass

    # ── unit helpers ──────────────────────────────────────────────
    @staticmethod
    def _is_odrive(j: AnyJoint) -> bool:
        return isinstance(j, OdriveCanJoint)

    def _rad_to_native(self, j: AnyJoint, rad: float) -> float:
        return rad / TWO_PI if self._is_odrive(j) else rad

    def _native_to_rad(self, j: AnyJoint, native: float) -> float:
        return native * TWO_PI if self._is_odrive(j) else native

    # ── sim-to-real hook: step / observe ──────────────────────────
    def step(self, action: Sequence[float]) -> None:
        """Apply one policy action.

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
