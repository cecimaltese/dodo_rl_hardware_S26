"""
dodo_ipc.py — the shared ZMQ message layer for the dodo robot.

This is the SINGLE SOURCE OF TRUTH for how every process in the stack talks:
topic names, endpoints, wire schemas, the canonical joint order, and the
latest-value delivery rules. Both the real robot services (motor_service.py,
imu_service.py) and the simulator (sim_service.py) import this module — that is
what guarantees `run_policy.py --backend zmq` cannot tell sim from robot.

Design (see ZMQ_MIGRATION.md for the full rationale):

  * One endpoint per topic, single-part binary messages. That combination makes
    ZMQ_CONFLATE safe on subscribers, so a control loop always reads the NEWEST
    state instead of a backlog. (CONFLATE is broken with multipart messages and
    with multi-topic sockets — do not "optimize" this into one socket.)
  * The topic's single PRODUCER binds; every consumer connects:
        motor_states    bound by motor_service / sim_service
        imu             bound by imu_service   / sim_service
        motor_commands  bound by THE policy runner (or bench demo) — exactly one
                        commander can exist at a time, and debug tools can
                        subscribe to commands like any other topic.
        ctl (REQ/REP)   bound by motor_service / sim_service
  * Wire arrays are ALWAYS length N_JOINTS in JOINT_ORDER; joint names never go
    on the wire. A command of NaN for a joint means "no command for this joint".
  * Every message carries (schema_version, seq, t_mono_ns). t_mono_ns is the
    sender's monotonic clock — meaningful for age checks only on the same host.
    seq gaps let tools count drops.

Environment overrides (all optional):
    DODO_ZMQ_HOST                    default host consumers connect to (127.0.0.1)
    DODO_ZMQ_HOST_<TOPIC>            per-topic host override (e.g. ..._MOTOR_COMMANDS)
    DODO_ZMQ_PORT_<TOPIC>            per-topic port override

Compatible with Python 3.10 (robot Ubuntu box) and 3.14 (dev Macs).
Dependencies: pyzmq only.
"""

from __future__ import annotations

import json
import math
import os
import struct
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import zmq

# ---------------------------------------------------------------------------
# Canonical joint order (the wire order)
# ---------------------------------------------------------------------------
# Identical to rl_env.POLICY_JOINT_ORDER (IsaacLab/PhysX articulation order,
# type-grouped, left before right) — empirically pinned by
# test_sim2sim_consistency.py. If this ever changes, bump SCHEMA_VERSION.
JOINT_ORDER: List[str] = [
    "hip_left", "hip_right",
    "upper_leg_left", "upper_leg_right",
    "lower_leg_left", "lower_leg_right",
    "foot_left", "foot_right",
]
N_JOINTS = len(JOINT_ORDER)
JOINT_INDEX: Dict[str, int] = {name: i for i, name in enumerate(JOINT_ORDER)}

SCHEMA_VERSION = 1

# "No command for this joint" marker in motor_commands.target_pos.
NO_COMMAND = float("nan")


def no_command(value: float) -> bool:
    """True if a command slot is the NaN 'leave this joint alone' marker."""
    return math.isnan(value)


# ---------------------------------------------------------------------------
# Topics / endpoints
# ---------------------------------------------------------------------------
TOPIC_CTL = "ctl"
TOPIC_MOTOR_STATES = "motor_states"
TOPIC_MOTOR_COMMANDS = "motor_commands"
TOPIC_IMU = "imu"
TOPIC_TELEMETRY = "telemetry"          # reserved (recorder / diagnostics)

DEFAULT_PORTS: Dict[str, int] = {
    TOPIC_CTL: 6000,
    TOPIC_MOTOR_STATES: 6001,
    TOPIC_MOTOR_COMMANDS: 6002,
    TOPIC_IMU: 6003,
    TOPIC_TELEMETRY: 6004,
}


def topic_port(topic: str) -> int:
    """Port for a topic, honoring DODO_ZMQ_PORT_<TOPIC> (resolved at call time
    so tests can override before opening sockets)."""
    env = os.environ.get(f"DODO_ZMQ_PORT_{topic.upper()}")
    return int(env) if env else DEFAULT_PORTS[topic]


def topic_host(topic: str) -> str:
    """Host consumers connect to for a topic (producer side always binds *)."""
    return (os.environ.get(f"DODO_ZMQ_HOST_{topic.upper()}")
            or os.environ.get("DODO_ZMQ_HOST")
            or "127.0.0.1")


def bind_endpoint(topic: str) -> str:
    return f"tcp://*:{topic_port(topic)}"


def connect_endpoint(topic: str) -> str:
    return f"tcp://{topic_host(topic)}:{topic_port(topic)}"


def now_ns() -> int:
    """Sender timestamp for message headers: monotonic, ns."""
    return time.monotonic_ns()


# ---------------------------------------------------------------------------
# Wire schemas
# ---------------------------------------------------------------------------
# Common header on every PUB/SUB message.
_HDR = struct.Struct("<HHIq")           # schema_version u16, msg_type u16, seq u32, t_mono_ns i64
MSG_MOTOR_STATES = 1
MSG_MOTOR_COMMANDS = 2
MSG_IMU = 3

# Per-joint flag bits in MotorStates.joint_flags.
JF_HAS_FEEDBACK = 1 << 0    # we have ever heard from this motor
JF_ENABLED = 1 << 1         # motor is enabled (closed loop)
JF_IDLED = 1 << 2           # safety layer sent this ONE motor to IDLE
JF_CALIBRATED = 1 << 3      # zero offset is valid (ODrive: only after homing/set_zero)
JF_FAULT = 1 << 4           # motor reports a fault

# Global flag bits in MotorStates.global_flags.
GF_ESTOPPED = 1 << 0        # e-stop latched: all motors disabled, commands ignored
GF_ALL_CALIBRATED = 1 << 1  # every configured joint is calibrated
GF_ENABLED = 1 << 2         # fleet has been enabled via ctl

# Flag bits in ImuSample.flags.
IMU_QUAT_VALID = 1 << 0     # orientation quat is fused/valid
IMU_LIN_VEL_VALID = 1 << 1  # base_lin_vel present — SIM ONLY (hardware never sets it)


@dataclass
class MotorStates:
    """Joint feedback in JOINT_ORDER: radians, rad/s, Nm (unit conversion from
    each motor's native units happens in the service, never on the wire)."""
    pos: List[float]                    # [N_JOINTS] rad
    vel: List[float]                    # [N_JOINTS] rad/s
    torque: List[float]                 # [N_JOINTS] Nm
    joint_flags: List[int]              # [N_JOINTS] JF_* bitmasks
    global_flags: int = 0               # GF_* bitmask
    seq: int = 0
    t_mono_ns: int = 0

    _PAYLOAD = struct.Struct(f"<{3 * N_JOINTS}f{N_JOINTS}BB")

    def pack(self, seq: int, t_mono_ns: Optional[int] = None) -> bytes:
        t = now_ns() if t_mono_ns is None else t_mono_ns
        return (_HDR.pack(SCHEMA_VERSION, MSG_MOTOR_STATES, seq & 0xFFFFFFFF, t)
                + self._PAYLOAD.pack(*self.pos, *self.vel, *self.torque,
                                     *self.joint_flags, self.global_flags))

    @classmethod
    def unpack(cls, payload: bytes, seq: int, t_mono_ns: int) -> "MotorStates":
        v = cls._PAYLOAD.unpack(payload)
        n = N_JOINTS
        return cls(pos=list(v[0:n]), vel=list(v[n:2 * n]), torque=list(v[2 * n:3 * n]),
                   joint_flags=list(v[3 * n:4 * n]), global_flags=v[4 * n],
                   seq=seq, t_mono_ns=t_mono_ns)

    # convenience
    def flag(self, joint: int, bit: int) -> bool:
        return bool(self.joint_flags[joint] & bit)

    @property
    def estopped(self) -> bool:
        return bool(self.global_flags & GF_ESTOPPED)

    @property
    def all_calibrated(self) -> bool:
        return bool(self.global_flags & GF_ALL_CALIBRATED)

    @property
    def enabled(self) -> bool:
        return bool(self.global_flags & GF_ENABLED)


@dataclass
class MotorCommands:
    """Absolute joint position targets [rad] in JOINT_ORDER. NaN = no command
    for that joint (see NO_COMMAND). The commander resolves all policy details
    (default pose offset, action scaling) BEFORE publishing — services must not
    know anything about the policy."""
    target_pos: List[float]             # [N_JOINTS] rad, NaN allowed
    seq: int = 0
    t_mono_ns: int = 0

    _PAYLOAD = struct.Struct(f"<{N_JOINTS}f")

    def pack(self, seq: int, t_mono_ns: Optional[int] = None) -> bytes:
        t = now_ns() if t_mono_ns is None else t_mono_ns
        return (_HDR.pack(SCHEMA_VERSION, MSG_MOTOR_COMMANDS, seq & 0xFFFFFFFF, t)
                + self._PAYLOAD.pack(*self.target_pos))

    @classmethod
    def unpack(cls, payload: bytes, seq: int, t_mono_ns: int) -> "MotorCommands":
        return cls(target_pos=list(cls._PAYLOAD.unpack(payload)),
                   seq=seq, t_mono_ns=t_mono_ns)


@dataclass
class ImuSample:
    """Body-frame IMU reading. quat is (w,x,y,z), valid only with IMU_QUAT_VALID.
    lin_vel is the sim-only ground-truth base linear velocity (IMU_LIN_VEL_VALID);
    real hardware never sets it — policies that need it are sim-verification only."""
    gyro: Tuple[float, float, float]            # rad/s
    accel: Tuple[float, float, float]           # m/s^2
    quat: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    lin_vel: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    flags: int = 0
    seq: int = 0
    t_mono_ns: int = 0

    _PAYLOAD = struct.Struct("<13fB")

    def pack(self, seq: int, t_mono_ns: Optional[int] = None) -> bytes:
        t = now_ns() if t_mono_ns is None else t_mono_ns
        return (_HDR.pack(SCHEMA_VERSION, MSG_IMU, seq & 0xFFFFFFFF, t)
                + self._PAYLOAD.pack(*self.gyro, *self.accel, *self.quat,
                                     *self.lin_vel, self.flags))

    @classmethod
    def unpack(cls, payload: bytes, seq: int, t_mono_ns: int) -> "ImuSample":
        v = cls._PAYLOAD.unpack(payload)
        return cls(gyro=v[0:3], accel=v[3:6], quat=v[6:10], lin_vel=v[10:13],
                   flags=v[13], seq=seq, t_mono_ns=t_mono_ns)

    @property
    def quat_valid(self) -> bool:
        return bool(self.flags & IMU_QUAT_VALID)

    @property
    def lin_vel_valid(self) -> bool:
        return bool(self.flags & IMU_LIN_VEL_VALID)


_MSG_CLASSES = {
    MSG_MOTOR_STATES: MotorStates,
    MSG_MOTOR_COMMANDS: MotorCommands,
    MSG_IMU: ImuSample,
}


def decode(frame: bytes):
    """Decode any wire frame into its message dataclass. Raises ValueError on
    schema mismatch — a stale process must fail loudly, not scramble legs."""
    if len(frame) < _HDR.size:
        raise ValueError(f"frame too short ({len(frame)} B)")
    version, msg_type, seq, t_mono_ns = _HDR.unpack_from(frame)
    if version != SCHEMA_VERSION:
        raise ValueError(f"schema_version {version} != {SCHEMA_VERSION} — "
                         f"update/restart the older process")
    cls = _MSG_CLASSES.get(msg_type)
    if cls is None:
        raise ValueError(f"unknown msg_type {msg_type}")
    return cls.unpack(frame[_HDR.size:], seq=seq, t_mono_ns=t_mono_ns)


def age_s(msg, now_mono_ns: Optional[int] = None) -> float:
    """Age of a message in seconds. Same-host only (monotonic clocks are not
    comparable across machines); cross-host consumers should track arrival
    times instead."""
    now = time.monotonic_ns() if now_mono_ns is None else now_mono_ns
    return (now - msg.t_mono_ns) / 1e9


# ---------------------------------------------------------------------------
# Sockets
# ---------------------------------------------------------------------------
def _ctx() -> zmq.Context:
    return zmq.Context.instance()


class Publisher:
    """PUB socket for one topic. The producer binds (see module docstring)."""

    def __init__(self, topic: str):
        self.topic = topic
        self._seq = 0
        self.sock = _ctx().socket(zmq.PUB)
        self.sock.setsockopt(zmq.LINGER, 0)
        self.sock.setsockopt(zmq.SNDHWM, 100)
        self.sock.bind(bind_endpoint(topic))

    def send(self, msg, t_mono_ns: Optional[int] = None) -> int:
        """Publish a message (assigns seq; stamps 'now' unless t_mono_ns is
        given — pass the original sample time when a message was intentionally
        delayed, e.g. sim_service latency injection). Returns the seq used."""
        seq = self._seq
        self.sock.send(msg.pack(seq=seq, t_mono_ns=t_mono_ns))
        self._seq = (seq + 1) & 0xFFFFFFFF
        return seq

    def close(self) -> None:
        self.sock.close(0)


class Subscriber:
    """SUB socket for one topic; always connects.

    latest-value mode (default, conflate=True): the kernel keeps only the newest
    message — right for control loops. Monitoring tools that must see EVERY
    message (rate/drop counting) pass conflate=False.
    """

    def __init__(self, topic: str, conflate: bool = True):
        self.topic = topic
        self.conflate = conflate
        self.sock = _ctx().socket(zmq.SUB)
        self.sock.setsockopt(zmq.LINGER, 0)
        if conflate:
            self.sock.setsockopt(zmq.CONFLATE, 1)   # must precede connect
        self.sock.setsockopt(zmq.SUBSCRIBE, b"")
        self.sock.connect(connect_endpoint(topic))
        self._poller = zmq.Poller()
        self._poller.register(self.sock, zmq.POLLIN)

    def latest(self):
        """Newest message since the previous call, or None if nothing new.
        Non-blocking. Drains the queue even without CONFLATE (belt & braces)."""
        frame = None
        while True:
            try:
                frame = self.sock.recv(zmq.NOBLOCK)
            except zmq.Again:
                break
        return decode(frame) if frame is not None else None

    def recv(self, timeout_s: Optional[float] = None):
        """Blocking receive of the next message (newest with CONFLATE);
        None on timeout."""
        timeout_ms = None if timeout_s is None else max(0, int(timeout_s * 1000))
        if not self._poller.poll(timeout_ms):
            return None
        return decode(self.sock.recv())

    def close(self) -> None:
        self.sock.close(0)


# ---------------------------------------------------------------------------
# ctl channel (REQ/REP, JSON) — rare human/runner-triggered operations
# ---------------------------------------------------------------------------
# Request : {"cmd": "<name>", ...args}
# Reply   : {"ok": true, ...data}   or   {"ok": false, "error": "..."}
class CtlServer:
    """REP socket owned by the service. Call poll_once(handlers) inside the
    service's main loop; it never blocks."""

    def __init__(self):
        self.sock = _ctx().socket(zmq.REP)
        self.sock.setsockopt(zmq.LINGER, 0)
        self.sock.bind(bind_endpoint(TOPIC_CTL))
        self._poller = zmq.Poller()
        self._poller.register(self.sock, zmq.POLLIN)

    def poll_once(self, handlers: Dict[str, Callable[[dict], dict]]) -> bool:
        """Serve at most one pending request. Returns True if one was served.
        handlers: cmd name -> fn(args_dict) -> reply_dict (merged with ok:true).
        Handler exceptions become {"ok": false, "error": ...} replies."""
        if not self._poller.poll(0):
            return False
        try:
            req = json.loads(self.sock.recv())
        except Exception as e:
            self.sock.send(json.dumps({"ok": False, "error": f"bad request: {e}"}).encode())
            return True
        cmd = req.get("cmd")
        fn = handlers.get(cmd)
        if fn is None:
            reply = {"ok": False, "error": f"unknown cmd '{cmd}'",
                     "known": sorted(handlers)}
        else:
            try:
                reply = {"ok": True, **(fn(req) or {})}
            except Exception as e:
                reply = {"ok": False, "error": f"{type(e).__name__}: {e}"}
        self.sock.send(json.dumps(reply).encode())
        return True

    def close(self) -> None:
        self.sock.close(0)


class CtlClient:
    """REQ client with timeout + automatic socket rebuild (a timed-out REQ
    socket is stuck in its state machine and must be recreated)."""

    def __init__(self):
        self._open()

    def _open(self) -> None:
        self.sock = _ctx().socket(zmq.REQ)
        self.sock.setsockopt(zmq.LINGER, 0)
        self.sock.connect(connect_endpoint(TOPIC_CTL))
        self._poller = zmq.Poller()
        self._poller.register(self.sock, zmq.POLLIN)

    def request(self, cmd: str, timeout_s: float = 2.0, **args) -> dict:
        """Send one ctl command, return the reply dict. Raises TimeoutError if
        the service does not answer (and rebuilds the socket so the client
        remains usable)."""
        self.sock.send(json.dumps({"cmd": cmd, **args}).encode())
        if not self._poller.poll(int(timeout_s * 1000)):
            self.sock.close(0)
            self._open()
            raise TimeoutError(f"ctl '{cmd}': no reply in {timeout_s}s — is the "
                               f"service running on {connect_endpoint(TOPIC_CTL)}?")
        return json.loads(self.sock.recv())

    def close(self) -> None:
        self.sock.close(0)


# ---------------------------------------------------------------------------
# Real-time loop helper
# ---------------------------------------------------------------------------
class RateLoop:
    """Fixed-rate loop with ABSOLUTE deadlines on the monotonic clock (no drift
    accumulation) and overrun accounting. Usage:

        loop = RateLoop(200.0, name="motor")
        while running:
            ...work...
            loop.sleep()

    If an iteration overruns its deadline, the schedule resets to 'now' (we do
    not try to catch up with a burst — bursty control is worse than late
    control) and the overrun is counted.
    """

    def __init__(self, rate_hz: float, name: str = ""):
        if rate_hz <= 0:
            raise ValueError("rate_hz must be > 0")
        self.period_s = 1.0 / rate_hz
        self.name = name
        self.ticks = 0
        self.overruns = 0
        self.worst_late_s = 0.0
        self._next_t: Optional[float] = None

    def sleep(self) -> None:
        now = time.monotonic()
        if self._next_t is None:
            self._next_t = now + self.period_s
        deadline = self._next_t
        self.ticks += 1
        if now > deadline:
            late = now - deadline
            self.overruns += 1
            self.worst_late_s = max(self.worst_late_s, late)
            self._next_t = now + self.period_s      # reset schedule, don't burst
            return
        time.sleep(deadline - now)
        self._next_t = deadline + self.period_s

    def stats(self) -> dict:
        return {"name": self.name, "ticks": self.ticks, "overruns": self.overruns,
                "worst_late_ms": round(self.worst_late_s * 1e3, 3)}

    def stats_str(self) -> str:
        s = self.stats()
        pct = 100.0 * s["overruns"] / max(1, s["ticks"])
        return (f"[{s['name'] or 'loop'}] {s['ticks']} ticks, "
                f"{s['overruns']} overruns ({pct:.2f}%), "
                f"worst {s['worst_late_ms']:.2f} ms late")
