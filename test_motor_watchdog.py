"""
test_motor_watchdog.py — a joint that stops talking must stop the robot.

Both drivers cache their last frame forever (damiao_motor never clears its state;
OdriveCanJoint keeps its last encoder frame), and the library's Damiao polling thread
exits silently on any bus error. Until 2026-11-04 motor_service's `has_feedback`
meant "has ever reported", so FB_WATCHDOG_S could never fire: a dead joint or bus
looked alive with frozen numbers. These tests pin the fix.

No hardware: the REAL MotorService, drivers and damiao_motor library run on
python-can's in-process `virtual` bus, against FakeFleet, which answers like the
motors do (Damiao: replies to enable / MIT / register frames only; ODrive: cyclic
encoder + torque + heartbeat frames every 10 ms, RTR answers, Set_Axis_State) and
records what it is told (MIT frames, Set_Input_Pos, gains).

The second half pins the calibration wiring (2026-11-05): the service talks JOINT
angles on the wire and each motor's own raw units on the bus, through a test
calibration with known signs and zeros (TEST_CAL), not the robot's joint_calib.py.
"""
from __future__ import annotations

import socket
import struct
import threading
import time
import uuid

import math

import can
import pytest
from damiao_motor.core.motor import float_to_uint, uint_to_float

import calibrate_joints
import dodo_ipc as ipc
import fleet_config
import motor_service as ms
import real2sim
from fleet_config import DM_JOINTS, FEEDBACK_FRESH_S, OD_JOINTS
from odrive_can_joint import (AXIS_STATE_CLOSED_LOOP_CONTROL, AXIS_STATE_IDLE,
                              CMD_GET_ENCODER_ESTIMATES,
                              CMD_GET_TORQUES, CMD_HEARTBEAT, CMD_SET_AXIS_STATE,
                              CMD_SET_INPUT_POS, CMD_SET_POS_GAIN, CMD_SET_VEL_GAINS)

pytestmark = pytest.mark.filterwarnings("ignore::DeprecationWarning")  # damiao: bustype=

DM_P_MAX = 12.5          # the 4340P preset's position range (damiao_motor)
DM_ENABLE = bytes([0xFF] * 7 + [0xFC])
DM_DISABLE = bytes([0xFF] * 7 + [0xFD])
CTRL_MODE_MIT = 1


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@pytest.fixture(autouse=True)
def isolated_ports(monkeypatch):
    for topic in ipc.DEFAULT_PORTS:
        monkeypatch.setenv(f"DODO_ZMQ_PORT_{topic.upper()}", str(_free_port()))


# The service's calibration in these tests: (sign, zero [raw], measured stops [rad]).
# Mixed signs on both motor types, so a missing or doubled sign shows.
TEST_CAL = {
    "hip_left":        (+1, 0.10, (-0.6, 0.6)),
    "hip_right":       (+1, -0.05, (-0.6, 0.6)),
    "upper_leg_left":  (+1, 0.0, (-0.9, 2.3)),
    "upper_leg_right": (-1, 1.0, (-1.0, 2.3)),
    "lower_leg_left":  (-1, 0.0, (-2.7, 1.6)),
    "lower_leg_right": (+1, 0.0, (-2.9, 1.6)),
    "foot_left":       (-1, 0.0, (-1.3, 1.5)),
    "foot_right":      (+1, 0.0, (-1.3, 1.5)),
}


def write_test_calib(path) -> None:
    lines = ["JOINT_CALIB = {"]
    for j, (sign, zero, (lo, hi)) in TEST_CAL.items():
        ratio = fleet_config.GEAR_RATIO.get(j) or 1
        lines.append(f"    {j!r}: dict(motor={j!r}, sign={sign:+d}, ratio={ratio}, "
                     f"sim_lo={lo}, sim_hi={hi}, zero={zero}, raw_lo=None, raw_hi=None),")
    lines.append("}")
    path.write_text("\n".join(lines) + "\n")


class FakeFleet:
    """The 8 motors, answering on two virtual buses. Set dm_alive[motor_id] or
    od_alive[node_id] False to make one go silent."""

    def __init__(self):
        tag = uuid.uuid4().hex[:8]
        self.dm_channel, self.od_channel = f"dm_{tag}", f"od_{tag}"
        self.dm_bus = can.Bus(interface="virtual", channel=self.dm_channel)
        self.od_bus = can.Bus(interface="virtual", channel=self.od_channel)
        self.dm_cfg = {c.motor_id: c for c in DM_JOINTS}
        self.dm_alive = {i: True for i in self.dm_cfg}
        self.dm_enabled = {i: False for i in self.dm_cfg}
        self.od_alive = {c.node_id: True for c in OD_JOINTS}
        self.od_state = {c.node_id: AXIS_STATE_IDLE for c in OD_JOINTS}
        # Raw readings (Damiao rad at its zero = TEST_CAL's, so every joint reads 0;
        # ODrive turns-since-boot, arbitrary) and what each motor was last told.
        self.dm_pos = {c.motor_id: TEST_CAL[c.name][1] for c in DM_JOINTS}
        self.od_pos = {c.node_id: 2.5 + 0.1 * c.node_id for c in OD_JOINTS}
        self.od_torque = {c.node_id: 0.0 for c in OD_JOINTS}     # motor-side Nm
        # Drive calibration: like the real drives, closed loop is refused (IDLE,
        # procedure_result NOT_CALIBRATED) until FULL_CALIBRATION_SEQUENCE ran. True by
        # default so the other tests start from calibrated drives.
        self.od_drive_cal = {c.node_id: True for c in OD_JOINTS}
        self.od_proc = {c.node_id: 0 for c in OD_JOINTS}
        self.od_cal_until = {}
        # A motor with an open phase (node 3 on the robot, 2026-11-05): its calibration
        # fails PHASE_RESISTANCE_OUT_OF_RANGE and latches CALIBRATION_ERROR until
        # Clear_Errors.
        self.od_phase_open = {c.node_id: False for c in OD_JOINTS}
        self.od_err = {c.node_id: 0 for c in OD_JOINTS}
        self.dm_cmd = {}        # motor_id -> (pos, kp, kd) of the last MIT frame
        self.od_input_pos = {}  # node -> turns of the last Set_Input_Pos
        self.od_gains = {}      # node -> [pos_gain, vel_gain, vel_integrator_gain]
        self.dm_id = {c.name: c.motor_id for c in DM_JOINTS}
        self.od_id = {c.name: c.node_id for c in OD_JOINTS}
        self._run = True
        self._threads = [threading.Thread(target=f, daemon=True)
                         for f in (self._damiao_loop, self._odrive_loop)]
        for t in self._threads:
            t.start()

    # ── Damiao: speaks only when spoken to ──────────────────────────
    def _dm_feedback(self, mid: int) -> None:
        status = 1 if self.dm_enabled[mid] else 0
        pos_u = float_to_uint(self.dm_pos[mid], -DM_P_MAX, DM_P_MAX, 16)
        vel_u, tq_u = 0x7FF, 0x7FF                         # ~0 rad/s, 0 Nm
        data = bytes([(status << 4) | mid, pos_u >> 8, pos_u & 0xFF, vel_u >> 4,
                      ((vel_u & 0xF) << 4) | (tq_u >> 8), tq_u & 0xFF, 30, 30])
        self.dm_bus.send(can.Message(arbitration_id=self.dm_cfg[mid].feedback_id,
                                     data=data, is_extended_id=False))

    def _damiao_loop(self):
        while self._run:
            msg = self.dm_bus.recv(timeout=0.01)
            if msg is None or len(msg.data) != 8:
                continue
            d = bytes(msg.data)
            if msg.arbitration_id == 0x7FF and d[2] == 0x33:          # register read
                mid = d[0] | (d[1] << 8)
                if self.dm_alive.get(mid):
                    val = CTRL_MODE_MIT if d[3] == 10 else 0
                    self.dm_bus.send(can.Message(
                        arbitration_id=self.dm_cfg[mid].feedback_id, is_extended_id=False,
                        data=bytes([d[0], d[1], 0x33, d[3]]) + struct.pack("<I", val)))
            elif msg.arbitration_id in self.dm_cfg and self.dm_alive[msg.arbitration_id]:
                mid = msg.arbitration_id
                if d == DM_ENABLE:
                    self.dm_enabled[mid] = True
                elif d == DM_DISABLE:
                    self.dm_enabled[mid] = False
                elif d[:7] != bytes([0xFF] * 7):       # an MIT command frame
                    self.dm_cmd[mid] = (
                        uint_to_float((d[0] << 8) | d[1], -DM_P_MAX, DM_P_MAX, 16),
                        uint_to_float(((d[3] & 0xF) << 8) | d[4], 0.0, 500.0, 12),
                        uint_to_float((d[5] << 4) | (d[6] >> 4), 0.0, 5.0, 12))
                self._dm_feedback(mid)                 # every frame gets a reply

    # ── ODrive: streams cyclic feedback, enabled or not ─────────────
    def _od_send(self, node: int, cmd: int, data: bytes) -> None:
        self.od_bus.send(can.Message(arbitration_id=(node << 5) | cmd, data=data,
                                     is_extended_id=False))

    def _odrive_loop(self):
        next_cyclic = 0.0
        while self._run:
            msg = self.od_bus.recv(timeout=0.002)
            if msg is not None:
                node, cmd = msg.arbitration_id >> 5, msg.arbitration_id & 0x1F
                if self.od_alive.get(node):
                    d = bytes(msg.data)
                    if cmd == CMD_SET_AXIS_STATE:
                        want = struct.unpack("<I", d[:4])[0]
                        if want == AXIS_STATE_CLOSED_LOOP_CONTROL and not self.od_drive_cal[node]:
                            self.od_state[node], self.od_proc[node] = AXIS_STATE_IDLE, 14
                        elif want == 3:                       # FULL_CALIBRATION_SEQUENCE
                            self.od_state[node] = 4           # MOTOR_CALIBRATION first
                            self.od_cal_until[node] = time.monotonic() + 0.3
                        elif not (want == AXIS_STATE_IDLE
                                  and self.od_state[node] == AXIS_STATE_IDLE):
                            # (IDLE while IDLE leaves the last result: the real drives
                            # kept NOT_CALIBRATED through it, 2026-11-05)
                            self.od_state[node], self.od_proc[node] = want, 0
                    elif cmd == 0x18:                         # Clear_Errors
                        self.od_err[node] = 0
                    elif cmd == CMD_SET_INPUT_POS:
                        self.od_input_pos[node] = struct.unpack("<fhh", d)[0]
                    elif cmd == CMD_SET_POS_GAIN:
                        self.od_gains.setdefault(node, [None] * 3)[0] = \
                            struct.unpack("<f", d[:4])[0]
                    elif cmd == CMD_SET_VEL_GAINS:
                        self.od_gains.setdefault(node, [None] * 3)[1:] = \
                            struct.unpack("<ff", d)
                    elif cmd == CMD_GET_ENCODER_ESTIMATES and msg.is_remote_frame:
                        self._od_send(node, CMD_GET_ENCODER_ESTIMATES,
                                      struct.pack("<ff", self.od_pos[node], 0.0))
            for node, until in list(self.od_cal_until.items()):
                if time.monotonic() >= until:                 # calibration finished
                    del self.od_cal_until[node]
                    if self.od_phase_open[node]:
                        self.od_state[node], self.od_proc[node] = AXIS_STATE_IDLE, 6
                        self.od_err[node] = 0x40000000
                    else:
                        self.od_state[node], self.od_proc[node] = AXIS_STATE_IDLE, 0
                        self.od_drive_cal[node] = True
            if time.monotonic() >= next_cyclic:
                next_cyclic = time.monotonic() + 0.010
                for node, alive in self.od_alive.items():
                    if alive:
                        self._od_send(node, CMD_GET_ENCODER_ESTIMATES,
                                      struct.pack("<ff", self.od_pos[node], 0.0))
                        self._od_send(node, CMD_GET_TORQUES,
                                      struct.pack("<ff", 0.0, self.od_torque[node]))
                        self._od_send(node, CMD_HEARTBEAT,
                                      struct.pack("<IBBBB", self.od_err[node],
                                                  self.od_state[node],
                                                  self.od_proc[node], 0, 0))

    def close(self):
        self._run = False
        for t in self._threads:
            t.join(timeout=1.0)
        self.dm_bus.shutdown()
        self.od_bus.shutdown()


@pytest.fixture
def fleet():
    f = FakeFleet()
    yield f
    f.close()


@pytest.fixture
def service(fleet, tmp_path):
    """The real MotorService on the fake fleet (TEST_CAL), its main loop in a thread."""
    calib = tmp_path / "joint_calib.py"
    write_test_calib(calib)
    svc = ms.MotorService(dm_channel=fleet.dm_channel, od_channel=fleet.od_channel,
                          bustype="virtual", calib_path=calib)
    th = threading.Thread(target=svc.run, daemon=True)
    th.start()
    ctl = ipc.CtlClient()
    yield svc, ctl
    svc._stop.set()
    th.join(timeout=3.0)
    ctl.close()


def status(ctl) -> dict:
    rep = ctl.request("status")
    assert rep["ok"], rep
    return rep


def wait_for(pred, timeout_s: float, what: str):
    t_end = time.monotonic() + timeout_s
    while time.monotonic() < t_end:
        out = pred()
        if out:
            return out
        time.sleep(0.05)
    raise AssertionError(f"timed out after {timeout_s}s waiting for {what}")


# Long enough to clear STARTUP_GRACE_S and then the watchdog with margin.
SETTLED_S = ms.STARTUP_GRACE_S + ms.FB_WATCHDOG_S + 0.5


def test_disabled_damiaos_are_silent_by_design(service):
    """Disabled, a Damiao answers nothing: it must read as 'no feedback' (honest)
    without e-stopping. ODrives stream regardless, so they read fresh."""
    _, ctl = service
    time.sleep(SETTLED_S)
    st = status(ctl)
    assert not st["estopped"], st["estop_reason"]
    for name, j in st["joints"].items():
        assert j["has_feedback"] == (j["type"] == "odrive"), (name, j)


def test_silent_odrive_estops_and_voids_its_calibration(service, fleet):
    _, ctl = service
    time.sleep(ms.STARTUP_GRACE_S)
    rep = ctl.request("calibrate", pose="straight")
    assert rep["ok"], rep
    assert status(ctl)["all_calibrated"]

    fleet.od_alive[7] = False                        # lower_leg_right's drive goes quiet
    st = wait_for(lambda: (s := status(ctl))["estopped"] and s,
                  ms.FB_WATCHDOG_S + 1.0, "e-stop on the silent ODrive")
    assert "lower_leg_right" in st["estop_reason"]
    assert not st["joints"]["lower_leg_right"]["has_feedback"]
    assert not st["joints"]["lower_leg_right"]["calibrated"], \
        "a rebooted ODrive re-zeroes its encoder; the old mapping must not survive"
    assert st["joints"]["lower_leg_left"]["calibrated"]       # the others keep theirs
    assert not st["all_calibrated"]


def test_estop_reset_does_not_revive_a_dead_joint(service, fleet):
    _, ctl = service
    time.sleep(ms.STARTUP_GRACE_S)
    fleet.od_alive[3] = False
    wait_for(lambda: status(ctl)["estopped"], ms.FB_WATCHDOG_S + 1.0, "first e-stop")
    assert ctl.request("estop_reset")["ok"]
    assert not status(ctl)["estopped"]
    st = wait_for(lambda: (s := status(ctl))["estopped"] and s,
                  ms.STARTUP_GRACE_S + 1.0, "e-stop again once the grace is over")
    assert "lower_leg_left" in st["estop_reason"]


def test_silent_damiao_while_enabled_estops(service, fleet):
    """Enabled, the loop sends every Damiao a frame each tick and each answers.
    One that stops answering (motor, wiring, or a dead library polling thread)
    must stop the fleet instead of being held on frozen numbers."""
    _, ctl = service
    time.sleep(ms.STARTUP_GRACE_S)
    rep = ctl.request("calibrate", pose="straight")
    assert rep["ok"], rep
    rep = ctl.request("enable")
    assert rep["ok"], rep
    time.sleep(ms.FB_WATCHDOG_S + 0.3)
    st = status(ctl)
    assert st["enabled"] and not st["estopped"], st["estop_reason"]
    assert all(j["has_feedback"] for j in st["joints"].values()), st["joints"]

    fleet.dm_alive[0x05] = False                     # hip_right stops answering
    st = wait_for(lambda: (s := status(ctl))["estopped"] and s,
                  ms.FB_WATCHDOG_S + 1.0, "e-stop on the silent Damiao")
    assert "hip_right" in st["estop_reason"]
    assert not st["enabled"]


def test_status_reports_the_held_targets(service, fleet):
    """`target_rad`: None while disabled; after the bumpless enable each joint's held
    setpoint (its measured angle, or the band edge it unparks to); after a command,
    that command (clamped). tools/dodo_startup.py ramps from these, not from the
    measured angles, which sag below the targets under load."""
    _, ctl = service
    time.sleep(ms.STARTUP_GRACE_S)
    assert ctl.request("calibrate", pose="straight")["ok"]
    assert all(j["target_rad"] is None for j in status(ctl)["joints"].values())
    rep = ctl.request("enable")
    assert rep["ok"], rep
    st = status(ctl)
    for name, j in st["joints"].items():
        want = rep["unparking"][name]["to_rad"] if name in rep["unparking"] else j["pos_rad"]
        assert j["target_rad"] == pytest.approx(want, abs=2e-3), (name, j)

    pub = ipc.Publisher(ipc.TOPIC_MOTOR_COMMANDS)
    try:
        time.sleep(0.3)                              # let the SUB connect
        cmd = [ipc.NO_COMMAND] * ipc.N_JOINTS
        cmd[ipc.JOINT_INDEX["hip_left"]] = 0.01
        for _ in range(5):
            pub.send(ipc.MotorCommands(target_pos=cmd))
            time.sleep(0.02)
        st = wait_for(lambda: (s := status(ctl))["joints"]["hip_left"]["target_rad"]
                      == pytest.approx(0.01, abs=1e-4) and s, 1.0, "the commanded target")
    finally:
        pub.close()
    assert ctl.request("disable")["ok"]
    assert all(j["target_rad"] is None for j in status(ctl)["joints"].values())


def test_published_flag_is_fresh_feedback(service, fleet):
    """JF_HAS_FEEDBACK on the wire means 'reporting now', not 'reported once'."""
    _, ctl = service
    sub = ipc.Subscriber(ipc.TOPIC_MOTOR_STATES, conflate=True)
    try:
        i = ipc.JOINT_INDEX["foot_right"]
        m = wait_for(lambda: sub.recv(timeout_s=0.5), 2.0, "motor_states")
        assert m.flag(i, ipc.JF_HAS_FEEDBACK)
        fleet.od_alive[8] = False                    # foot_right
        wait_for(lambda: (m := sub.recv(timeout_s=0.5)) is not None
                 and not m.flag(i, ipc.JF_HAS_FEEDBACK),
                 FEEDBACK_FRESH_S + 1.0, "foot_right's has_feedback to drop")
    finally:
        sub.close()


def test_calibration_tool_sees_a_silent_motor_as_offline(fleet, monkeypatch):
    """calibrate_joints.Fleet used to read a silent motor as a steady 0.0, so
    `online()` always said 8/8 and --check-persistence would have called an
    offline drive 'BOOT-RELATIVE'. It now pokes every motor and returns None for
    one that doesn't answer."""
    monkeypatch.setattr(calibrate_joints, "DM_CHANNEL", fleet.dm_channel)
    monkeypatch.setattr(calibrate_joints, "OD_CHANNEL", fleet.od_channel)
    monkeypatch.setattr(calibrate_joints, "BUSTYPE", "virtual")
    f = calibrate_joints.Fleet()
    try:
        wait_for(lambda: len(f.online()) == 8, 2.0, "all 8 motors online")
        fleet.dm_alive[0x02] = False                 # upper_leg_left
        fleet.od_alive[4] = False                    # foot_left
        time.sleep(FEEDBACK_FRESH_S + 0.1)
        reading = f.read()
        assert reading["upper_leg_left"] is None
        assert reading["foot_left"] is None
        assert sorted(f.online()) == sorted(
            set(reading) - {"upper_leg_left", "foot_left"})
    finally:
        f.close()
        f.dm_ctl.shutdown()


# ── the calibration wiring: joint angles on the wire, raw units on the bus ─────────

def enabled(ctl, fleet, settle_s: float = None) -> dict:
    """Calibrate (ODrives at `straight`) and enable, past the startup grace."""
    time.sleep(ms.STARTUP_GRACE_S if settle_s is None else settle_s)
    rep = ctl.request("calibrate", pose="straight")
    assert rep["ok"], rep
    rep = ctl.request("enable")
    assert rep["ok"], rep
    time.sleep(0.2)
    return status(ctl)


def scale(joint: str) -> float:
    return real2sim.rad_per_native(joint)


def test_enable_refused_until_calibrated(service):
    _, ctl = service
    time.sleep(ms.STARTUP_GRACE_S)
    rep = ctl.request("enable")
    assert not rep["ok"] and "calibrate" in rep["error"], rep
    rep = ctl.request("calibrate")                       # no pose: refused
    assert not rep["ok"] and "pose=" in rep["error"], rep
    rep = ctl.request("calibrate", pose="sitting_somewhere")
    assert not rep["ok"], rep
    assert not status(ctl)["all_calibrated"]


def test_states_are_joint_angles_through_the_calibration(service, fleet):
    """calibrate pose=straight makes every ODrive read 0 where it is; a Damiao reads
    sign * (raw - zero); moving a motor moves its joint by sign x scale."""
    _, ctl = service
    st = enabled(ctl, fleet)
    for name, j in st["joints"].items():
        assert j["pos_rad"] == pytest.approx(0.0, abs=2e-3), (name, j)
    fleet.dm_pos[fleet.dm_id["upper_leg_right"]] = 1.0 + 0.3      # raw +0.3 rad
    fleet.od_pos[fleet.od_id["lower_leg_left"]] += 1.0             # one motor turn
    time.sleep(0.2)
    j = status(ctl)["joints"]
    assert j["upper_leg_right"]["pos_rad"] == pytest.approx(-0.3, abs=2e-3)   # sign -1
    assert j["lower_leg_left"]["pos_rad"] == pytest.approx(
        -scale("lower_leg_left"), abs=1e-4)                          # -2pi / 25.6
    ctl.request("disable")


def test_commands_reach_the_motors_in_their_own_units(service, fleet):
    _, ctl = service
    enabled(ctl, fleet)
    raw_knee0 = fleet.od_pos[fleet.od_id["lower_leg_left"]]   # calibrated as 0 here
    q = {"upper_leg_right": 0.4, "hip_left": -0.2, "lower_leg_left": -1.0,
         "foot_right": 0.5}
    pub = ipc.Publisher(ipc.TOPIC_MOTOR_COMMANDS)
    try:
        time.sleep(0.3)
        cmd = [ipc.NO_COMMAND] * ipc.N_JOINTS
        for name, v in q.items():
            cmd[ipc.JOINT_INDEX[name]] = v
        for _ in range(10):
            pub.send(ipc.MotorCommands(target_pos=cmd))
            time.sleep(0.02)
    finally:
        pub.close()
    time.sleep(0.1)
    res = 25.0 / 65535                                   # Damiao position resolution
    assert fleet.dm_cmd[fleet.dm_id["upper_leg_right"]][0] == pytest.approx(
        1.0 - 0.4, abs=res)                              # zero + sign * q, sign -1
    assert fleet.dm_cmd[fleet.dm_id["hip_left"]][0] == pytest.approx(0.10 - 0.2, abs=res)
    assert fleet.od_input_pos[fleet.od_id["lower_leg_left"]] == pytest.approx(
        raw_knee0 + 1.0 / scale("lower_leg_left"), abs=1e-4)          # sign -1
    raw_ankle0 = fleet.od_pos[fleet.od_id["foot_right"]]
    assert fleet.od_input_pos[fleet.od_id["foot_right"]] == pytest.approx(
        raw_ankle0 + 0.5 / scale("foot_right"), abs=1e-4)
    j = status(ctl)["joints"]
    assert j["upper_leg_right"]["target_rad"] == pytest.approx(0.4)
    ctl.request("disable")


def test_commands_are_clamped_inside_the_measured_stops(service, fleet):
    _, ctl = service
    enabled(ctl, fleet)
    pub = ipc.Publisher(ipc.TOPIC_MOTOR_COMMANDS)
    try:
        time.sleep(0.3)
        cmd = [ipc.NO_COMMAND] * ipc.N_JOINTS
        cmd[ipc.JOINT_INDEX["hip_right"]] = 5.0               # far past its +0.6 stop
        for _ in range(5):
            pub.send(ipc.MotorCommands(target_pos=cmd))
            time.sleep(0.02)
    finally:
        pub.close()
    time.sleep(0.1)
    want = 0.6 - fleet_config.STOP_MARGIN_RAD
    assert status(ctl)["joints"]["hip_right"]["target_rad"] == pytest.approx(want)
    assert fleet.dm_cmd[fleet.dm_id["hip_right"]][0] == pytest.approx(
        -0.05 + want, abs=25.0 / 65535)
    ctl.request("disable")


def test_gains_reach_both_motor_types_and_follow_the_scale(service, fleet):
    svc, ctl = service
    enabled(ctl, fleet)
    kp_hip, kd_hip = fleet_config.JOINT_KP["hip"], fleet_config.JOINT_KD["hip"]
    _, kp, kd = fleet.dm_cmd[fleet.dm_id["hip_left"]]
    assert (kp, kd) == pytest.approx((kp_hip, kd_hip), abs=0.15)   # 12-bit encoding
    want = real2sim.odrive_gains("foot_left", fleet_config.JOINT_KP["foot"],
                                 fleet_config.JOINT_KD["foot"])
    assert fleet.od_gains[fleet.od_id["foot_left"]] == pytest.approx(want, rel=1e-6)

    rep = ctl.request("gains", scale=0.25)
    assert rep["ok"], rep
    time.sleep(0.1)
    _, kp, kd = fleet.dm_cmd[fleet.dm_id["hip_left"]]
    assert (kp, kd) == pytest.approx((kp_hip * 0.25, kd_hip * 0.5), abs=0.15)
    want = real2sim.odrive_gains("foot_left", fleet_config.JOINT_KP["foot"] * 0.25,
                                 fleet_config.JOINT_KD["foot"] * 0.5)
    assert fleet.od_gains[fleet.od_id["foot_left"]] == pytest.approx(want, rel=1e-6)
    assert not ctl.request("gains", scale=1.5)["ok"]          # never above fleet_config
    assert not ctl.request("gains", scale=0.0)["ok"]
    for c in DM_JOINTS:                                      # the shared config untouched
        assert (c.kp, c.kd) == (fleet_config.JOINT_KP[fleet_config.joint_group(c.name)],
                                fleet_config.JOINT_KD[fleet_config.joint_group(c.name)])
    ctl.request("disable")


def test_odrive_torque_is_joint_torque(service, fleet):
    """Get_Torques is MOTOR-side Nm: x gear ratio (and the sign) at the joint, where
    the 0.8 x 9 Nm IDLE threshold lives. Raw, 0.3 Nm could never trip it."""
    _, ctl = service
    enabled(ctl, fleet)
    node = fleet.od_id["lower_leg_left"]
    n = real2sim.gear_ratio("lower_leg_left")
    fleet.od_torque[node] = 0.2                              # 5.1 Nm at the joint
    time.sleep(0.2)
    j = status(ctl)["joints"]["lower_leg_left"]
    assert j["torque_nm"] == pytest.approx(-0.2 * n, abs=1e-3)       # sign -1
    assert not j["idled"]
    fleet.od_torque[node] = 0.3                              # 7.7 Nm > 0.8 x 9
    st = wait_for(lambda: (s := status(ctl))["joints"]["lower_leg_left"]["idled"] and s,
                  1.0, "the knee IDLEd on joint torque")
    assert not st["estopped"] and st["joints"]["lower_leg_right"]["idled"] is False
    ctl.request("disable")



# ── the drives' own calibration (lost at every power-up) ─────────────────────────────

def test_enable_fails_closed_when_an_odrive_refuses_closed_loop(service, fleet):
    """2026-11-05 on the robot: every ODrive answered closed loop with IDLE +
    NOT_CALIBRATED, no error bit, and the service carried on with four limp joints."""
    _, ctl = service
    time.sleep(ms.STARTUP_GRACE_S)
    fleet.od_drive_cal[fleet.od_id["lower_leg_right"]] = False      # just powered up
    assert ctl.request("calibrate", pose="straight")["ok"]
    rep = ctl.request("enable")
    assert not rep["ok"], rep
    assert "lower_leg_right" in rep["error"] and "NOT_CALIBRATED" in rep["error"]
    assert "odrive_calibrate" in rep["error"]
    st = status(ctl)
    assert not st["enabled"]
    assert st["joints"]["lower_leg_right"]["drive"]["procedure_result"] == "NOT_CALIBRATED"
    time.sleep(0.1)
    assert all(v != AXIS_STATE_CLOSED_LOOP_CONTROL for v in fleet.od_state.values()), \
        "the drives that did close the loop must be disabled again"
    assert not any(fleet.dm_enabled.values())


def test_odrive_calibrate_then_enable(service, fleet):
    _, ctl = service
    time.sleep(ms.STARTUP_GRACE_S)
    for node in fleet.od_drive_cal:
        fleet.od_drive_cal[node] = False
    assert ctl.request("calibrate", pose="straight")["ok"]
    rep = ctl.request("odrive_calibrate")
    assert rep["ok"] and len(rep["started"]) == 4, rep
    time.sleep(0.1)                                      # the drives are calibrating
    st = status(ctl)
    assert not st["all_calibrated"], "the rotor moved: the joint zeros are void"
    rep = ctl.request("calibrate", pose="straight")
    assert not rep["ok"] and "busy" in rep["error"], "no joint zero mid-motion"
    assert not ctl.request("enable")["ok"]
    time.sleep(0.4)
    st = wait_for(lambda: (s := status(ctl)) and all(
        j["drive"]["state"] == "IDLE" and j["drive"]["procedure_result"] == "SUCCESS"
        for j in s["joints"].values() if j["type"] == "odrive") and s, 2.0,
        "the drives' calibration to finish")
    assert not ctl.request("enable")["ok"]               # joint zeros still void
    assert ctl.request("calibrate", pose="straight")["ok"]
    rep = ctl.request("enable")
    assert rep["ok"], rep
    ctl.request("disable")


def test_odrive_calibrate_refused_while_enabled(service, fleet):
    _, ctl = service
    enabled(ctl, fleet)
    rep = ctl.request("odrive_calibrate")
    assert not rep["ok"] and "disable first" in rep["error"]
    rep = ctl.request("disable")
    rep = ctl.request("odrive_calibrate", joints=["hip_left"])
    assert not rep["ok"] and "not ODrive" in rep["error"]


def test_an_odrive_leaving_closed_loop_estops(service, fleet):
    _, ctl = service
    enabled(ctl, fleet)
    fleet.od_state[fleet.od_id["foot_right"]] = AXIS_STATE_IDLE      # disarmed itself
    st = wait_for(lambda: (s := status(ctl))["estopped"] and s, 1.0,
                  "e-stop on the drive that dropped out")
    assert "foot_right left closed loop" in st["estop_reason"]



def drives_done(ctl):
    return wait_for(lambda: (s := status(ctl)) and all(
        j["drive"]["state"] == "IDLE" and j["drive"]["procedure_result"] != "SUCCESS"
        or j["drive"]["state"] == "IDLE" and j["drive"]["procedure_result"] == "SUCCESS"
        for j in s["joints"].values() if j["type"] == "odrive") and s, 2.0,
        "every drive back in IDLE")


def test_a_dead_motor_doesnt_estop_or_cancel_the_others(service, fleet):
    """2026-11-05 on the robot: the left knee's calibration failed (open phase), its
    latched error e-stopped the disabled service, and the e-stop's IDLE cancelled the
    other drives' calibration, every time."""
    _, ctl = service
    time.sleep(ms.STARTUP_GRACE_S)
    for node in fleet.od_drive_cal:
        fleet.od_drive_cal[node] = False
    fleet.od_phase_open[fleet.od_id["lower_leg_left"]] = True
    assert ctl.request("odrive_calibrate")["ok"]
    time.sleep(0.6)
    st = drives_done(ctl)
    assert not st["estopped"], st["estop_reason"]
    bad = st["joints"]["lower_leg_left"]
    assert bad["drive"]["procedure_result"] == "PHASE_RESISTANCE_OUT_OF_RANGE"
    assert "CALIBRATION_ERROR" in bad["drive"]["errors"]
    assert "phase wires" in bad["fault"]
    for name in ("foot_left", "lower_leg_right", "foot_right"):
        assert st["joints"][name]["drive"]["procedure_result"] == "SUCCESS", name
    assert ctl.request("calibrate", pose="straight")["ok"]
    rep = ctl.request("enable")
    assert not rep["ok"] and "lower_leg_left" in rep["error"] and "fault" in rep["error"]
    assert not status(ctl)["estopped"]


def test_odrive_calibrate_clears_a_latched_error(service, fleet):
    """After a repair the old CALIBRATION_ERROR is still latched in the drive: the
    retry must clear it first, then succeed."""
    _, ctl = service
    time.sleep(ms.STARTUP_GRACE_S)
    node = fleet.od_id["lower_leg_left"]
    fleet.od_err[node], fleet.od_proc[node], fleet.od_drive_cal[node] = 0x40000000, 6, False
    time.sleep(0.1)
    assert status(ctl)["joints"]["lower_leg_left"]["fault"]
    assert ctl.request("odrive_calibrate", joints=["lower_leg_left"])["ok"]
    time.sleep(0.6)
    st = drives_done(ctl)
    j = st["joints"]["lower_leg_left"]
    assert j["drive"]["procedure_result"] == "SUCCESS" and j["fault"] is None, j
    assert ctl.request("calibrate", pose="straight")["ok"]
    assert ctl.request("enable")["ok"]
    ctl.request("disable")


def test_the_fleet_runs_without_the_dead_joint(fleet, tmp_path):
    """motor_service --only <the other seven>: the bad knee isn't part of the fleet."""
    calib = tmp_path / "joint_calib.py"
    write_test_calib(calib)
    fleet.od_phase_open[fleet.od_id["lower_leg_left"]] = True
    fleet.od_err[fleet.od_id["lower_leg_left"]] = 0x40000000
    others = [j for j in TEST_CAL if j != "lower_leg_left"]
    svc = ms.MotorService(dm_channel=fleet.dm_channel, od_channel=fleet.od_channel,
                          bustype="virtual", calib_path=calib, only=others)
    th = threading.Thread(target=svc.run, daemon=True)
    th.start()
    ctl = ipc.CtlClient()
    try:
        st = enabled(ctl, fleet)
        assert st["enabled"] and "lower_leg_left" not in st["joints"]
        ctl.request("disable")
    finally:
        svc._stop.set()
        th.join(timeout=3.0)
        ctl.close()


def test_estop_reset_clears_the_drives_errors(service, fleet):
    _, ctl = service
    enabled(ctl, fleet)
    node = fleet.od_id["foot_right"]
    fleet.od_err[node] = 0x1000                          # CURRENT_LIMIT_VIOLATION
    st = wait_for(lambda: (s := status(ctl))["estopped"] and s, 1.0, "e-stop on the fault")
    assert "CURRENT_LIMIT_VIOLATION" in st["estop_reason"]
    assert ctl.request("estop_reset")["ok"]
    time.sleep(0.2)
    assert fleet.od_err[node] == 0
    assert status(ctl)["joints"]["foot_right"]["fault"] is None
