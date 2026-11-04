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
encoder + heartbeat frames every 10 ms, RTR answers, Set_Axis_State).
"""
from __future__ import annotations

import socket
import struct
import threading
import time
import uuid

import can
import pytest

import calibrate_joints
import dodo_ipc as ipc
import motor_service as ms
from fleet_config import DM_JOINTS, FEEDBACK_FRESH_S, OD_JOINTS
from odrive_can_joint import (AXIS_STATE_IDLE, CMD_GET_ENCODER_ESTIMATES,
                              CMD_HEARTBEAT, CMD_SET_AXIS_STATE)

pytestmark = pytest.mark.filterwarnings("ignore::DeprecationWarning")  # damiao: bustype=

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
        self._run = True
        self._threads = [threading.Thread(target=f, daemon=True)
                         for f in (self._damiao_loop, self._odrive_loop)]
        for t in self._threads:
            t.start()

    # ── Damiao: speaks only when spoken to ──────────────────────────
    def _dm_feedback(self, mid: int) -> None:
        status = 1 if self.dm_enabled[mid] else 0
        pos_u, vel_u, tq_u = 0x7FFF, 0x7FF, 0x7FF          # ~0 rad, 0 rad/s, 0 Nm
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
                    if cmd == CMD_SET_AXIS_STATE:
                        self.od_state[node] = struct.unpack("<I", bytes(msg.data[:4]))[0]
                    elif cmd == CMD_GET_ENCODER_ESTIMATES and msg.is_remote_frame:
                        self._od_send(node, CMD_GET_ENCODER_ESTIMATES,
                                      struct.pack("<ff", 0.0, 0.0))
            if time.monotonic() >= next_cyclic:
                next_cyclic = time.monotonic() + 0.010
                for node, alive in self.od_alive.items():
                    if alive:
                        self._od_send(node, CMD_GET_ENCODER_ESTIMATES,
                                      struct.pack("<ff", 0.0, 0.0))
                        self._od_send(node, CMD_HEARTBEAT,
                                      struct.pack("<IBBBB", 0, self.od_state[node], 0, 0, 0))

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
def service(fleet):
    """The real MotorService on the fake fleet, its main loop in a thread."""
    svc = ms.MotorService(dm_channel=fleet.dm_channel, od_channel=fleet.od_channel,
                          bustype="virtual")
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
    rep = ctl.request("calibrate")
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
    rep = ctl.request("calibrate")
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
