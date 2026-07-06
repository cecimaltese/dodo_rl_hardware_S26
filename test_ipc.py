"""
test_ipc.py — unit tests for dodo_ipc.py (schemas, sockets, ctl, rate loop).

Runs anywhere (no hardware, no robot model): pure pyzmq over localhost tcp on
randomized ports, so parallel test runs / leftover services don't collide.

    .venv/bin/python -m pytest test_ipc.py -v
"""
from __future__ import annotations

import math
import socket
import threading
import time

import pytest

import dodo_ipc as ipc


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------
def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@pytest.fixture(autouse=True)
def isolated_ports(monkeypatch):
    """Every test gets its own ports so tests never cross-talk."""
    for topic in ipc.DEFAULT_PORTS:
        monkeypatch.setenv(f"DODO_ZMQ_PORT_{topic.upper()}", str(_free_port()))
    yield


def _settle():
    """PUB/SUB slow-joiner settle: give the connection a moment before sending."""
    time.sleep(0.25)


def _states(seed: float = 0.0) -> ipc.MotorStates:
    n = ipc.N_JOINTS
    return ipc.MotorStates(
        pos=[seed + 0.1 * i for i in range(n)],
        vel=[seed - 0.2 * i for i in range(n)],
        torque=[0.5 * i for i in range(n)],
        joint_flags=[ipc.JF_HAS_FEEDBACK | ipc.JF_ENABLED] * n,
        global_flags=ipc.GF_ENABLED,
    )


# ---------------------------------------------------------------------------
# contract guards
# ---------------------------------------------------------------------------
def test_wire_order_matches_policy_order():
    """dodo_ipc.JOINT_ORDER (wire) and rl_env.POLICY_JOINT_ORDER (policy) are
    mapped by name in ZmqEnv, so a drift wouldn't scramble legs — but they are
    MEANT to be identical; a mismatch signals someone changed one without the
    other. (Skipped under the DODO_JOINT_ORDER sim-experiment override.)"""
    import os
    if os.environ.get("DODO_JOINT_ORDER"):
        pytest.skip("DODO_JOINT_ORDER override active")
    from rl_env import POLICY_JOINT_ORDER
    assert list(ipc.JOINT_ORDER) == list(POLICY_JOINT_ORDER)


# ---------------------------------------------------------------------------
# schemas
# ---------------------------------------------------------------------------
def test_motor_states_roundtrip():
    m = _states(1.0)
    out = ipc.decode(m.pack(seq=42))
    assert isinstance(out, ipc.MotorStates)
    assert out.seq == 42
    assert out.pos == pytest.approx(m.pos, abs=1e-6)
    assert out.vel == pytest.approx(m.vel, abs=1e-6)
    assert out.torque == pytest.approx(m.torque, abs=1e-6)
    assert out.joint_flags == m.joint_flags
    assert out.enabled and not out.estopped and not out.all_calibrated
    assert out.flag(0, ipc.JF_HAS_FEEDBACK)
    assert not out.flag(0, ipc.JF_CALIBRATED)


def test_motor_commands_roundtrip_with_nan():
    t = [0.1, ipc.NO_COMMAND, -0.5, 0.3, ipc.NO_COMMAND, 0.0, 1.2, -1.2]
    out = ipc.decode(ipc.MotorCommands(target_pos=t).pack(seq=7))
    assert isinstance(out, ipc.MotorCommands)
    assert out.seq == 7
    for got, want in zip(out.target_pos, t):
        if math.isnan(want):
            assert ipc.no_command(got)
        else:
            assert got == pytest.approx(want, abs=1e-6)


def test_imu_roundtrip_and_flags():
    m = ipc.ImuSample(gyro=(0.1, -0.2, 0.3), accel=(0.0, 0.0, -9.81),
                      quat=(1.0, 0.0, 0.0, 0.0), lin_vel=(0.5, 0.0, 0.0),
                      flags=ipc.IMU_QUAT_VALID | ipc.IMU_LIN_VEL_VALID)
    out = ipc.decode(m.pack(seq=3))
    assert isinstance(out, ipc.ImuSample)
    assert out.quat_valid and out.lin_vel_valid
    assert tuple(out.gyro) == pytest.approx(m.gyro, abs=1e-6)
    assert tuple(out.accel) == pytest.approx(m.accel, abs=1e-6)
    assert tuple(out.quat) == pytest.approx(m.quat, abs=1e-6)
    assert tuple(out.lin_vel) == pytest.approx(m.lin_vel, abs=1e-6)
    # hardware default: no quat, no lin_vel
    bare = ipc.decode(ipc.ImuSample(gyro=(0, 0, 0), accel=(0, 0, 0)).pack(seq=0))
    assert not bare.quat_valid and not bare.lin_vel_valid


def test_decode_rejects_garbage():
    with pytest.raises(ValueError):
        ipc.decode(b"\x00" * 4)                       # too short
    good = _states().pack(seq=0)
    bad_version = b"\xff\xff" + good[2:]
    with pytest.raises(ValueError):
        ipc.decode(bad_version)                       # wrong schema version
    bad_type = good[:2] + b"\xee\xee" + good[4:]
    with pytest.raises(ValueError):
        ipc.decode(bad_type)                          # unknown msg type


def test_age_is_sane():
    m = ipc.decode(_states().pack(seq=0))
    assert 0.0 <= ipc.age_s(m) < 0.5


# ---------------------------------------------------------------------------
# pub/sub delivery semantics
# ---------------------------------------------------------------------------
def test_conflate_subscriber_gets_newest():
    pub = ipc.Publisher(ipc.TOPIC_MOTOR_STATES)
    sub = ipc.Subscriber(ipc.TOPIC_MOTOR_STATES, conflate=True)
    try:
        _settle()
        for i in range(50):
            pub.send(_states(float(i)))
        time.sleep(0.2)                               # let delivery finish
        got = sub.recv(timeout_s=1.0)
        assert got is not None
        assert got.seq == 49                          # newest, not a backlog
        assert sub.latest() is None                   # queue fully drained
    finally:
        pub.close()
        sub.close()


def test_nonconflate_subscriber_sees_every_message():
    pub = ipc.Publisher(ipc.TOPIC_IMU)
    sub = ipc.Subscriber(ipc.TOPIC_IMU, conflate=False)
    try:
        _settle()
        n = 30
        for _ in range(n):
            pub.send(ipc.ImuSample(gyro=(0, 0, 0), accel=(0, 0, 0)))
            time.sleep(0.002)
        seqs = []
        while True:
            m = sub.recv(timeout_s=0.3)
            if m is None:
                break
            seqs.append(m.seq)
        assert seqs == list(range(n))                 # no drops, in order
    finally:
        pub.close()
        sub.close()


def test_latest_returns_none_when_nothing_new():
    pub = ipc.Publisher(ipc.TOPIC_MOTOR_COMMANDS)
    sub = ipc.Subscriber(ipc.TOPIC_MOTOR_COMMANDS)
    try:
        _settle()
        assert sub.latest() is None
        pub.send(ipc.MotorCommands(target_pos=[0.0] * ipc.N_JOINTS))
        time.sleep(0.2)
        assert sub.latest() is not None
        assert sub.latest() is None                   # consumed; nothing new
    finally:
        pub.close()
        sub.close()


# ---------------------------------------------------------------------------
# ctl REQ/REP
# ---------------------------------------------------------------------------
def test_ctl_roundtrip_unknown_and_error():
    server = ipc.CtlServer()
    stop = threading.Event()

    def boom(_args):
        raise RuntimeError("kaboom")

    handlers = {"ping": lambda a: {"pong": a.get("x", 0)}, "boom": boom}

    def serve():
        while not stop.is_set():
            server.poll_once(handlers)
            time.sleep(0.002)

    t = threading.Thread(target=serve, daemon=True)
    t.start()
    client = ipc.CtlClient()
    try:
        assert client.request("ping", x=5) == {"ok": True, "pong": 5}
        r = client.request("nope")
        assert r["ok"] is False and "unknown" in r["error"]
        r = client.request("boom")
        assert r["ok"] is False and "kaboom" in r["error"]
    finally:
        stop.set()
        t.join(timeout=1.0)
        client.close()
        server.close()


def test_ctl_timeout_raises_and_recovers():
    client = ipc.CtlClient()                          # nothing bound on this port
    try:
        with pytest.raises(TimeoutError):
            client.request("ping", timeout_s=0.2)
        # client must have rebuilt its socket and still be usable
        server = ipc.CtlServer()
        stop = threading.Event()
        t = threading.Thread(
            target=lambda: [server.poll_once({"ping": lambda a: {}}) or time.sleep(0.002)
                            for _ in iter(lambda: stop.is_set(), True)],
            daemon=True)
        t.start()
        try:
            assert client.request("ping", timeout_s=2.0)["ok"] is True
        finally:
            stop.set()
            t.join(timeout=1.0)
            server.close()
    finally:
        client.close()


# ---------------------------------------------------------------------------
# RateLoop
# ---------------------------------------------------------------------------
def test_rateloop_holds_rate():
    loop = ipc.RateLoop(200.0, name="test")
    t0 = time.monotonic()
    for _ in range(50):
        loop.sleep()
    elapsed = time.monotonic() - t0
    # 50 ticks @ 5 ms = 0.25 s; generous upper bound for a loaded CI machine.
    assert 0.22 <= elapsed <= 0.40, f"elapsed {elapsed:.3f}s"
    assert loop.ticks == 50


def test_rateloop_counts_overruns_without_bursting():
    loop = ipc.RateLoop(100.0, name="overrun")       # 10 ms period
    t0 = time.monotonic()
    for _ in range(5):
        time.sleep(0.03)                              # work 3x the period
        loop.sleep()
    elapsed = time.monotonic() - t0
    assert loop.overruns >= 4
    assert loop.worst_late_s > 0.0
    # no catch-up burst: total time ≈ work time, not work + owed sleep
    assert elapsed < 0.25
