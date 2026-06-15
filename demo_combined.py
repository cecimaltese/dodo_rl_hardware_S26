"""
demo.py — drive a Damiao motor (radians) and an ODrive motor (turns) through a
single shared control loop.

Both joints expose the same interface (enable/disable, set_zero, get_position,
send_position, goto, start_move/is_moving/update), so the spin loop below treats
them uniformly. The only thing that differs is *units*: Damiao is in radians,
ODrive is in turns (1 turn = 2*pi rad). Targets are written in each motor's own
native units.
"""

import math
import time

from damiao_motor import DaMiaoController

from damiao_joint import DamiaoJoint, DamiaoJointConfig
from odrive_position_control import OdriveJoint, OdriveJointConfig

# ── configuration ─────────────────────────────────────────────────
DM_JOINTS = [
    DamiaoJointConfig(name="dm_bench", motor_id=0x02, feedback_id=0x12,
                      motor_type="4310", kp=30.0, kd=0.5),
]

OD_JOINTS = [
    OdriveJointConfig(name="od_bench", axis=0),
]

CONTROL_HZ = 200.0
DT = 1.0 / CONTROL_HZ


def spin_until(joints, done, extra=None):
    """Stand-in for a ROS executor: tick every joint at CONTROL_HZ until `done`."""
    t0 = time.time()
    while not done():
        t = time.time() - t0
        if extra is not None:
            extra(t)
        for j in joints:
            j.update()
        time.sleep(DT)


def main():
    # Damiao motors share one CAN controller; ODrive joints connect over USB.
    dm_controller = DaMiaoController(channel="can0", bustype="socketcan")
    dm_joints = [DamiaoJoint(dm_controller, cfg) for cfg in DM_JOINTS]
    od_joints = [OdriveJoint(cfg) for cfg in OD_JOINTS]

    joints = dm_joints + od_joints

    # ── optional zeroing ──────────────────────────────────────────
    if input("\nSet zero at current shaft position(s)? (y/N): ").strip().lower() == "y":
        for j in joints:
            j.set_zero()
        print("  Zero set.")

    # ── enable ────────────────────────────────────────────────────
    dm_controller.enable_all()      # enables all Damiao motors on the bus
    for j in od_joints:
        j.enable()                  # ODrive: enter closed-loop control
    time.sleep(0.1)

    try:
        # 1) BLOCKING goto (homing) — each motor in its own units
        print("\n[1] Blocking goto to 0 ...")
        for j in dm_joints:
            j.goto(0.0, duration=2.0, ramp=1.0)        # rad
        for j in od_joints:
            j.goto(0.0, duration=2.0, ramp=1.0)        # turns

        # 2) NON-BLOCKING coordinated transition through the shared loop
        print("[2] Non-blocking transition ...")
        for j in dm_joints:
            j.start_move(+1.5, duration=2.0)           # rad
        for j in od_joints:
            j.start_move(+1.5, duration=2.0)           # turns
        spin_until(joints, done=lambda: not any(j.is_moving() for j in joints))
        print("    transition done.")

        # 3) POLICY path — sine around current pose, both motors in lock-step
        print("[3] Policy-style send_position (5 s sine around current pose) ...")
        centers = [j.get_position() for j in joints]
        t0 = time.time()
        while (t := time.time() - t0) < 5.0:
            for j, c in zip(joints, centers):
                j.send_position(c + 0.5 * math.sin(2.0 * math.pi * 0.5 * t))
            time.sleep(DT)

        # 4) Homing
        print("[4] Easing back to 0 ...")
        for j in joints:
            j.start_move(0.0, duration=2.0)
        spin_until(joints, done=lambda: not any(j.is_moving() for j in joints))

    finally:
        dm_controller.shutdown()
        for j in od_joints:
            j.shutdown()
        print("\nDone, buses shut down.")


if __name__ == "__main__":
    main()
