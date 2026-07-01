"""
demo_combined.py — drive a Damiao motor (radians) and an ODrive motor (turns)
through the unified MotorController, with BOTH motors on the same CAN bus (can0).

This replaces the old USB-ODrive demo: the ODrive now speaks CAN Simple via
OdriveCanJoint, and everything goes through MotorController so the motors share
one filtered bus and one control loop. The same loop is the sim-to-real hook:
swap the hand-written moves below for `mc.step(policy_action)` driven by the
trained IsaacLab policy.

Before running:
    sudo ip link set can0 up type can bitrate 1000000
    python3 odrive_can_setup.py            # once, to put the ODrive on CAN (node 7, 1M)
"""

import math
import time

from damiao_joint import DamiaoJointConfig
from odrive_can_joint import OdriveCanJointConfig
from motor_controller import MotorController, MotorControllerConfig, PolicyJoint

# ── configuration (bench: 1 Damiao + 1 ODrive) ────────────────────
DM_CONFIGS = [
    DamiaoJointConfig(name="dm_bench", motor_id=0x02, feedback_id=0x12,
                      motor_type="4310", kp=32.0, kd=3.0, rated_torque=9.0),
]
OD_CONFIGS = [
    OdriveCanJointConfig(name="od_bench", node_id=7, rated_torque=27.0),
]

# Policy joint order + home pose (radians). On the bench this is just the two
# test motors; on the robot, list all 8 in the sim's articulation order with the
# matching default_pos from dodo.py (hip 0.0, upper_leg -0.15, lower_leg 0.30,
# foot -0.15).
POLICY_JOINTS = [
    PolicyJoint(name="dm_bench", default_pos=0.0),
    PolicyJoint(name="od_bench", default_pos=0.0),
]

CONTROL_HZ = 200.0
DT = 1.0 / CONTROL_HZ


def spin_until(mc, done, extra=None):
    """Tick every joint at CONTROL_HZ until `done` returns True."""
    t0 = time.time()
    while not done():
        if extra is not None:
            extra(time.time() - t0)
        mc.update_all()
        time.sleep(DT)


def main():
    mc = MotorController(MotorControllerConfig(
        dm_configs=DM_CONFIGS,
        od_configs=OD_CONFIGS,
        policy_joints=POLICY_JOINTS,
    ))
    # Give the ODrive reader thread a moment to receive the first feedback frames.
    time.sleep(0.3)

    try:
        # ── optional zeroing ──────────────────────────────────────
        if input("\nSet zero at current shaft position(s)? (y/N): ").strip().lower() == "y":
            mc.set_zero_all()
            print("  Zero set.")

        mc.enable_all()

        # 1) BLOCKING goto (homing) — each motor in its own native units
        print("\n[1] Blocking goto to 0 ...")
        for j in mc.joints:
            j.goto(0.0, duration=2.0, ramp=1.0)

        # 2) NON-BLOCKING coordinated transition through the shared loop
        print("[2] Non-blocking transition ...")
        for j in mc.joints:
            j.start_move(+1.5, duration=2.0)        # rad for DM, turns for ODrive
        spin_until(mc, done=lambda: not any(j.is_moving() for j in mc.joints))
        print("    transition done.")

        # 3) POLICY path — exercise the sim-to-real hook directly.
        #    Here we feed a sine "action" through mc.step() instead of a real
        #    policy; on the robot this is where policy(obs) plugs in.
        print("[3] Policy-style step() (5 s sine) ...")
        t0 = time.time()
        while (t := time.time() - t0) < 5.0:
            a = 0.5 * math.sin(2.0 * math.pi * 0.5 * t)
            mc.step([a] * len(POLICY_JOINTS))
            time.sleep(DT)

        # quick look at the assembled observation (motor part)
        obs = mc.get_observation()
        print("    obs.joint_pos_rel =", [round(x, 3) for x in obs["joint_pos_rel"]])

        # 4) Homing
        print("[4] Easing back to 0 ...")
        for j in mc.joints:
            j.start_move(0.0, duration=2.0)
        spin_until(mc, done=lambda: not any(j.is_moving() for j in mc.joints))

    finally:
        mc.shutdown()
        print("\nDone, buses shut down.")


if __name__ == "__main__":
    main()
