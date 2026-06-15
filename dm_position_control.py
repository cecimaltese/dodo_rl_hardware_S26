import time
from damiao_motor import DaMiaoController

MOTOR_ID = 0x02
FEEDBACK_ID = 0x12

KP = 30.0
KD = 0.5


def goto(motor, target_rad, duration=2.0, rate_hz=200, ramp=0.0):
    print(f"\nMoving to {target_rad:.3f} rad (duration={duration:.1f}s, ramp={ramp:.1f}s)")
    dt = 1.0 / rate_hz
    t0 = time.time()
    q0 = (motor.get_states() or {}).get("pos", 0.0)

    while time.time() - t0 < duration:
        t = time.time() - t0

        if ramp > 0 and t < ramp:
            s = t / ramp
            alpha = 3*s**2 - 2*s**3
            q_cmd = q0 + alpha * (target_rad - q0)
        else:
            q_cmd = target_rad

        motor.send_cmd_mit(
            target_position=q_cmd,
            target_velocity=0.0,
            stiffness=KP,
            damping=KD,
            feedforward_torque=0.0,
        )
        time.sleep(dt)






controller = DaMiaoController(channel="can0", bustype="socketcan")
motor = controller.add_motor(motor_id=MOTOR_ID, feedback_id=FEEDBACK_ID,
                             motor_type="4310")

motor.ensure_control_mode("MIT")

ans = input("\nSet zero at current shaft position? (y/N): ").strip().lower()
if ans == "y":
    motor.disable()
    time.sleep(0.1)
    motor.set_zero_position()
    time.sleep(0.2)
    print("  Zero set.")

controller.enable_all()
time.sleep(0.1)



# ─────────────────────────────────────────────────────────────
goto(motor, +2.0, duration=5.0, ramp=0.0)
goto(motor,  0.0, duration=3.0, ramp=1.0)

controller.shutdown()
