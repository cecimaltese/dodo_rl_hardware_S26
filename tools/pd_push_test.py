"""
pd_push_test.py — measure one joint's real PD by pushing it by hand. No USB needed.

While motor_service HOLDS the joint (on the gantry, in the air: e.g. tools/dodo_startup.py
--pose home in another terminal), push the joint slowly away from where it's held, let
it come back, and push again a little harder, for --secs seconds. Push the moving link
near its end, slowly: only near-still samples count. It reads motor_states and reports:

  STIFFNESS  torque / error at small errors, against the kp motor_service set:
             ~1      the gains reached the drive, and its torque reads in the units
                     assumed (ODrive: motor-side Nm x gear ratio; fleet_config.
                     ODRIVE_TORQUE_IS_MOTOR_SIDE)
             >> 1    the drive's own gains still run (CAN Set_Pos_Gain/Set_Vel_Gains
                     didn't take; WS25's had an integrator: the error creeps back to 0
                     under a steady push), or, at ~the gear ratio, the ODrive's torque
                     is already joint-side
             < 0     the reported torque's sign is inverted against the error
  CAP        the most torque it held. An ODrive in position control holds at most
             vel_gain x vel_limit at standstill (its velocity setpoint saturates), so
             a torque PLATEAU while the error keeps growing is that, or the drive's
             current limit. Run again at another gain scale (dodo_ctl gains --arg
             scale=...) to tell which: the vel_limit cap scales with sqrt(scale), a
             current limit doesn't. The report turns the plateau into the drive's
             vel_limit and the cap at full gains, against what the PD stand needs.

    python3 tools/pd_push_test.py --joint lower_leg_left        # 30 s
    python3 tools/pd_push_test.py --joint foot_right --secs 20

Read-only: it commands nothing. A Damiao joint works too (no velocity cap: its MIT
torque law is exact up to the motor's limit).
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import dodo_ipc as ipc  # noqa: E402
import fleet_config  # noqa: E402
import real2sim  # noqa: E402

STILL_RAD_S = 0.15     # only samples slower than this are "static"
KT_MOTOR = 8.27 / 300  # Nm/A, the MN4004 KV300 (torque_constant = 8.27 / KV)
# The joint torques the PD stand needs in sim (`home`, sim_service --scene): peak while
# settling, settled ~0.5 Nm less (HANDOFF log 2026-11-05 (c), (e)).
STAND_NEEDS_NM = {"hip": 1.1, "upper_leg": 0.9, "lower_leg": 2.9, "foot": 1.4}


def analyze(err, tau, vel, kp: float, kd: float, joint: str,
            gain_scale: float = 1.0) -> dict:
    """The push test's verdicts from samples of error (target - pos) [rad], joint
    torque [Nm] and velocity [rad/s]. kp/kd: the gains acting (status, i.e. scaled)."""
    err, tau, vel = (np.asarray(x, dtype=float) for x in (err, tau, vel))
    still = np.abs(vel) < STILL_RAD_S
    e, t = err[still], tau[still]
    out = {"joint": joint, "kp": kp, "n_still": int(still.sum()),
           "max_err": float(np.abs(e).max()) if e.size else 0.0}
    if e.size < 20 or out["max_err"] < 0.01:
        out["verdict"] = "not enough push: deflect it further (a few degrees), slowly"
        return out

    small = (np.abs(e) <= max(0.02, 0.3 * out["max_err"])) & (np.abs(e) > 0.002)
    if small.sum() < 5:
        small = np.abs(e) > 0.002
    k = float(np.sum(e[small] * t[small]) / np.sum(e[small] ** 2))
    out["k_measured"] = k
    out["k_ratio"] = k / kp

    restoring = t * np.sign(e)                         # torque toward the target
    hi = np.abs(e) >= np.quantile(np.abs(e), 0.8)
    e_hi, t_hi = float(np.median(np.abs(e[hi]))), float(np.median(restoring[hi]))
    out["tau_max"] = float(restoring.max())
    out["saturated"] = bool(k > 0 and t_hi < 0.75 * k * e_hi)
    out["tau_plateau"] = t_hi if out["saturated"] else None

    group = fleet_config.joint_group(joint)
    need = STAND_NEEDS_NM[group]
    out["stand_needs"] = need
    if real2sim.is_odrive(joint):
        n = real2sim.gear_ratio(joint)
        _, vg, _ = real2sim.odrive_gains(joint, kp, kd)
        _, vg_full, _ = real2sim.odrive_gains(joint, fleet_config.JOINT_KP[group],
                                              fleet_config.JOINT_KD[group])
        if out["saturated"]:
            out["vel_limit_if_velocity_cap"] = t_hi / n / vg          # motor turns/s
            out["cap_full_gains"] = vg_full * out["vel_limit_if_velocity_cap"] * n
            out["current_if_current_cap"] = t_hi / n / KT_MOTOR       # A
        else:
            # Unsaturated up to tau_max: the velocity cap is above it, and scales with
            # sqrt(scale) up to full gains.
            out["cap_full_gains_at_least"] = out["tau_max"] / math.sqrt(gain_scale)

    msgs = []
    if k <= 0:
        msgs.append("torque sign INVERTED against the error (torque_scale wrong)")
    elif out["k_ratio"] > 3:
        msgs.append("much STIFFER than configured: the drive's own gains (integrator?) "
                    "still run, or its torque is already joint-side")
    elif out["k_ratio"] < 0.5:
        msgs.append("much SOFTER than configured: gains not applied as computed, or "
                    "the torque reads low")
    else:
        msgs.append("stiffness matches kp: gains and torque units OK")
    if real2sim.is_odrive(joint):
        if out["saturated"]:
            cap = out["cap_full_gains"]
            msgs.append(f"PLATEAU at {t_hi:.2f} Nm: if it's the velocity cap, vel_limit "
                        f"~ {out['vel_limit_if_velocity_cap']:.1f} turns/s and full gains "
                        f"hold {cap:.1f} Nm ({'enough' if cap > 1.3 * need else 'NOT enough'}"
                        f" for the stand's {need} Nm); if it's the current limit, "
                        f"~ {out['current_if_current_cap']:.1f} A. Run again at another "
                        f"gain scale to tell which.")
        else:
            lb = out["cap_full_gains_at_least"]
            msgs.append(f"no plateau up to {out['tau_max']:.2f} Nm: full gains hold at "
                        f"least {lb:.1f} Nm (the stand needs {need} Nm)"
                        + ("" if lb > 1.3 * need else "; push harder to find the cap"))
    out["verdict"] = "; ".join(msgs)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--joint", required=True, choices=ipc.JOINT_ORDER)
    ap.add_argument("--secs", type=float, default=30.0)
    ap.add_argument("--rate", type=float, default=200.0, help="sampling rate [Hz]")
    args = ap.parse_args()

    ctl = ipc.CtlClient()
    sub = ipc.Subscriber(ipc.TOPIC_MOTOR_STATES, conflate=True)
    try:
        st = ctl.request("status")
        j = st.get("joints", {}).get(args.joint, {})
        if not st.get("enabled") or j.get("target_rad") is None:
            sys.exit(f"{args.joint} isn't being held (enabled={st.get('enabled')}, "
                     f"target={j.get('target_rad')}): start e.g. tools/dodo_startup.py "
                     f"--pose home, then run this")
        kp, kd = j.get("kp"), j.get("kd")
        if kp is None or kd is None:
            sys.exit("this service doesn't report kp/kd (sim_service's stand-policy "
                     "model?); use motor_service, or sim_service --scene")
        scale = float(st.get("gain_scale", 1.0))
        i = ipc.JOINT_INDEX[args.joint]
        print(f"{args.joint}: held at {j['target_rad']:+.3f} rad, kp {kp} kd {kd} "
              f"(gain scale {scale}). Push it slowly, release, push harder; "
              f"{args.secs:.0f} s (Ctrl-C ends early).")
        time.sleep(0.2)
        loop = ipc.RateLoop(args.rate, name="pd_push_test")
        err, tau, vel = [], [], []
        target, t0, t_status, last_print = j["target_rad"], time.monotonic(), 0.0, 0.0
        try:
            while time.monotonic() - t0 < args.secs:
                now = time.monotonic()
                if now - t_status > 1.0:              # follow the held target
                    jt = ctl.request("status").get("joints", {}).get(args.joint, {})
                    if jt.get("target_rad") is None:
                        print("\nno longer held: stopping")
                        break
                    target, t_status = jt["target_rad"], now
                m = sub.latest()
                if m is not None:
                    e = target - m.pos[i]
                    err.append(e)
                    tau.append(m.torque[i])
                    vel.append(m.vel[i])
                    if now - last_print > 0.25:
                        top = max((t * math.copysign(1, x) for x, t in zip(err, tau)),
                                  default=0.0)
                        print(f"\r  error {math.degrees(e):+6.2f} deg   torque "
                              f"{m.torque[i]:+6.2f} Nm   most held {top:5.2f} Nm  ",
                              end="", flush=True)
                        last_print = now
                loop.sleep()
        except KeyboardInterrupt:
            pass
        print()
        r = analyze(err, tau, vel, kp, kd, args.joint, scale)
        print(f"\n{len(err)} samples, {r['n_still']} still; largest error "
              f"{math.degrees(r['max_err']):.1f} deg")
        if "k_measured" in r:
            print(f"stiffness {r['k_measured']:.1f} Nm/rad vs kp {kp} "
                  f"(ratio {r['k_ratio']:.2f}); most torque held {r['tau_max']:.2f} Nm")
        print(f"=> {r['verdict']}")
    finally:
        sub.close()
        ctl.close()


if __name__ == "__main__":
    main()
