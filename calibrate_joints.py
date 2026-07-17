"""
calibrate_joints.py — guided sign / zero / limit calibration for the whole fleet.
READ-ONLY: never commands motion. You move the robot by hand; this only watches.

    python3 calibrate_joints.py                 # main pass: pose the crouch, nudge each joint
    python3 calibrate_joints.py --verify        # prove the signs before commanding anything
    python3 calibrate_joints.py --limits        # opt-in: hard stops -> clamps, enables --home
    python3 calibrate_joints.py --home          # re-zero at startup (see HOMING)
    python3 calibrate_joints.py --only hip_left # redo one joint
    python3 calibrate_joints.py --dry-run       # no hardware; see the prompts
    python3 calibrate_joints.py --check-persistence  # power-cycle test: is a stored zero valid at all?

THE MAIN PASS, IN ORDER
-----------------------
  1. POSE the robot once at the sim's default crouch. One snapshot of all 8 motors. This
     is taken FIRST, while you are careful — not last, when you are eight joints deep.
  2. NUDGE each joint from there, one at a time, the sim's + way. Each nudge names its own
     motor (mapping) and its own direction (sign). Small moves; no hard stops.
  3. zero = crouch_raw - sign * to_native(default) — it falls out of steps 1 and 2.

The hard stops are NOT in the main pass. They are the slow, risky half (you are pushing a
joint into metal, and a stop you ease off before is a stop you measured WRONG), and they
are only needed for position clamps and for --home. Run --limits when you want them.

WHAT IT SOLVES
--------------
Three unknowns block the policy, and all three are the same shape: the sim believes
something about the hardware that nobody has checked.

  1. MAPPING  — which motor drives which joint. The ODrive scan proves nodes 3/4/7/8
                exist; it cannot say which is a knee and which an ankle. WS25 never
                recorded it either.
  2. SIGN     — does +angle in the sim mean +reading on the motor? The measured Damiao
                limits are left/right MIRRORED while the URDF says both sides share an
                axis, so at least one side is very likely inverted.
  3. ZERO     — the motor's 0 is not the sim's 0. The sim's default crouch is
                hip 0 / upper_leg 0.2 / lower_leg -0.5 / foot 0.3 rad.

Get any of them wrong and the joint drives the WRONG WAY into its hard stop under a
27 Nm motor. A gantry stops the robot falling; it does not stop it breaking itself.

HOW IT AVOIDS HUMAN ERROR
-------------------------
  * It never asks "which motor is this?" — it WATCHES ALL 8 and picks the one that
    actually moved. You cannot mislabel a joint.
  * It rejects a move if two joints moved together, or if the winner isn't clearly
    ahead of the runner-up — so a sloppy grab doesn't silently poison a sign.
  * The + directions below are not hand-derived from URDF axis conventions; they were
    MEASURED in MuJoCo by perturbing each joint +0.3 rad from the default pose and
    watching where the foot went. Re-derive them the same way if the URDF changes.
  * Every number it writes is one it measured. Nothing is inferred.

HOMING — why the zero is not a write-once number
------------------------------------------------
The ODrives are very likely BOOT-RELATIVE: they zero their position estimate at power-up,
so a `zero` measured yesterday describes an encoder origin that no longer exists. Run
--check-persistence to confirm. What survives a power cycle is not the encoder — it is the
METAL: the mechanical hard stops don't move.

So the calibration splits into two halves:

  DURABLE (measure once)     motor, sign, sim_lo, sim_hi — the stops expressed in SIM
                             radians. Mechanical facts. Valid until the robot is rebuilt.
  PERISHABLE (per boot)      zero, raw_lo, raw_hi — encoder-origin-relative. For a
                             boot-relative motor these are stale the moment it powers down.

`--home` rebuilds the perishable half from the durable half: push each joint to ONE known
hard stop, read the raw value there, and back-solve zero = raw - sign*to_native(sim_hi).
That is repeatable to the metal, not to your eye — which is why homing beats re-posing at
the crouch. A full run does the eyeball pose ONCE (it is what defines sim_lo/sim_hi in the
first place); every boot after that is --home.

OUTPUT: joint_calib.py — a plain dict of {joint: {motor, sign, zero, sim_lo, sim_hi,
raw_lo, raw_hi}} for _JointAdapter to consume. Re-runnable; writes MERGE into the existing
file, so --only patches a single joint and --home patches only the zeros.
"""

from __future__ import annotations

import argparse
import dataclasses
import math
import statistics
import sys
import threading
import time
from typing import Dict, Optional

import can
from damiao_motor import DaMiaoController

from damiao_joint import DamiaoJoint
from odrive_can_joint import OdriveCanJoint
from fleet_config import DM_JOINTS, OD_JOINTS, DM_CHANNEL, OD_CHANNEL, BUSTYPE
from rl_env import DEFAULT_JOINT_POS, POLICY_JOINT_ORDER

TWO_PI = 2.0 * math.pi

# What +angle DOES, physically, in the base frame (x fwd, y left, z up).
# MEASURED in MuJoCo: each joint perturbed +0.3 rad from the default crouch; this is
# where the FOOT went. NOT derived by hand from the URDF axis tags.
#
# Note both sides move the SAME way in sim (hips -> foot LEFT, upper_leg -> foot
# BACKWARD). The sim is NOT mirrored. If the real left and right move OPPOSITE for the
# same reading sign, that IS the mirroring, and one side gets sign = -1.
PLUS_MEANS = {
    "hip_left":        "the whole leg swings so the FOOT moves LEFT (~11 cm)",
    "hip_right":       "the whole leg swings so the FOOT moves LEFT (~12 cm)",
    "upper_leg_left":  "the thigh swings so the FOOT moves BACKWARD (~12 cm)",
    "upper_leg_right": "the thigh swings so the FOOT moves BACKWARD (~12 cm)",
    "lower_leg_left":  "the knee bends so the FOOT moves BACKWARD (~6 cm)",
    "lower_leg_right": "the knee bends so the FOOT moves BACKWARD (~6 cm)",
    "foot_left":       "the ankle pitches so the TOE goes DOWN (~2 cm)",
    "foot_right":      "the ankle pitches so the TOE goes DOWN (~2 cm)",
}

MOVE_MIN = 0.05          # rad — below this the joint didn't really move (~2.9 deg)
DOMINANCE = 3.0          # winner must move >= 3x the runner-up, else it's ambiguous
# Both thresholds are compared in RADIANS, after to_rad(). Never against native units:
# the two buses report different ones and the comparison would be nonsense.
SETTLE_S = 0.3


# ---------------------------------------------------------------------------
class Fleet:
    """All 8 motors, read-only. Damiao LIMP (kp=kd=0, backdrivable); ODrive listen."""

    def __init__(self, dry=False):
        self.dry = dry
        self.pos: Dict[str, Optional[float]] = {}
        self._stop = threading.Event()
        if dry:
            self.pos = {j: 0.0 for j in POLICY_JOINT_ORDER}
            return

        # Damiao — copy the config first: DM_JOINTS are the SHARED fleet_config objects
        # and zeroing kp/kd in place would disarm motor_controller's gains.
        self.dm_ctl = DaMiaoController(channel=DM_CHANNEL, bustype=BUSTYPE)
        self.dm: Dict[str, DamiaoJoint] = {}
        for cfg in DM_JOINTS:
            limp = dataclasses.replace(cfg, kp=0.0, kd=0.0)
            j = DamiaoJoint(self.dm_ctl, limp)
            j.enable()
            self.dm[cfg.name] = j

        # ODrive — filtered socket + reader thread. Never commanded closed-loop.
        filters = [{"can_id": c.node_id << 5, "can_mask": 0x7E0} for c in OD_JOINTS]
        self.od_bus = can.Bus(channel=OD_CHANNEL, interface=BUSTYPE, can_filters=filters)
        self.od: Dict[str, OdriveCanJoint] = {}
        self.od_by_node: Dict[int, OdriveCanJoint] = {}
        for cfg in OD_JOINTS:
            j = OdriveCanJoint(self.od_bus, cfg)
            self.od[cfg.name] = j
            self.od_by_node[cfg.node_id] = j
        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self):
        while not self._stop.is_set():
            msg = self.od_bus.recv(timeout=0.1)
            if msg is not None:
                j = self.od_by_node.get(msg.arbitration_id >> 5)
                if j is not None:
                    j.process_frame(msg)

    def read(self) -> Dict[str, Optional[float]]:
        """Every motor's position in ITS OWN native units (Damiao rad, ODrive turns)."""
        if self.dry:
            return dict(self.pos)
        out = {}
        for n, j in self.dm.items():
            try:
                out[n] = float(j.get_position())
            except Exception:
                out[n] = None
        for n, j in self.od.items():
            try:
                p = j.get_position()
                out[n] = float(p) if p is not None else None
            except Exception:
                out[n] = None
        return out

    def online(self):
        return [n for n, v in self.read().items() if v is not None]

    def close(self):
        self._stop.set()
        if self.dry:
            return
        for j in self.dm.values():
            try:
                j.disable()
            except Exception:
                pass
        try:
            self.od_bus.shutdown()
        except Exception:
            pass


# ---------------------------------------------------------------------------
def settle(fleet, n=8):
    """Median of n reads — rejects a single noisy sample."""
    acc: Dict[str, list] = {}
    for _ in range(n):
        for k, v in fleet.read().items():
            if v is not None:
                acc.setdefault(k, []).append(v)
        time.sleep(0.02)
    return {k: statistics.median(v) for k, v in acc.items() if v}


def watch_move(fleet, prompt) -> Optional[tuple]:
    """Prompt, snapshot, wait, snapshot. Return (joint, delta) for the joint that ACTUALLY
    moved — chosen by measurement, never by asking. None if ambiguous."""
    print(f"\n  {prompt}")
    input("    press Enter when the robot is STILL and ready... ")
    before = settle(fleet)
    input("    now MOVE it as described, then press Enter (hold it there)... ")
    time.sleep(SETTLE_S)
    after = settle(fleet)

    # Compare in RADIANS, not native units. Damiao reports rad and ODrive turns, so ranking
    # the raw deltas would pit 0.3 rad against 0.048 turns for the same physical motion --
    # a drifting hip would out-rank a knee that really moved, and MOVE_MIN would mean 2.9deg
    # on one bus and 18deg on the other.
    deltas = {k: to_rad(k, after[k] - before[k]) for k in before if k in after}
    if not deltas:
        print("    !! no readings at all — is the fleet online?")
        return None
    ranked = sorted(deltas.items(), key=lambda kv: abs(kv[1]), reverse=True)
    win, dwin = ranked[0]
    run, drun = ranked[1] if len(ranked) > 1 else (None, 0.0)

    if abs(dwin) < MOVE_MIN:
        print(f"    !! nothing moved enough (largest: {win} {dwin:+.3f}). Move it further.")
        return None
    if run and abs(drun) > 1e-9 and abs(dwin) / abs(drun) < DOMINANCE:
        print(f"    !! AMBIGUOUS — {win} moved {dwin:+.3f} but {run} moved {drun:+.3f} too.")
        print(f"       Move ONLY one joint. (Hold the rest of the leg still.)")
        return None
    print(f"    -> that was {win}  (moved {dwin:+.3f}; next-largest {run} {drun:+.3f})")
    return win, dwin


def is_odrive(motor):
    return motor.startswith(("lower_leg", "foot"))


def to_rad(motor, v):
    """ODrive reports turns; the sim speaks radians."""
    return v * TWO_PI if is_odrive(motor) else v


def to_native_units(motor, rad):
    """Inverse of to_rad: the sim's radians in the motor's own units."""
    return rad / TWO_PI if is_odrive(motor) else rad


def raw_to_sim(c, raw):
    """q_sim = to_rad(raw - zero) * sign"""
    return to_rad(c["motor"], raw - c["zero"]) * c["sign"]


def sim_to_raw(c, q):
    """raw = zero + sign * to_native(q_sim)"""
    return c["zero"] + c["sign"] * to_native_units(c["motor"], q)


# ---------------------------------------------------------------------------
def pose_home(fleet):
    """Snapshot every motor at the sim's default crouch. The reference the run builds on.

    Taken FIRST, while the robot is posed and you are careful — not last, when you are
    eight joints deep and tired. It needs no sign: it is just the raw reading at a known
    sim pose. The signs come next, and `zero` falls out of the two.
    """
    print("\n" + "=" * 72)
    print("  STEP 1 — pose the robot at the SIM'S DEFAULT CROUCH:")
    for j in POLICY_JOINT_ORDER:
        print(f"     {j:17s} = {DEFAULT_JOINT_POS[j]:+.2f} rad   ({PLUS_MEANS[j].split(' so ')[0]})")
    print("\n  Get it as close as you can by eye — the gantry helps. Every joint at once.")
    print("  (This is the offset between the motor's 0 and the sim's 0, so it only has")
    print("   to be as good as your pose. A few degrees of error = a few degrees of bias.)")
    print("=" * 72)
    input("\n  press Enter when the robot is HELD in the default crouch... ")
    time.sleep(SETTLE_S)
    snap = settle(fleet)
    print(f"  captured {len(snap)} motors at the crouch")
    return snap


def identify(fleet, joints, snap):
    """From the crouch, nudge one joint at a time: mapping + sign, then zero.

    One move per joint, small, starting from a known safe pose. No hard stops — see
    measure_limits() for those. The nudge need not START at the crouch (mapping and sign
    only care about the DELTA), but it is where you already are, and a small move from a
    known pose is the safest thing you can ask a human to do to a 27 Nm joint.
    """
    calib = {}
    for j in joints:
        print("\n" + "=" * 72)
        print(f"  JOINT: {j}")
        print(f"  In the SIM, +angle means: {PLUS_MEANS[j]}")
        print("=" * 72)

        res = None
        while res is None:
            res = watch_move(fleet, f"From the crouch, nudge ONLY {j} (a small move is "
                                    f"plenty), so that: {PLUS_MEANS[j]}")
            if res is None and input("    retry? [Y/n] ").strip().lower() == "n":
                break
        if res is None:
            print(f"  SKIPPED {j} — not written; re-run with --only {j}")
            continue
        motor, delta = res
        sign = 1 if delta > 0 else -1
        print(f"\n  MAPPING: {j} is driven by motor '{motor}'")
        if motor != j:
            print(f"  ** NOTE: fleet_config calls that motor '{motor}', but you moved '{j}'.")
            print(f"     The config's name->motor assignment is WRONG for this joint.")
        print(f"  SIGN:    {sign:+d}  (you moved it the sim's + way and the reading went "
              f"{'UP' if sign > 0 else 'DOWN'})")
        if sign < 0:
            print("           -> INVERTED vs the sim. The sign layer will negate it.")

        raw = snap.get(motor)
        if raw is None:
            print(f"  !! '{motor}' gave no reading at the crouch — zero NOT captured.")
            zero = None
        else:
            # target_raw = zero + sign*q_sim  ->  zero = raw - sign*default
            zero = raw - sign * to_native_units(motor, DEFAULT_JOINT_POS[j])
            print(f"  ZERO:    {zero:+.4f}  (crouch raw {raw:+.4f} - sign*"
                  f"{DEFAULT_JOINT_POS[j]:+.2f} rad)")

        # sim_lo/sim_hi are left alone: --limits measures them, and a merge-aware write
        # means an earlier --limits pass survives this run untouched.
        calib[j] = {"motor": motor, "sign": sign, "zero": zero}
    return calib


def measure_limits(fleet, joints, path="joint_calib.py"):
    """Opt-in pass: walk each joint to BOTH hard stops. Needs motor/sign/zero already.

    Split out of the main run because it is the slow, risky half — you are pushing a joint
    into metal, and a stop you ease off before is a stop you measured wrong. Only needed
    for position clamps and for --home, which back-solves the zero from sim_hi.
    """
    calib = read_calib(path)
    todo = [j for j in joints if j in calib and calib[j].get("zero") is not None]
    missing = [j for j in joints if j not in todo]
    if missing:
        print(f"\n  !! no motor/sign/zero yet for: {', '.join(missing)}")
        print("     Limits are meaningless without the zero they are measured against.")
        print(f"     Run the main pass first: python3 calibrate_joints.py --only {' '.join(missing)}")
    if not todo:
        return None

    print("\n" + "=" * 72)
    print("  LIMITS — walking each joint to its mechanical hard stops")
    print("=" * 72)
    print("""
  GENTLY, but DO reach the metal. A stop you ease off before is recorded as a stop
  that isn't there, and the clamp built from it will be wrong in the direction that
  lets the joint travel further than you think.
""")
    out = {}
    for j in todo:
        c = calib[j]
        motor = c["motor"]
        print("\n  " + "-" * 68)
        print(f"  {j}  (motor '{motor}')  + direction: {PLUS_MEANS[j]}")
        input(f"    Move {j} GENTLY to its hard stop in the sim's + direction, hold, Enter... ")
        time.sleep(SETTLE_S)
        hi_raw = settle(fleet).get(motor)
        input(f"    Now GENTLY to the OTHER hard stop, hold, Enter... ")
        time.sleep(SETTLE_S)
        lo_raw = settle(fleet).get(motor)
        if hi_raw is None or lo_raw is None:
            print(f"    !! no reading from '{motor}' — limits NOT recorded for {j}")
            continue
        if hi_raw < lo_raw:
            hi_raw, lo_raw = lo_raw, hi_raw

        span = to_rad(motor, hi_raw - lo_raw)
        if abs(span) < MOVE_MIN:
            print(f"    !! the two stops are only {span:+.3f} rad apart — that joint did not "
                  f"move. Not recorded.")
            continue
        a, b = raw_to_sim(c, lo_raw), raw_to_sim(c, hi_raw)
        upd = {"raw_lo": lo_raw, "raw_hi": hi_raw, "sim_lo": min(a, b), "sim_hi": max(a, b)}
        out[j] = upd
        print(f"    raw [{lo_raw:+.4f}, {hi_raw:+.4f}] "
              f"({'turns' if is_odrive(motor) else 'rad'})  ->  sim "
              f"[{upd['sim_lo']:+.3f}, {upd['sim_hi']:+.3f}] rad")

    if out:
        write_calib(out, path)
        print(f"\n  LIMITS RECORDED ({len(out)}/{len(todo)}): {', '.join(out)}")
        print("  --home can now re-zero against these stops.")
    return out


def verify(fleet, joints, path="joint_calib.py"):
    """Read-only proof: move a joint by hand, see what the CALIBRATED layer thinks you did.

    Everything else in this file measures. This one CHECKS — it is the only place the
    calibration gets to be wrong out loud, before a motor is ever commanded from it.
    """
    calib = read_calib(path)
    todo = [j for j in joints if j in calib and calib[j].get("zero") is not None]
    if not todo:
        print(f"\n  !! nothing calibrated in {path} yet — run the main pass first.")
        return None

    print("\n" + "=" * 72)
    print("  VERIFY — does the calibration read back what you actually did?")
    print("=" * 72)
    print("""
  Pose the crouch again. For each joint the script says what it BELIEVES you are at;
  then you move that joint the sim's + way and it says what it believes now. If the
  numbers disagree with your hands, the calibration is wrong — not your hands.
""")
    input("  press Enter when the robot is HELD in the default crouch... ")
    time.sleep(SETTLE_S)
    snap = settle(fleet)

    print(f"\n  {'joint':17s} {'reads':>9s} {'expected':>9s} {'error':>9s}   at the crouch")
    bad = []
    for j in todo:
        c = calib[j]
        raw = snap.get(c["motor"])
        if raw is None:
            print(f"  {j:17s}        --  no reading")
            continue
        q, want = raw_to_sim(c, raw), DEFAULT_JOINT_POS[j]
        err = q - want
        mark = "  <-- OFF" if abs(err) > 0.15 else ""
        if mark:
            bad.append(j)
        print(f"  {j:17s} {q:+9.3f} {want:+9.3f} {err:+9.3f}{mark}")
    if bad:
        print(f"\n  ** {', '.join(bad)} read >0.15 rad from the crouch you posed.")
        print("     Either the pose drifted, or that joint's zero is wrong. If it's ~2x the")
        print("     default, suspect the SIGN.")

    print("\n  Now the direction check — the one that catches a flipped sign:")
    for j in todo:
        c = calib[j]
        print("\n  " + "-" * 68)
        print(f"  {j}: {PLUS_MEANS[j]}")
        input(f"    Nudge {j} the sim's + way, hold, Enter... ")
        time.sleep(SETTLE_S)
        raw = settle(fleet).get(c["motor"])
        if raw is None:
            print("    !! no reading")
            continue
        q = raw_to_sim(c, raw)
        d = q - raw_to_sim(c, snap[c["motor"]]) if snap.get(c["motor"]) is not None else None
        if d is None:
            continue
        ok = d > 0
        print(f"    calibrated reading moved {d:+.3f} rad  ->  "
              f"{'OK, + means +' if ok else '** WRONG WAY: you moved + and it read NEGATIVE'}")
        if not ok:
            print(f"       That is a flipped sign. Re-run: python3 calibrate_joints.py --only {j}")
    return True


# ---------------------------------------------------------------------------
def home(fleet, joints, path="joint_calib.py"):
    """Rebuild the perishable half (zero, raw_lo, raw_hi) from the durable half (sim_hi).

    The startup ritual for a boot-relative motor. Push each joint to ONE hard stop — the
    same stop whose sim angle we measured during the full run — read the raw value there,
    and back-solve the encoder origin:

        zero = raw_at_stop - sign * to_native(sim_hi)

    No posing by eye: the stop is metal and lands in the same place every time. The stop
    used is the one in the sim's + direction, which is where sim_hi came from.
    """
    calib = read_calib(path)
    if not calib:
        print(f"\n  !! no {path} — nothing to home against. Do a full run first:")
        print("     python3 calibrate_joints.py")
        return None

    todo = [j for j in joints if j in calib]
    unmeasured = [j for j in todo if calib[j].get("sim_hi") is None]
    if unmeasured:
        print(f"\n  !! no measured sim_hi for: {', '.join(unmeasured)}")
        print(f"     Homing needs the durable half first. Re-run: "
              f"python3 calibrate_joints.py --only {' '.join(unmeasured)}")
        todo = [j for j in todo if j not in unmeasured]
    if not todo:
        return None

    print("\n" + "=" * 72)
    print("  HOMING — re-establishing each zero against a mechanical hard stop")
    print("=" * 72)
    print("""
  One joint at a time. GENTLY to the stop — you are looking for the metal, not
  forcing it. The motors are LIMP; nothing is ever commanded.
""")
    out = {}
    for j in todo:
        c = calib[j]
        motor, sim_hi = c["motor"], c["sim_hi"]
        print("\n  " + "-" * 68)
        print(f"  {j}  (motor '{motor}')")
        print(f"  The + direction is the one where: {PLUS_MEANS[j]}")
        input(f"    Move {j} GENTLY to its hard stop in that + direction, hold, Enter... ")
        time.sleep(SETTLE_S)
        raw = settle(fleet).get(motor)
        if raw is None:
            print(f"    !! no reading from '{motor}' — zero NOT re-established for {j}")
            continue

        zero = raw - c["sign"] * to_native_units(motor, sim_hi)
        upd = {"zero": zero}
        # raw_lo/raw_hi are perishable too: they are the same metal, re-expressed against
        # the new origin. Recomputing them here keeps the file self-consistent — leaving
        # the old ones next to a fresh zero would describe stops that aren't there.
        lo = sim_to_raw({**c, "zero": zero}, c["sim_lo"])
        hi = sim_to_raw({**c, "zero": zero}, sim_hi)
        upd["raw_lo"], upd["raw_hi"] = min(lo, hi), max(lo, hi)
        out[j] = upd

        drift = zero - c["zero"] if c.get("zero") is not None else None
        d = "  (first home)" if drift is None else f"  drift vs stored: {drift:+.4f}"
        print(f"    raw at stop={raw:+.4f}  sim_hi={sim_hi:+.3f} rad  -> zero={zero:+.4f}{d}")

    if out:
        write_calib(out, path)
        print(f"\n  RE-ZEROED ({len(out)}/{len(todo)}): {', '.join(out)}")
        print("  The durable half (motor/sign/sim_lo/sim_hi) was left untouched.")
    return out


# ---------------------------------------------------------------------------
def check_persistence(fleet):
    """Is a stored zero worth anything? Decides the whole calibration design.

    A stored `zero` (and the stored raw_lo/raw_hi) are only meaningful if the motor's
    RAW reading means the same thing after a power cycle. Two possibilities:

      ABSOLUTE      the encoder reports a real shaft angle relative to a zero held in
                    the motor -> calibrate once, reuse forever.
      BOOT-RELATIVE the drive zeroes its position estimate at startup -> a stored zero
                    is GARBAGE after every power cycle, and the robot must be re-zeroed
                    (re-posed, or homed against a hard stop) at every startup.

    Damiao is probably absolute (magnetic encoder + a hardware zero written into the
    motor). ODrive is the doubt: on the first scan three of the four drives read
    ~0.0000 turns, which is what booting-at-zero looks like.

    The only way to know is to power-cycle and look. This does exactly that, and
    nothing else. It commands no motion.
    """
    print("\n" + "=" * 72)
    print("  PERSISTENCE TEST — is a stored zero still true after a power cycle?")
    print("=" * 72)
    print("""
  This decides whether calibration is a one-time job or a startup ritual.

  DO NOT MOVE THE ROBOT during this test. Not at all — any motion is
  indistinguishable from a drifting zero and will give a false answer.
""")
    input("  Hold the robot still, then press Enter to record 'before'... ")
    before = settle(fleet, n=12)
    print(f"\n  before: " + "  ".join(f"{k}={v:+.4f}" for k, v in sorted(before.items())))

    print("""
  Now, WITHOUT touching the robot:
    1. power-cycle the motors (both Damiao and ODrive supplies),
    2. wait for the ODrives to finish booting (a few seconds),
    3. press Enter.
""")
    input("  press Enter once the motors are back up... ")
    time.sleep(2.0)
    after = settle(fleet, n=12)
    print(f"  after:  " + "  ".join(f"{k}={v:+.4f}" for k, v in sorted(after.items())))

    print("\n  " + "-" * 68)
    print(f"  {'motor':17s} {'before':>10s} {'after':>10s} {'drift':>10s}   verdict")
    verdict = {}
    for k in sorted(before):
        if k not in after:
            print(f"  {k:17s} {before[k]:+10.4f} {'--':>10s}      --      went OFFLINE")
            continue
        d = after[k] - before[k]
        if abs(d) < 0.01:
            v = "ABSOLUTE — stored zero survives"
        elif abs(after[k]) < 0.01:
            v = "BOOT-RELATIVE — reset to 0, must re-zero every boot"
        else:
            v = "DRIFTED — stored zero NOT reusable"
        verdict[k] = v
        print(f"  {k:17s} {before[k]:+10.4f} {after[k]:+10.4f} {d:+10.4f}   {v}")

    absolute = [k for k, v in verdict.items() if v.startswith("ABSOLUTE")]
    print("\n  " + "=" * 68)
    if len(absolute) == len(verdict) and verdict:
        print("  RESULT: every motor is ABSOLUTE. Calibrate once; joint_calib.py stays valid.")
    elif not absolute:
        print("  RESULT: NOTHING is absolute. The zero must be re-established at EVERY")
        print("          startup — pose-and-capture, or home against a hard stop. A stored")
        print("          zero would silently be wrong after any power cycle.")
    else:
        print(f"  RESULT: MIXED. Absolute: {', '.join(absolute)}.")
        print("          The rest must be re-zeroed at every startup. Do NOT assume the")
        print("          stored zero applies to them.")
    print("  Also note: the same applies to raw_lo/raw_hi — a boot-relative motor's")
    print("  measured limits shift with its zero and cannot be stored either.")
    return verdict


def read_calib(path="joint_calib.py"):
    """Load an existing joint_calib.py, or {} if there isn't one. Exec'd rather than
    imported so --out can point anywhere and a stale sys.modules entry can't win."""
    ns: dict = {}
    try:
        with open(path) as f:
            exec(compile(f.read(), path, "exec"), ns)
    except FileNotFoundError:
        return {}
    except Exception as e:
        print(f"  !! {path} exists but won't parse ({e}); treating as empty.")
        return {}
    return dict(ns.get("JOINT_CALIB", {}))


def write_calib(calib, path="joint_calib.py"):
    """MERGE calib into whatever is already on disk, then rewrite the file.

    Merging is the whole point of --only and --home: both hand us a PARTIAL dict, and a
    plain overwrite would silently delete every joint they didn't touch.
    """
    merged = read_calib(path)
    for j, c in calib.items():
        merged.setdefault(j, {}).update(c)
    calib = merged
    lines = [
        '"""',
        "joint_calib.py — MEASURED per-joint calibration. Generated by calibrate_joints.py.",
        "",
        "DO NOT hand-edit: re-run `python3 calibrate_joints.py` (or --only <joint>).",
        "",
        "DURABLE — mechanical facts. Survive a power cycle; valid until the robot is rebuilt.",
        "  motor  : the fleet_config motor that actually drives this joint (measured, not assumed)",
        "  sign   : +1 if the motor reading increases in the sim's +angle direction, else -1",
        "  sim_lo/sim_hi : the mechanical hard stops, in SIM radians",
        "",
        "PERISHABLE — encoder-origin-relative. A boot-relative motor (the ODrives zero their",
        "position estimate at power-up) invalidates these on EVERY power cycle. Re-establish",
        "them with `python3 calibrate_joints.py --home` before trusting a reading.",
        "  zero   : the motor's raw reading corresponding to the sim's 0 rad",
        "  raw_lo/raw_hi : the same stops, in the motor's NATIVE units",
        "",
        "Conversion, both directions:",
        "    raw    = zero + sign * to_native(q_sim)",
        "    q_sim  = to_rad(raw - zero) * sign",
        '"""',
        "",
        "JOINT_CALIB = {",
    ]
    def num(c, k):
        v = c.get(k)
        return "None" if v is None else f"{v:+.6f}"

    for j in POLICY_JOINT_ORDER:
        if j not in calib:
            continue
        c = calib[j]
        lines.append(f'    "{j}": dict(motor="{c["motor"]}", sign={c["sign"]:+d},')
        lines.append(f'        sim_lo={num(c, "sim_lo")}, sim_hi={num(c, "sim_hi")},')
        lines.append(f'        zero={num(c, "zero")}, raw_lo={num(c, "raw_lo")}, '
                     f'raw_hi={num(c, "raw_hi")}),')
    lines.append("}")
    open(path, "w").write("\n".join(lines) + "\n")
    print(f"\n  wrote {path}")


def report(calib):
    print("\n" + "=" * 72)
    print("  SUMMARY")
    print("=" * 72)
    print(f"  {'joint':17s} {'motor':17s} {'sign':>5s}  {'zero':>10s}  {'stops (sim rad)':>18s}")
    for j in POLICY_JOINT_ORDER:
        if j not in calib:
            continue
        c = calib[j]
        z = "  --" if c["zero"] is None else f"{c['zero']:+10.4f}"
        s = ("        --" if c.get("sim_lo") is None
             else f"[{c['sim_lo']:+.3f}, {c['sim_hi']:+.3f}]")
        flag = "  <-- MISMATCH" if c["motor"] != j else ""
        print(f"  {j:17s} {c['motor']:17s} {c['sign']:+5d}  {z}  {s:>18s}{flag}")

    # The default crouch is a pose the robot MUST be able to hold — the policy starts
    # there. If it lands outside the measured stops, something upstream is wrong (a flipped
    # sign, a mislabelled motor, or a sloppy zero pose) and the policy would command the
    # joint straight into the metal. Cheap check, expensive failure.
    for j in POLICY_JOINT_ORDER:
        c = calib.get(j)
        if not c or c.get("sim_lo") is None:
            continue
        d = DEFAULT_JOINT_POS[j]
        if not (c["sim_lo"] <= d <= c["sim_hi"]):
            print(f"\n  ** {j}: the sim's default {d:+.2f} rad is OUTSIDE the measured stops "
                  f"[{c['sim_lo']:+.3f}, {c['sim_hi']:+.3f}].")
            print("     The robot cannot reach its own start pose — so a number here is wrong.")
            print("     Suspect the sign or the zero pose. Re-run --only for this joint.")

    inverted = [j for j, c in calib.items() if c["sign"] < 0]
    print(f"\n  INVERTED joints ({len(inverted)}): {', '.join(inverted) or 'none'}")
    for base in ("hip", "upper_leg", "lower_leg", "foot"):
        l, r = f"{base}_left", f"{base}_right"
        if l in calib and r in calib:
            if calib[l]["sign"] != calib[r]["sign"]:
                print(f"    * {base}: left and right have OPPOSITE signs -> the pair is "
                      f"MIRRORED on hardware, exactly as the measured limits suggested.")
    mism = [j for j, c in calib.items() if c["motor"] != j]
    if mism:
        print(f"\n  ** fleet_config's motor assignment is WRONG for: {', '.join(mism)}")
        print("     Fix OD_JOINTS/DM_JOINTS names to match, or the calib's 'motor' field wins.")

    # Say the quiet part out loud. A joint that got skipped mid-run is simply absent from
    # the file, and an absent joint is easy to not notice until the policy commands it.
    gone = [j for j in POLICY_JOINT_ORDER if j not in calib]
    if gone:
        print(f"\n  ** NOT CALIBRATED ({len(gone)}/8): {', '.join(gone)}")
        print(f"     Re-run: python3 calibrate_joints.py --only {' '.join(gone)}")
    nolim = [j for j in POLICY_JOINT_ORDER
             if j in calib and calib[j].get("sim_lo") is None]
    if nolim:
        print(f"\n  no measured stops yet ({len(nolim)}/8): {', '.join(nolim)}")
        print("     -> position clamps and --home need them: calibrate_joints.py --limits")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--only", nargs="*", help="calibrate just these joints")
    ap.add_argument("--dry-run", action="store_true", help="no hardware; preview the prompts")
    ap.add_argument("--check-persistence", action="store_true",
                    help="power-cycle test: is a stored zero valid at all? RUN THIS FIRST")
    ap.add_argument("--home", action="store_true",
                    help="startup re-zero: push each joint to its hard stop; rebuilds zero "
                         "from the stored sim_hi. Needs --limits to have run once.")
    ap.add_argument("--limits", action="store_true",
                    help="opt-in second pass: walk both hard stops per joint -> sim_lo/sim_hi. "
                         "Needed for position clamps and for --home.")
    ap.add_argument("--verify", action="store_true",
                    help="read-only check: move a joint by hand, see what the CALIBRATED "
                         "layer thinks you did. Catches a flipped sign.")
    ap.add_argument("--out", default="joint_calib.py")
    args = ap.parse_args()

    joints = args.only or list(POLICY_JOINT_ORDER)
    for j in joints:
        if j not in POLICY_JOINT_ORDER:
            sys.exit(f"unknown joint {j!r}; expected one of {list(POLICY_JOINT_ORDER)}")

    print(__doc__.split("WHAT IT SOLVES")[0])
    print("  ROBOT ON THE GANTRY. Motors stay LIMP — nothing is ever commanded.")
    print(f"  Buses: Damiao={DM_CHANNEL}  ODrive={OD_CHANNEL}")

    fleet = Fleet(dry=args.dry_run)
    try:
        on = fleet.online()
        print(f"\n  ONLINE ({len(on)}/8): {', '.join(on) if on else 'NOTHING'}")
        missing = [j for j in POLICY_JOINT_ORDER if j not in on]
        if missing and not args.dry_run:
            print(f"  OFFLINE: {', '.join(missing)}")
            print("  -> powered? right bus? `python3 read_motor_state.py` first.")
            if input("  continue anyway? [y/N] ").strip().lower() != "y":
                return 1

        if args.check_persistence:
            check_persistence(fleet)
            return 0

        if args.home:
            return 0 if home(fleet, joints, args.out) else 1
        if args.limits:
            return 0 if measure_limits(fleet, joints, args.out) else 1
        if args.verify:
            return 0 if verify(fleet, joints, args.out) else 1

        # The main pass: one pose, then one nudge per joint.
        snap = pose_home(fleet)
        calib = identify(fleet, joints, snap)
        if calib:
            write_calib(calib, args.out)
            report(read_calib(args.out))
            print("\n  NEXT:")
            print(f"    python3 calibrate_joints.py --verify   # prove the signs before "
                  f"anything is commanded")
            print(f"    python3 calibrate_joints.py --limits   # hard stops -> position "
                  f"clamps, and --home")
            print("  Re-run --only <joint> to redo one; writes merge, so the rest survive.")
    except KeyboardInterrupt:
        print("\n  aborted — nothing written.")
        return 1
    finally:
        fleet.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
