# Locating the IMU on the robot — `imu_locate.py`

The Nicla Sense ME is mounted on `body`, and two places need its pose:

- **training:** the model's `imu_link`: in `dodo_assets`, the URDF's `imu_link` + fixed
  `imu_joint` and the MJCF's `imu_link` body;
- **deployment:** `AXIS_REMAP` in `imu_protocol.py`, which rotates every reading into the
  body frame before the policy sees it.

`imu_locate.py` measures the pose once, stores it in one record, `imu_mount.json` in the
`dodo_assets` checkout next to this repo (`assets.py`), and writes it into all of them,
plus the IMU rows of `dodo_assets`' README. Then it checks that they agree and that the
policy sees the right thing when you tilt the robot. Since 2026-11-05 the model files
live only in `dodo_assets`; before that they were in this repo's `description/`.

| | How | Why this way |
|---|---|---|
| **Orientation** | The IMU measures it: three resting poses of the robot. | At rest the accelerometer reads +1 g along whichever chip axis points up, so each pose shows where one body axis lies in chip terms. |
| **Position** | You measure it: four ruler readings. | A single IMU can't find its own lever arm without a rotation rig. The policy doesn't need it precisely: angular velocity and the gravity direction are the same everywhere on a rigid body, so only raw accelerometer readings depend on it. ±5 mm is plenty. |

Status (2026-11-04): run on the robot on 2026-11-02, snapped. Quality: level vs z axis
**4.44°**, snapped vs exact **4.73°**, cross-check 0.43° (all kept in the record). The
first two say the board sits ~4.4° off square on its mount (it looks tilted, too), and
a snapped remap hands that tilt to the policy: a level robot would read ~3.6° nose-down
and ~2.6° right-side-down. The record is now `refit --exact`. The first live check with
it (2026-11-04) still read 3.8° nose-down: that day's phone-levelled pose and 11-02's
differ by 3.8° in pitch (roll agrees), so the pitch is open. Settle it with
`imu_locate.py level` while levelling carefully (HANDOFF §2 step 1).

---

## Before you go

**On the robot PC** (`/home/dodo/Documents/dodo/…`, branch `merge-zmq-dev`):
```bash
git pull
python3 -m pytest test_imu_locate.py -q          # all pass, no hardware needed
```

**Or on a Mac.** `imu_locate.py` only needs the IMU's serial port, the files and MuJoCo,
so any machine with the repo works: move the Nicla's USB cable from the robot PC to the
Mac (make sure it's long enough for the robot to lie on its front and side). Checked on
2026-11-02: the full suite and a complete `measure` run against a fake Nicla on a macOS
pseudo-terminal pass.
```bash
git switch merge-zmq-dev && git pull
python3.10 -m venv .venv && .venv/bin/pip install -r requirements.txt   # once
.venv/bin/python -m pytest test_imu_locate.py -q
ls /dev/cu.usbmodem*        # the Nicla. Use cu.*, not tty.* (tty.* blocks on open)
```
Then run every command below as `.venv/bin/python …` instead of `python3 …`, with
`--port /dev/cu.usbmodemXXXX` added (the default port, `/dev/ttyACM0`, only exists on
Linux). Commit and push from the Mac; the robot PC just pulls later.

**Bring:**
- a steel rule (a caliper with a depth rod is nicer);
- something with a right angle: a square, or a sturdy box;
- a phone with a spirit-level app. Use the **top plate** as the reference: in the CAD
  it is exactly parallel to the body frame (0.00° both ways). A phone on its back sits
  on its camera bump and case, which can tilt it by a degree or more, so read it, turn
  it 180° in place, read again, and use the average;
- a towel or foam to lay the robot on;
- paper for the four numbers (the record keeps them too).

**Robot state:**
- **Motor power off.** You will turn the robot onto its front and its side. Nothing may
  be enabled, so don't run `motor_service`.
- **Legs hanging straight down, not splayed** (hip roll ≈ 0). The position is measured
  from the hip-pitch axis, and that axis only sits at its URDF place with hip roll at 0.
- IMU on USB to the machine you run from. On the robot PC, **stop `imu_service.py`**: it
  holds the serial port.

---

## The session (about 20 minutes)

### 0. Is the IMU itself sane?
```bash
python3 imu_check.py                             # must print RESULT: PASS
```

### 1. Measure
```bash
python3 imu_locate.py measure                    # --port /dev/ttyACM1 if it isn't ACM0
python3 imu_locate.py measure --note "steel rule, chip visible"   # optional free text
```
It runs four stages in one go: orientation, position, write, and the checks.

#### 1a. Orientation: three poses

For each pose: put the robot there, let it settle, press Enter. It averages ~1 s of
readings and redoes the pose if the robot was moving (> 8 °/s).

| Pose | Put the robot… | Pins | Accuracy needed |
|---|---|---|---|
| 1 LEVEL | upright, body level, as if standing on flat ground | body z (up) | **the one that matters.** Rest the body on the table or equal blocks, phone level on a flat body face parallel to the top plate, and shim until it reads ~0° both ways. |
| 2 NOSE-DOWN | pitched forward ~90°: front face toward the floor (lying on its front is ideal) | body x (forward) | ~10° is fine; only a sideways lean matters |
| 3 LEFT-SIDE-DOWN | rolled ~90°: **the robot's own left** side toward the floor (lying on its left side) | body y (left), and a cross-check | ~10° is fine |

"Left" means the robot's left: stand behind it, looking where it looks. Doing pose 2
nose-*up* or pose 3 *right*-side-down is the classic mistake. The tool catches it,
because then poses 2 and 3 disagree about which way is forward by ~180°.

What you'll see (a rehearsal with a simulated board):
```
  RESULT (nearest square mounting):
    chip +x -> right, chip +y -> forward, chip +z -> up
    AXIS_REMAP = (( 0,  1,  0),
                  (-1,  0,  0),
                  ( 0,  0,  1))
    imu_joint rpy = 0 0 -1.570796 rad
  Quality:
    level reading vs the result's z axis    1.1 deg   board tilt on its mount + body not quite level
    snapped vs exact fit                    1.2 deg   < 10: the board is square to the body
    nose-down vs left-side-down heading     0.1 deg   < 20: the tilted poses agree
```

**Sanity-check the first line with your eyes.** "chip +x -> right" should match how
the board sits, if you know the BHI260AP's axes on the Nicla (Arduino's Nicla Sense ME
documentation shows them). Either way, the live check in 1e proves it.

The result is **snapped to the nearest square mounting** (one of the 24 ways to put a
board flat or on edge, square to the body). A board screwed to a plate is square to
within a degree or two; hand-held poses are not that good. How to read the quality lines:

| Line | Typical | Meaning if large |
|---|---|---|
| level vs z axis | < 3° | The body wasn't level (check with the phone), or the board is tilted on its mount. If the body WAS level, snapping hands this tilt to the policy as a standing error: use `--exact` (or `refit --exact` afterwards). The live check's level step tells the two apart. |
| snapped vs exact | < 5° | > 10° and the tool refuses: either a pose was far off (redo it lying on the table), or the board **really is mounted at an angle**. Look at it. If it is, re-run with `--exact`, which keeps the fitted rotation instead of snapping. |
| nose-down vs left-side-down | < 10° | > 20° and the tool refuses: ~180° means a pose went the wrong way; anything else means a sloppy pose. |

#### 1b. Position: four numbers, in mm

Measure to the **centre of the IMU chip** (the BHI260AP). If the board is mounted
chip-side down, use the board centre; at most ~1 cm off, which is harmless (see the
table at the top). Every distance runs **along one body axis**, not straight across.

The reference is the **hip-pitch axis**: the line the thighs swing about, i.e. the
centre of the hip-pitch (`upper_leg`) motors' shafts, on either side. The body frame's
origin lies on it, midway between the legs. The tool takes its exact place from the URDF
(x = −0.75 mm, z = 0).

```
SIDE VIEW, standing at the robot's RIGHT side (nose pointing right →)

                                 ● chip
                                 ┆
                                 ┆  2. up         (+ if the chip is above the axis)
                                 ┆
     hip-pitch axis  ⊗┄┄┄┄┄┄┄┄┄┄┄┘
     (thigh pivot)   ┃   1. forward              (+ if the chip is in front of it)
                     ┃
                     ┃ thigh


TOP VIEW (nose pointing up ↑)

     a feature on the                                   its mirror image on the
     LEFT side      |←─ 3. left ─→ ● ←──── 4. right ────→|  RIGHT side
                                  chip
     y = (right − left) / 2         (+ = chip left of the centreline)
```

The prompts take arithmetic, so type two readings off the same datum and the tool does
the subtraction:

| # | Prompt | Easiest way to measure it |
|---|---|---|
| 1 | chip IN FRONT of the hip-pitch axis | Stand the square against the robot's nose, perpendicular to the table. Read the distance from it to the axis (motor shaft centre) and to the chip: type `axis - chip`, e.g. `95-60` = 35 mm in front. |
| 2 | chip ABOVE the hip-pitch axis | Robot level on the table (pose 1). Read the height of the chip and of the axis above the table: type `chip - axis`, e.g. `142-31`. |
| 3 | chip to a feature on the robot's LEFT side | Any feature that has a mirror image on the right: the outer faces of the two hip-pitch motors, or the body's side plates. Measure parallel to body y. |
| 4 | chip to the mirror-image feature on the RIGHT side | The same feature's twin. You never need to find the centreline. |

The tool then prints the body-frame position and `left + right`, the distance between
your two features. Compare that with the robot as a check. If the point lands more than
2 cm outside the body's CAD outline it warns you. That almost always means a sign mix-up
(in front/behind, above/below, left/right).

#### 1c. Write
Answer `Y` and it writes:

| File | What changes |
|---|---|
| `dodo_assets/imu_mount.json` | the record: rotation, position, mass, and the raw readings behind them |
| `dodo_assets/urdf/dodo_daimao.urdf` | an `imu_link` (5 g) + fixed `imu_joint`, between `BEGIN/END imu_locate.py` markers |
| `dodo_assets/mjcf/dodo_daimao.xml` | the `imu_link` body (site `imu`, which the MJCF's IMU sensors use), between the same markers |
| `imu_protocol.py` (this repo) | `AXIS_REMAP` |
| `dodo_assets/README.md` | the two rows of the IMU table |

#### 1d. Static check (automatic)
```
  [PASS] RECORD     chip +x -> right, chip +y -> forward, chip +z -> up; xyz = (+29.2, +20.0, +110.0) mm
  [PASS] URDF       imu_joint: fixed, body -> imu_link, pose = record
  [PASS] AXIS_REMAP imu_protocol.py = record
  [PASS] MUJOCO     imu_link pos/quat = record
  [PASS] MJCF       imu_link body: child of body, pose = record
  [PASS] SIM        sim_env builds its floating-base model from this URDF
```
MUJOCO loads the URDF the way a trainer would and compares what MuJoCo makes of
`imu_joint` with the record; MJCF does the same for the MJCF. `dodo_assets`'
`scripts/check_models.py` then compares the two models with each other in full. SIM builds the floating-base model `sim_env` and
`sim_service` use.

#### 1e. Live check (automatic, the real acceptance test)
Three moves, read through the deployment code itself (`imu_protocol` → `imu_service`'s
scales → `ZmqEnv`'s projected gravity):

| Move | PASS when |
|---|---|
| 1. robot level (spirit level ~0° both ways), still | `projected_gravity` within 2° of (0, 0, −1). It prints the tilt the policy would see, split into nose-down and right-side-down. |
| 2. from level, tilt nose-down ~45° and hold | gravity x > +0.5, and the gyro saw a turn mostly about **+y** |
| 3. from level, tilt left-side-down ~45° and hold | gravity y > +0.5, and the turn was mostly about **−x** |

Those are the signs Isaac Lab uses (body frame, x forward, y left, z up). If the live
check passes, the policy gets the right `base_ang_vel` and `projected_gravity` from this
IMU.

### 2. Optional cross-checks
```bash
python3 imu_check.py --level       # robot level: LEVEL must PASS now
python3 -m pytest -q               # 140 passed, 1 known failure, 1 skip (as before + the new tests)
```

### 3. Commit, push, hand over
Two repos: the model files go to `dodo_assets` (a PR to Atarilab), `AXIS_REMAP` here.
```bash
(cd ../dodo_assets && python3 scripts/check_models.py && git switch -c imu-remeasure \
  && git commit -am "IMU pose re-measured on the robot" && git push -u origin HEAD)
git add imu_protocol.py && git commit -m "IMU pose re-measured on the robot" && git push
```
- Training uses `dodo_assets` directly, so Shafeef gets it with the PR (its README's
  IMU section says how to use `imu_link`).
- Per the working agreement, update `HANDOFF.md`: tick off §2 step 1 and add a log line
  with the three quality numbers and anything odd.

---

## Redoing part of it

| Situation | Command |
|---|---|
| Orientation looks wrong, the position is fine | `python3 imu_locate.py measure --skip-position` |
| Position wrong, orientation fine (no IMU needed) | `python3 imu_locate.py measure --skip-orientation` |
| You already know the position (body frame, mm) | `python3 imu_locate.py measure --xyz-mm 30 -2 110` (combine with `--skip-orientation`) |
| Board really is mounted at an angle | `python3 imu_locate.py measure --exact` |
| Same, but the poses you already took were fine | `python3 imu_locate.py refit --exact`: re-solves from the readings stored in the record, no IMU needed. `refit` without `--exact` snaps again. Then `check --live`. |
| You edited the record by hand (e.g. a weighed mass) | `python3 imu_locate.py apply` |
| Just verify (any machine) | `python3 imu_locate.py check` |
| Verify against the robot again (e.g. after remounting) | `python3 imu_locate.py check --live` |
| Level the robot, or see what the policy reads right now | `python3 imu_locate.py level`: a live tilt readout (nose-down / right-down), Ctrl-C to stop. A phone level has its own bias: read it, turn the phone 180°, read again, use the average. |
| Only see the remap, write nothing | `python3 imu_sniff.py --axes` |

**Don't hand-edit** `AXIS_REMAP` or the generated URDF block: `check` flags any
difference from the record, and the next `apply` overwrites it. If the board is moved,
measure again.

---

## Troubleshooting

| Symptom | Cause → fix |
|---|---|
| `no IMU response on /dev/ttyACM0` | `imu_service` or a serial monitor holds the port, or it's another port: stop it; `ls /dev/ttyACM*` (Linux) or `ls /dev/cu.usbmodem*` (Mac); `--port`. |
| "it was moving" over and over | Rest the robot on the table instead of holding it. |
| "… readings are N deg apart; they should be ~90" | A pose was skipped, or the robot wasn't turned far enough. |
| "disagree by ~180 deg about which way is forward" | Nose-*up* instead of nose-down, or *right* side down instead of left. Answer Y to redo. |
| "nearest square mounting is N deg from the measured one" | Sloppy pose (redo lying on the table) or an angled mount (`--exact`). |
| WARNING "… outside the body's CAD outline" | A sign is flipped in the position prompts. Check them against the diagram, then `measure --skip-orientation`. |
| `has a hand-written imu_link/imu_joint` | Someone added one by hand: delete it from the URDF, then `apply`. |
| Static check FAIL on AXIS_REMAP | `imu_protocol.py` was edited or merged by hand: `python3 imu_locate.py apply`. |
| Live check FAIL on "level" | The robot wasn't level (check it with the phone), or the remap is off by the printed tilt. A snapped remap on a board tilted on its mount: `refit --exact`. Otherwise `measure --skip-position`. |
| Live check FAIL on a tilt, "it never got there" | Not tilted far or soon enough (20 s per step): re-run `check --live`. Not a remap problem. |
| Live check FAIL on a tilt, "it went the other way" | You tilted the wrong way (re-run), or the orientation is wrong (`measure --skip-position`). |

---

## How it works (for whoever maintains it)

**One convention, everywhere.** `R` (= `AXIS_REMAP`) has the body axes in chip terms as
rows, so `v_body = R · v_chip`. That is exactly what the URDF joint rotation means
(child `imu_link` → parent `body`). URDF `rpy` is extrinsic x-y-z,
`R = Rz(yaw)·Ry(pitch)·Rx(roll)`. Square mountings often hit gimbal lock (pitch ±90°,
e.g. the chip's x along body z); there the tool sets yaw to 0 and gives roll all of it.
Every conversion is round-tripped before it's written.

**Orientation.** Pose 1 gives body z. Pose 2 gives body x and pose 3 body y, each
projected flat onto the plane perpendicular to z (so neither tilt needs to be exactly
90°). Pose 3's `y × z` is a second estimate of x: the angle between the two is the
cross-check, and their average is the heading. The fit is then snapped to the nearest of
the 24 axis permutations with det +1; `--exact` keeps the fit instead.

**Position.** `x = x_axis + forward`, `z = z_axis + up`, with the hip-pitch axis taken
from the URDF chain `body → hip_left → upper_leg_left` (so it can't go stale), and
`y = (right − left) / 2`. The outline check uses the bounding box of `body.STL`.

**The record is the source of truth.** `measure` writes it and then runs `apply`,
which regenerates the three files from it. `check` compares the record, the URDF, the
`AXIS_REMAP` parsed from `imu_protocol.py`, MuJoCo's import, and `sim_env`'s model. The
URDF block lives between `BEGIN/END imu_locate.py` comments and is replaced in place.
The IMU mass (5 g, a 23 × 23 × 5 mm box) is a guess; weigh it and edit `mass_kg` in
the record, then `apply`.

**What it doesn't touch.** `sim_env` keeps its IMU site on `body` at the origin. That
is correct: the deployment stack delivers body-frame readings (after `AXIS_REMAP`), and
gyro and gravity direction don't depend on where on the body the sensor sits.

### How it was tested
- `test_imu_locate.py` uses a fake Nicla that answers `'L'` like firmware v2 for a board
  at any mounting on a robot in any pose. All 24 square mountings round-trip through
  the solver, the serial line, the files, MuJoCo, and the live check through
  `imu_protocol`. Also covered: nose-up, right-side-down and a skipped pose are
  caught; a board turned 30° refuses to snap and is recovered by `--exact`; a stale
  `AXIS_REMAP` and a hand-written `imu_link` are caught. Breaking the solver's signs,
  the y formula, the URDF rotation or the live check's expected signs makes these tests
  fail.
- Rehearsed on 2026-11-01: the full `measure` CLI against a fake Nicla on a
  pseudo-terminal (`open_imu`, prompts, writes, both checks), plus `--skip-*`,
  `--xyz-mm`, `apply`, `check` and `imu_sniff --axes`. With a measured, gimbal-locked
  mounting written into a copy of the repo, the whole suite still passed (the stack
  accepts a non-identity `AXIS_REMAP` and an `imu_link`).
- 2026-11-04: `refit` (a board 4.4° off square comes back from its stored poses and
  through the files) and the stricter live level step (the snapped remap of such a board
  now FAILS it; with the old `g_z < −0.97` test, ~14°, it passed).
- **Real board:** one run, 2026-11-02 (see Status). The fake can't reproduce noise, the
  CDC first-command drop (`open_imu` handles that), or a robot that won't sit still.
