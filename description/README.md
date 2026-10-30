# Dodo robot description (`dodo_daimao`)

URDF and meshes for the Dodo biped (8 actuated joints), as currently built.

```
description/
├── urdf/dodo_daimao.urdf   the robot (mesh paths are relative: ../meshes/*.STL)
└── meshes/*.STL            visual meshes (SolidWorks export, metres)
```

**Provenance.** SolidWorks URDF export, taken unchanged from
[`liambansbach/legged-robot_rl_genesis`](https://github.com/liambansbach/legged-robot_rl_genesis)
@ `1cfa733` (`ressources/robots/dodo_daimao`). Every policy trained so far
(`dodo_rl`, Isaac Lab) used the Isaac Sim USD stored next to this file in that repo
(`dodo_rl` carries a byte-identical copy). With this file the
MuJoCo sim2sim suite (`test_sim2sim_consistency.py` in the repo root) gives the
result recorded on the robot PC: 17 passed, 1 failed, 1 skipped. The failure is the
known joint-order regression guard, which this file doesn't affect.

**Changes from upstream (2026-10-30):**
- `imu_link` + fixed `imu_joint`: the mounted IMU (see [IMU](#imu)).
- A `<mujoco><compiler fusestatic="false"/></mujoco>` block. MuJoCo fuses fixed-joint
  links on URDF import by default, which would silently delete `imu_link`. Other
  parsers ignore the block.

## Frames and joints

Base link `body`: x forward, y left, z up. Its origin lies on the hip-pitch axis,
midway between the legs. All joints at 0 means legs straight down, with the sole
bottoms 0.447 m below the base origin. Total mass: 4.717 kg (CAD).

| Joints | Axis | Limits [rad] | Effort [Nm] | Vel [rad/s] | Actuator |
|---|---|---|---|---|---|
| `hip_left`, `hip_right` (hip roll) | +x | −0.35 … 0.35 | 27 | 6 | Damiao DM4340P (CAN, MIT mode) |
| `upper_leg_*` (hip pitch) | +y | −1.57 … 1.57 | 27 | 6 | Damiao DM4340P |
| `lower_leg_*` (knee) | +y | −3.1416 … 1.3963 | 9 | 6 | T-Motor MN4004 KV300 + ODrive, geared |
| `foot_*` (ankle) | +y | −1.05 … 1.57 | 9 | 6 | T-Motor MN4004 KV300 + ODrive, geared |

`foot_sole_left/right` are fixed frames on the feet. Left and right use the same axes
and limits; the URDF does not mirror them.

Signs (right-hand rule; +y points left):
- `hip_* > 0` swings the leg toward the robot's left.
- `upper_leg_* > 0` swings the thigh backward.
- `lower_leg_* < 0` swings the shank forward, so the knee points backward (bird-like).
  The knee range is chosen for this direction.
- `foot_* > 0` pitches the toes down.

The default pose of the existing policies is hip 0, upper_leg 0.2, lower_leg −0.5,
foot 0.3: a crouch with flat feet and the base about 0.44 m high.

## IMU

Arduino Nicla Sense ME (Bosch BHI260AP), rigidly mounted on `body`. The `imu_link` axes
are the sensor's own axes, so a gyro/accelerometer sensor placed on `imu_link` in sim
reads what the real chip reads.

| | |
|---|---|
| Position in `body` frame | _pending: measured on the robot_ |
| Orientation (`imu_joint` rpy) | _pending: measured on the robot_ |
| Gyro | ±1000 °/s |
| Accelerometer | ±4 g |
| Orientation | on-chip accel+gyro fusion quaternion (no magnetometer): roll/pitch are gravity-referenced, yaw drifts |
| Rate | ~60 Hz, polled over USB serial |

The deploy stack rotates readings into the `body` frame with `AXIS_REMAP` in
`imu_protocol.py` (repo root). That matrix must equal the rotation in
`imu_joint`. Angular velocity and gravity direction don't depend on where the IMU
sits on `body`; only accelerometer readings do.

**Observable on the robot:** base angular velocity, gravity direction, joint positions
and velocities. **Not observable:** base linear velocity (no state estimator), base
height, and contact forces.

## Loading

- **MuJoCo / mjlab:** `mujoco.MjSpec.from_file("urdf/dodo_daimao.urdf")`. The URDF root
  has no free joint, so add one with `spec.body("body").add_freejoint()`. Visual meshes
  are discarded by default, so only the collision boxes are loaded. Put a site on
  `spec.body("imu_link")` for gyro/accelerometer sensors.
- **Isaac Lab / other importers that merge fixed links:** take `imu_joint`'s origin from
  the URDF and use it as the IMU sensor's offset on `body`. Regenerate the USD from this
  file rather than reusing `dodo_rl`'s: there, `hip_left` has mass 0 (PhysX falls back
  to about 0.44 kg from its convex hull) and the colliders are full meshes, not the
  URDF's boxes.

## What the existing deploy code expects

The hardware stack in this repo runs a 50 Hz loop that sends joint position
targets. The hips track them with the Damiao on-motor PD; knees and ankles use the
ODrive position loop. Policies so far used:

- action = default pose + 0.5 · a
- PD kp = 32, kd = 3; armature 0.01 on all joints
- observation (30): `base_ang_vel(3)`, `projected_gravity(3)`, joint pos − default (8),
  joint vel (8), last action (8)
- joint order `hip_l, hip_r, upper_leg_l, upper_leg_r, lower_leg_l, lower_leg_r,
  foot_l, foot_r`

Any of this can change. The joint order is a single constant (`rl_env.POLICY_JOINT_ORDER`).

## Not yet verified on hardware

- **Joint limits** are CAD/soft limits, not measured stops. Joint calibration is still
  pending.
- **Knee/ankle transmission ratio** is unconfirmed; a July measurement implied about
  32:1. That makes the 9 Nm / 6 rad/s limits and the rotor inertia for those joints
  assumptions. The drives' own velocity and current limits were set over USB and are
  not recorded.
- **Mass and inertia** are CAD values. Wiring, electronics and the IMU mount were not
  modelled, and the robot has not been weighed.
- **Collision geometry** is coarse boxes. Foot contact happens on the foot box: a
  95 × 34 mm patch whose bottom coincides with the sole, centred 14 mm ahead of the
  ankle axis. The thinner sole box sits inside it, 0.65 mm higher, so on flat ground
  the foot box makes the contact.
