"""
run_policy.py — the Env-agnostic policy runner (the sim-to-real substitution point).

The SAME loop drives MuJoCo (verification) and the real robot (deployment); only
`--backend` changes:

    # verify a policy in MuJoCo (sim-to-sim)
    python3 run_policy.py --backend sim  --task flat  --policy <path>/policy.onnx \
        --command 0.5 0 0 --steps 500 --render

    # deploy to the real robot (needs MotorController + Observer running)
    python3 run_policy.py --backend real --task stand --policy <path>/policy.onnx

`--task` selects the observation layout (must match how the .onnx was trained):
  flat  = walking (obs 36, includes base_lin_vel + velocity_commands) — SIM ONLY.
  stand = balance (obs 30) — the deployable layout (no base_lin_vel).
"""

from __future__ import annotations

import argparse
import time

import numpy as np

from rl_env import OnnxPolicy, OBS_FLAT, OBS_STAND


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--backend", choices=["sim", "real"], default="sim")
    ap.add_argument("--task", choices=["flat", "stand"], default="stand")
    ap.add_argument("--policy", help="path to exported policy.onnx (not needed with --hold-pose)")
    ap.add_argument("--command", type=float, nargs=3, default=[0.0, 0.0, 0.0],
                    metavar=("VX", "VY", "WZ"), help="velocity command (flat task)")
    ap.add_argument("--steps", type=int, default=1000)
    ap.add_argument("--control-hz", type=float, default=50.0)
    ap.add_argument("--render", action="store_true", help="(sim) open the viewer")
    ap.add_argument("--hold-pose", action="store_true",
                    help="(sim) ignore the policy and just PD-hold the default pose, "
                         "to preview the home stance. Implies --backend sim --render.")
    ap.add_argument("--urdf", default="dodo_files/urdf/dodo_daimao.urdf")
    args = ap.parse_args()

    obs_cfg = OBS_FLAT if args.task == "flat" else OBS_STAND

    # --- Preview mode: hold the default (home) pose, no policy needed ---------
    if args.hold_pose:
        import numpy as np
        from sim_env import SimEnv
        env = SimEnv(urdf_path=args.urdf, obs_cfg=obs_cfg, render=True)
        env.reset()
        print(f"[hold-pose] PD-holding the default pose ({args.steps} steps). "
              f"Close the window to exit.")
        zero = np.zeros(env.action_dim)  # action 0 -> target = default pose
        try:
            for _ in range(args.steps):
                if not env.is_running():
                    break
                env.step(zero)
        except KeyboardInterrupt:
            pass
        finally:
            env.close()
            print("done.")
        import os
        os._exit(0)

    if not args.policy:
        raise SystemExit("--policy is required (unless using --hold-pose).")

    policy = OnnxPolicy(args.policy)
    if policy.obs_dim != obs_cfg.dim:
        raise SystemExit(
            f"policy obs_dim {policy.obs_dim} != {args.task} layout {obs_cfg.dim}. "
            f"Wrong --task for this .onnx?")

    control_dt = 1.0 / args.control_hz
    if args.backend == "sim":
        from sim_env import SimEnv
        env = SimEnv(urdf_path=args.urdf, obs_cfg=obs_cfg,
                     command=tuple(args.command), render=args.render)
    else:
        if args.task == "flat":
            print("WARNING: 'flat' obs needs base_lin_vel, which the real robot "
                  "cannot observe — use a 'stand' policy for real deployment.")
        from real_env import RealEnv
        env = RealEnv(obs_cfg=obs_cfg, command=tuple(args.command), control_dt=control_dt)

    obs = env.reset()
    print(f"[{args.backend}] obs_dim {env.obs_dim}, action_dim {env.action_dim}; "
          f"running {args.steps} steps @ {args.control_hz:.0f} Hz")
    is_running = getattr(env, "is_running", lambda: True)
    try:
        for k in range(args.steps):
            if not is_running():  # viewer window closed
                break
            action = policy(obs)
            obs = env.step(action)
    except KeyboardInterrupt:
        pass
    finally:
        env.close()
        print("done.")

    # The MuJoCo GLFW viewer can segfault during interpreter teardown on Wayland
    # (glfw.terminate called twice). We're done and everything is flushed, so exit
    # hard to skip that crashy global cleanup.
    if args.backend == "sim" and args.render:
        import os
        os._exit(0)


if __name__ == "__main__":
    main()
