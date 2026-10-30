# Reference policies

Trained policies the deploy stack is tested against. Each run keeps the rsl_rl layout:
`params/env.yaml` is the fully resolved training config and `exported/policy.onnx` the
network. `test_sim2sim_consistency.py` and `test_e2e_zmq.py` find runs here without the
`dodo_rl` training repo (a sibling `dodo_rl` checkout is still searched if present).

| Run | Policy | Contract |
|---|---|---|
| `dodo_stand/2026-06-30_18-44-06` | Stand and balance (Isaac Lab, rsl_rl PPO). Stands 30 s+ in MuJoCo and 10 s through the ZMQ stack. Never run on the robot. | obs 30: `base_ang_vel`, `projected_gravity`, joint pos − default, joint vel, last action. Action 8: target = default + 0.5 · a. PD kp 32 / kd 3, 50 Hz. |

Source: `cecimaltese/dodo_rl` @ `52b01db`,
`scripts/rsl_rl/logs/rsl_rl/dodo_stand/2026-06-30_18-44-06/`. Checkpoints, logs and the
`.pt` export are left out.

```bash
python3 run_policy.py --backend sim --task stand \
    --policy policies/dodo_stand/2026-06-30_18-44-06/exported/policy.onnx
```
