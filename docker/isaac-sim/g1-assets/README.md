# G1 runtime assets (not stored in Git)

Set `G1_ASSET_DIR` to the complete `assets/` directory produced by the pinned
[`unitreerobotics/unitree_sim_isaaclab`](https://github.com/unitreerobotics/unitree_sim_isaaclab)
`fetch_assets.sh`, or place that directory's contents here.

For the phase-1 mobile G1 29-DOF + dual Dex3 plugin, startup requires at least:

```text
${G1_ASSET_DIR}/
├── model/policy.onnx
├── robots/g1-29dof_wholebody_dex3/
│   └── g1_29dof_with_dex3_rev_1_0.usd
└── objects/
    ├── small_warehouse/small_warehouse_digital_twin.usd
    ├── PackingTable/PackingTable.usd
    └── PackingTable_2/PackingTable.usd
```

The locomotion policy is mandatory. The bridge never falls back to a cube,
kinematic placeholder, sinusoidal gait, or zero-action "success" state.

Launch after the assets are in place:

```bash
ISAAC_ROBOT_TYPE=g1_29dof_dex3 \
G1_ASSET_DIR=/absolute/path/to/unitree_sim_isaaclab/assets \
docker compose -f docker/isaac-sim/docker-compose.yaml up
```

The Unitree DDS channels are isolated to container loopback. ROS 2 remains on
the configured host network and exposes `/cmd_vel_nav`, `/cmd_vel`, the full
`/joint_states`, and explicit `/g1/{body,left_arm,right_arm,left_hand,right_hand}`
joint command/state topics.
