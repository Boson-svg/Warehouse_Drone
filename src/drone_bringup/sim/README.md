# Gazebo Vision Sim

This directory contains local Gazebo assets for visual-servo tests:

- `px4_models/iris_opt_flow_mid360`: PX4 model override with a front depth camera.
- `models/vision_target_board`: high-contrast target board model.
- `worlds/warehouse_vision.world`: simple scan scene aligned with the current warehouse mission path.

Typical usage:

```bash
bash "/home/bosen/drone_ws/src/drone_bringup/scripts/install_px4_gazebo_vision_assets.sh"
bash "/home/bosen/drone_ws/src/drone_bringup/scripts/start_px4_gazebo_vision_sim.sh"
```

Then in another terminal:

```bash
source "/opt/ros/humble/setup.bash"
ros2 launch drone_bringup run_qr_vision_gazebo.launch.py
```
