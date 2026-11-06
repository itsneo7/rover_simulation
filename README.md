# ARES-1 Rover Simulation

ROS 2 Jazzy + Gazebo Harmonic simulation of a 4WS4WD swivel-drive IGVC rover with Nav2 (A* + MPPI Omni), SLAM and EKF.

## Run

```bash
cd ~/ares1_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch ares1_bringup simulation.launch.py
```
