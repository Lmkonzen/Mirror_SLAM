# MirrorSLAM

A ROS2 (Jazzy) system that drives a TurtleBot3 through an unknown space,
follows walls using lidar, and uses a UR3e arm to physically probe surfaces.
If the arm's motion fails (indicating a hard/reflective surface like a mirror),
the location is stored permanently and injected into the Nav2 costmap so the
robot avoids it on future passes.

## Architecture

| Package | Role |
|---|---|
| `gap_explorer` | TurtleBot3 brain — wall detection, Nav2 client, 5-state machine |
| `ur3` | UR3e action server — receives `ProbeArm` goals, executes home→poke→home |
| `gap_explorer_interfaces` | Shared `ProbeArm.action` definition |
| `mirror_slam_bringup` | Nav2 params, world file |
| `arm_ur_moveit_config` | Custom MoveIt config for UR3e |

### State machine (gap_explorer)
`COLLECT` → `NAV` → `SETTLE` → `FOLLOW` → `PROBE` → `COLLECT`

The robot collects lidar scans, fits wall segments by PCA, navigates to the
nearest unvisited wall, follows it to the endpoint, then triggers the arm probe.
A failed poke stores the wall as a detected mirror and marks it permanently in
the Nav2 costmap so the robot routes around it on future passes.

---

## Dependencies

```bash
sudo apt install ros-jazzy-nav2-bringup ros-jazzy-turtlebot3-gazebo \
    ros-jazzy-ur-robot-driver ros-jazzy-moveit
```

Set your TurtleBot3 model:
```bash
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
source ~/.bashrc
```

---

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select \
    gap_explorer_interfaces ur3 gap_explorer \
    mirror_slam_bringup arm_ur_moveit_config
source install/setup.bash
```

---

## Launch — Simulation (5 terminals)

Kill any leftover processes before starting:
```bash
pkill -f ros2; pkill -f gazebo; pkill -f gz
ros2 daemon stop && ros2 daemon start
```

### Terminal 1 — UR3e driver (mock hardware)
```bash
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur3e use_mock_hardware:=true robot_ip:=0.0.0.0 \
    launch_rviz:=false
```

### Terminal 2 — MoveIt + RViz
```bash
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=ur3e launch_rviz:=true
```
Wait for `"You can start planning now!"` before continuing.

### Terminal 3 — Arm probe action server
```bash
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run ur3 probe_server --ros-args -p use_sim_time:=true
```

### Terminal 4 — TurtleBot3 simulation + Nav2 + SLAM
```bash
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch nav2_bringup tb3_simulation_launch.py \
    world:=$HOME/ros2_ws/src/Mirror_SLAM_clean/mirror_slam_bringup/worlds/simpleroom.sdf.xacro \
    params_file:=$HOME/ros2_ws/src/Mirror_SLAM_clean/mirror_slam_bringup/params/nav2_params.yaml \
    x_pose:=2.0 y_pose:=0.0 z_pose:=0.1 slam:=True headless:=True
```

### Terminal 5 — Gap explorer
```bash
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run gap_explorer gap_explorer --ros-args -p use_sim_time:=true
```

---

## Launch — Real Hardware

### Terminal 1 — UR3e driver
```bash
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur3e robot_ip:=10.3.13.64 launch_rviz:=false
```
Start the `external_control` URCap program from the teach pendant.

### Terminal 2 — MoveIt
```bash
ros2 launch ur_moveit_config ur_moveit.launch.py \
    ur_type:=ur3e launch_rviz:=true
```
Wait for `"You can start planning now!"` before continuing.

### Terminal 3 — Arm probe server
```bash
ros2 run ur3 probe_server
```

### Terminal 4 — Unitree L1 Lidar
```bash
sudo chmod 666 /dev/ttyUSB0
ros2 launch unitree_lidar_ros2 launch.py
```

### Terminal 5 — SLAM (RTAB-Map)
```bash
ros2 launch rtabmap_examples lidar3d_assemble.launch.py \
    lidar_topic:=/unilidar/cloud \
    imu_topic:=/unilidar/imu \
    frame_id:=base_link
```

### Terminal 6 — Gap explorer
```bash
ros2 run gap_explorer gap_explorer
```

---

## Manually trigger a probe (testing)

```bash
ros2 action send_goal --feedback /probe_arm \
    gap_explorer_interfaces/action/ProbeArm "{}"
```

Feedback states: `homing` → `poking` → `returning` → `done`

---

## Useful commands

```bash
# Kill everything and reset before a fresh launch
pkill -f ros2; pkill -f gazebo; pkill -f gz
ros2 daemon stop && ros2 daemon start

# Check controllers are active
ros2 control list_controllers

# Confirm Nav2 is running
ros2 action list | grep navigate

# Watch mirror detections live
ros2 topic echo /detected_mirrors

# Check probe action server is up
ros2 action list | grep probe_arm
```

---

## MoveIt & NavStack assignment:
f391ca97a8ce2d8ae10699f42f9265be00e98ad4
