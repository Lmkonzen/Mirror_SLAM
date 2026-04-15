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
| `arm_ur_moveit_config` | Custom MoveIt config with `arm_` TF prefix on all joints/links |
| `mirror_slam_bringup` | Worlds, Nav2 params, unified sim launch file |

### State machine (gap_explorer)
`COLLECT` → `NAV` → `SETTLE` → `FOLLOW` → `PROBE` → `COLLECT`

The robot collects lidar scans, fits wall segments by PCA, navigates to the
nearest unvisited wall, follows it to the endpoint, then triggers the arm probe.
A failed poke stores the wall as a detected mirror and marks it permanently in
the Nav2 costmap so the robot routes around it on future passes.

### TF prefix strategy

The UR3e driver is launched with `tf_prefix:=arm_`, which renames every frame
in the URDF before `robot_state_publisher` sees it. The TF tree becomes:

```
TB3:  map → odom → base_footprint → base_link → base_scan
UR:   arm_world → arm_base_link → arm_base_link_inertia → arm_shoulder_link → … → arm_tool0
```

No collision. The `arm_ur_moveit_config` package has matching prefixed
joint/link names in the SRDF, joint limits, and controller configs.

---

## Dependencies

```bash
sudo apt install ros-jazzy-nav2-bringup ros-jazzy-turtlebot3-gazebo \
    ros-jazzy-ur-robot-driver ros-jazzy-ur-moveit-config \
    ros-jazzy-moveit ros-jazzy-rtabmap-ros
```

Set your TurtleBot3 model:
```bash
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
```

---

## Build

First-time setup — copy planning configs that don't need modification:
```bash
cp /opt/ros/jazzy/share/ur_moveit_config/config/{ompl_planning.yaml,pilz_cartesian_limits.yaml,pilz_industrial_motion_planner_planning.yaml,chomp_planning.yaml} \
    src/Mirror_SLAM/arm_ur_moveit_config/config/
```

Build all packages:
```bash
cd ~/ros2_ws
colcon build --packages-select gap_explorer_interfaces arm_ur_moveit_config ur3 gap_explorer mirror_slam_bringup
source install/setup.bash
```

---

## Launch — Simulation (5 terminals)

### Terminal 1 — UR3e driver (mock hardware, arm_ prefix)

```bash
cd ~/ros2_ws && source install/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur3e use_mock_hardware:=true robot_ip:=0.0.0.0 \
    launch_rviz:=false tf_prefix:=arm_
```

### Terminal 2 — MoveIt + RViz (custom config)

```bash
cd ~/ros2_ws && source install/setup.bash
ros2 launch arm_ur_moveit_config arm_ur_moveit.launch.py \
    ur_type:=ur3e launch_rviz:=true
```

Wait for `"You can start planning now!"` before continuing.

### Terminal 3 — TB3 sim + Nav2 + SLAM

```bash
cd ~/ros2_ws && source install/setup.bash
ros2 launch nav2_bringup tb3_simulation_launch.py \
    world:=$HOME/ros2_ws/src/Mirror_SLAM/mirror_slam_bringup/worlds/simpleroom.sdf.xacro \
    params_file:=$HOME/ros2_ws/src/Mirror_SLAM/mirror_slam_bringup/params/nav2_params.yaml \
    x_pose:=2.0 y_pose:=0.0 z_pose:=0.1 slam:=True headless:=True
```

Wait for `"Managed nodes are active"`.

### Terminal 4 — Arm probe server

```bash
cd ~/ros2_ws && source install/setup.bash
ros2 run ur3 arm_probe_server --ros-args -p use_sim_time:=true
```

Wait for `"probe_arm action server ready."`.

### Terminal 5 — Gap explorer

```bash
cd ~/ros2_ws && source install/setup.bash
ros2 run gap_explorer gap_explorer --ros-args -p use_sim_time:=true
```

---

## Launch — Simulation (single command)

```bash
cd ~/ros2_ws && source install/setup.bash
ros2 launch mirror_slam_bringup sim_launch.py
```

This brings up in sequence:
1. UR3e driver with `tf_prefix:=arm_` (immediate)
2. MoveIt + RViz via `arm_ur_moveit_config` (5 s delay)
3. TurtleBot3 + Gazebo + Nav2 + SLAM (10 s delay)
4. arm_probe_server (25 s delay)
5. gap_explorer (30 s delay)

---

## Manually trigger a probe (testing)

Useful for verifying the arm moves correctly before running the full system:

```bash
ros2 action send_goal --feedback /probe_arm \
    gap_explorer_interfaces/action/ProbeArm "{}"
```

You should see feedback states: `homing` → `poking` → `returning` → `done`

---

## Useful commands

```bash
# View the full TF tree (saves PDF in current directory)
ros2 run tf2_tools view_frames

# Check all ros2_control controllers are active
ros2 control list_controllers

# Confirm Nav2 is running
ros2 action list | grep navigate

# Watch mirror detections live
ros2 topic echo /detected_mirrors

# Check the probe action server is advertised
ros2 action list | grep probe_arm
```
