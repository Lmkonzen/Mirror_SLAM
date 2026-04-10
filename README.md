#Sequence to initialize Lidar, RViz, and Slam using RTB

##Initialize Lidar
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
sudo chmod 666 /dev/ttyUSB0
ros2 launch unitree_lidar_ros2 launch.py

##Start SLAM using RTABMAP
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch rtabmap_examples lidar3d_assemble.launch.py \
  lidar_topic:=/unilidar/cloud \
  imu_topic:=/unilidar/imu \
  frame_id:=base_link


#Launch GazeboLidar Sim
source /opt/ros/jazzy/setup.bash
source ~/ws_moveit/install/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch nav2_bringup tb3_simulation_launch.py \
  world:=$HOME/ros2_ws/src/my_nav_sim/worlds/simpleroom.sdf.xacro \
  x_pose:=2.0 \
  y_pose:=0.0 \
  z_pose:=0.1 \
  slam:=True \
  headless:=False

#Launch Arm RVIZ
source /opt/ros/jazzy/setup.bash
source ~/ws_moveit/install/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch moveit2_tutorials demo.launch.py

#Launch Probe Node
source /opt/ros/jazzy/setup.bash
source ~/ws_moveit/install/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run arm_probe_cpp arm_probe_node

#Launch GapDetect
cd ~/ros2_ws
colcon build --packages-select gap_explorer
source /opt/ros/jazzy/setup.bash
source ~/ws_moveit/install/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch gap_explorer gap_explorer.launch.py


###Useful Stuff
ros2 run rviz2 rviz2

ros2 launch rtabmap_launch rtabmap.launch.py 

source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch nav2_bringup tb3_simulation_launch.py \
  headless:=False


ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur3e \
    robot_ip:=10.3.13.64 \
    launch_rviz:=false


ros2 launch ur_moveit_config ur_moveit.launch.py \
    ur_type:=ur3e \
    robot_ip:=10.3.13.64 \
    launch_rviz:=true

