# Homework_4 Instructions

Ferdinando Dionisio, Vittorio Lisi, Giovanni Gabriele Imbimbo, Emanuele Cifelli

## Overview

This guide provides updated instructions for working with the robotics package, focusing on Gazebo simulation, navigation tasks, mapping and localization, and advanced features like vision-based navigation.

---

## Prerequisites

Ensure that you have cloned the repository and built the workspace:

```bash
git clone https://github.com/vitlisi/RL24_HW4.git .
colcon build
source install/setup.bash
```

---

## Visualization and Simulation

### Robot Visualization in RViz

To launch the robot visualization in RViz:

```bash
ros2 launch rl_fra2mo_description display_fra2mo.launch.py
```

### Simulation in Gazebo

To start the simulation in Gazebo:

```bash
ros2 launch rl_fra2mo_description gazebo_fra2mo.launch.py
```

---

## Mapping and Localization Commands

### SLAM for Environment Mapping

To launch SLAM and create a map of the environment:

```bash
ros2 launch rl_fra2mo_description fra2mo_slam.launch.py
```

### Localization with AMCL

To start robot localization using AMCL:

```bash
ros2 launch rl_fra2mo_description fra2mo_amcl.launch.py
```

### Saving the Map

To save a generated map:

```bash
ros2 run nav2_map_server map_saver_cli -f map
```

---

## Navigation

### Autonomous Navigation with AMCL (Nav2)

To enable autonomous navigation using Nav2:

```bash
ros2 launch rl_fra2mo_description fra2mo_navigation.launch.py
```

### Vision-Based Navigation

To enable vision-based navigation:

```bash
ros2 launch rl_fra2mo_description fra2mo_navigation_vision.launch.py
```
remember to also start the AMCL localization node
### Autonomous Exploration and Mapping

To launch autonomous environment exploration:

```bash
ros2 launch rl_fra2mo_description fra2mo_explore.launch.py
```

---

## Manual and Advanced Control

### Manual Robot Control with Keyboard

To manually control the robot via keyboard:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Sending the Robot to the Initial Pose

To send the robot to an initial position:

```bash
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{
  header: {
    frame_id: 'map',
    stamp: {sec: 0, nanosec: 0}
  },
  pose: {
    position: {x: 0.0, y: 0.0, z: 0.1},
    orientation: {x: 0.0, y: 0.0, z: -0.7071, w: 0.7071}
  }
}"
```

### Running follow_waypoints.py

To execute the `follow_waypoints.py` script:

```bash
ros2 run rl_fra2mo_description follow_waypoints.py
```

### Running task.py

To execute the `task.py` script:

```bash
ros2 run rl_fra2mo_description task.py
```

### Recording a Bag

To record a bag with specific data:

```bash
ros2 bag record -o recorded_bag /plan /pose
```

### Visualizing TF Frames

To visualize TF frames:

```bash
ros2 topic echo /tf_static
ros2 run tf2_ros tf2_echo camera_link aruco_marker_frame
```

---

## Basic Commands

### Sending Velocity Commands

To manually send velocity commands:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.2}}"
```

---

## Notes and Tips

1. **Multiple Terminals**: Ensure you run each node or command in a separate terminal after sourcing the workspace (`source install/setup.bash`).
2. **Video Demonstrations**: Videos of key tasks are available:

- `https://youtu.be/_O3qW9CAqX8` - Autonomous navigation with new goals
- `https://youtu.be/pcC26Ym-iqM` - Vision-based navigation

3. **Troubleshooting**: If the simulation or tracking does not work as expected:
   - Verify that all required nodes are running.
   - Check for error messages in each terminal.

With these instructions, you can configure and test advanced simulation tasks, including autonomous navigation, exploration, and vision-based control, as described in the context of Homework_4.
