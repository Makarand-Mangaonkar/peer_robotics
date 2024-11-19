# Peer Robotics Assignment Submission 

## Prerequisites

Before using, ensure you have the following dependencies installed:

- ROS2 `Humble` distribution
- Additional packages:
  - `gazebo*`
  - `xacro`
  - `rmw_cyclonedds_cpp`
  - `nav*`

>If you are using **`binary installation`**, you can install these packages using `apt`. 

>If you are using **`source installation`**, make sure you have built your ROS2 Humble source folder, and you'll need the `behaviours_tree_v3` package from the GitHub v3.8 branch. 

#### Build the repository using the following command:

```bash
colcon build --symlink-install
```

>Make sure you have sourced ROS2 Humble in your terminal before building.

## Installation

To get started, clone the repository

## Usage

  - Source your build workspace every time before running any files or nodes:

    ```bash
    source <path_to_your_workspace>/install/setup.bash
    ```

  - To launch the Gazebo simulation and spawn the robot with all plugins:

    ```bash
    ros2 launch cube_bringup gazebo_bringup.launch.py
    ```
    
    ```bash
    ros2 launch testbed_bringup gazebo_bringup.launch.py
    ```

  - For SLAM (Simultaneous Localization and Mapping) functionality:

    ```bash
    ros2 launch cube_navigation slam.launch.py
    ```

  - For launching navigation of robot1:

    ```bash
    ros2 launch cube_navigation navigation.launch.py
    ```

  - For controlling of robot2:

    ```bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/testbed/cmd_vel
    
## Working Video

Please refer the video for the same: /robot_multiple/task1.webm

