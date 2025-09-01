# jaka_ros2 Package

This project contains the source code for the **Jaka Robot Driver Package** in **ROS2** with **SDK version 2.2.2**. 

## Quick Start
Before installing and building the **jaka_ros2** package, ensure that your system is running **Ubuntu 22.04 (Jammy)** and **ROS 2 Humble** as the middleware framework for compatibility and stability.

## Build Instructions

### 1. Clone the repository

#### a. Clone with HTTPS:
```bash
git clone https://github.com/JAKARobotics/jaka_ros2.git
```

#### b. Clone with SSH:
```bash
git clone git@github.com:JAKARobotics/jaka_ros2.git
```

### 2. Build the packages
```bash
cd <path-to-where-the-repository-is-cloned>/jaka_ros2
colcon build --symlink-install
```

### 3. Setup the terminal environment
```bash
echo "source <path-to-where-the-repository-is-cloned>/jaka_ros2/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```


## Start the JAKA Driver Server and Call Functional Services
### 1. Start the `jaka_driver` server passing the `<robot_ip>` parameter via the ros2 launch command
```bash
ros2 launch jaka_driver robot_start_launch.py ip:=<robot_ip>
```

### 2. Call various functional services with parameters through ros2 service call
    
#### a. Joint motion:
```bash
ros2 service call /jaka_driver/joint_move jaka_msgs/srv/Move "{pose: [0.0, 1.57, -1.57, 1.57, 1.57, 0.0], has_ref: false, ref_joint: [0], mvvelo: 0.5, mvacc: 0.5, mvtime: 0.0, mvradii: 0.0, coord_mode: 0, index: 0}"
```

#### b. Linear motion:
```bash
ros2 service call /jaka_driver/linear_move jaka_msgs/srv/Move "{pose: [111.126, 282.111, 271.55, 3.142, 0.0, -0.698], has_ref: false, ref_joint: [0], mvvelo: 100, mvacc: 100, mvtime: 0.0, mvradii: 0.0, coord_mode: 0, index: 0}"
```

#### c. Forward kinematic solution:
```bash
ros2 service call /jaka_driver/get_fk jaka_msgs/srv/GetFK "{joint: [0, 1.57, -1.57, 1.57, 1.57, 0]}"
```

#### d. Inverse kinematics solution:
```bash
ros2 service call /jaka_driver/get_ik jaka_msgs/srv/GetIK "{ref_joint: [0, 1.57, -1.57, 1.57, 1.57, 0], cartesian_pose: [130.7, 116, 291, 3.13, 0.0, -1.5707]}"
```

## MoveIt 2 and RViz: Plan and Execute Trajectories with a Real JAKA Robot
This section describes how to start the **MoveIt 2** server and **RViz** for planning and executing trajectories with a real JAKA robot. Ensure that **MoveIt 2** (version **2.2+** recommended) is installed before proceeding.

### Precondition
Ensure `jaka_driver` is not running, as **MoveIt 2** and the **JAKA ROS driver** cannot operate simultaneously.

### Start the MoveIt 2 Server
Launch the **MoveIt 2** server by replacing `<robot_ip>` and `<robot_model>` with the actual robot IP and model (e.g., `zu3`, `s5`, `a12`, `minicobo`, etc.):
```bash
ros2 launch jaka_planner moveit_server.launch.py ip:=<robot_ip> model:=<robot_model>
```

### Start MoveIt 2 Client in RViz
In a new terminal, launch the MoveIt 2 RViz interface:
```bash
ros2 launch jaka_<robot_model>_moveit_config demo.launch.py
```
**Note:** The parameter `use_sim` (used to enable/disable RViz simulation mode) is `false` by default, no need to specify unless you want clarity.

### Plan & Execute Trajectories in RViz
Use RViz to plan and execute motions for the JAKA robot with **interactive markers**.

## jaka_ros2 Official Documentation

For **detailed instructions and advanced usage** of jaka_ros2 package, refer to the [JAKA ROS 2 Documentation](jaka_ros2_documentation.md).

For **Chinese version** of the documentation, refer to  [JAKA ROS 2 Documentation-中文版](jaka_ros2_documentation-中文版.md).

## Release Notes
Want to know what's new in the latest version? Check out the [Release Notes](release_notes.md) for highlights, new features, and important improvements introduced in this release.