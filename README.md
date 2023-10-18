# ROS2 PRO Arms Package

The ROS2-PRO-Arms repository contains common packages that are used by both the physical and simulated LSS Arms (5DoF/6DoF & 550mm/900mm versions).

## Table of Contents

- [Prerequisites](#prerequisites)
- [Package installation](#package-installation)
- [Description package](#description-package)
- [MoveIt package](#moveit-package)
- [Simulated examples (C++)](#lss-ignition-moveit-example)
- [Author](#author)
- [Resources](#resources)

## Prerequisites

1. [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html)
2. ROS 2 dev tools:
    - [colcon-common-extensions](https://pypi.org/project/colcon-common-extensions/)
    - [vcs](https://pypi.org/project/vcstool/): Automates cloning of git repositories declared on a YAML file.
3. Rosdep: Used to install dependencies when building from sources
```
sudo apt install python3-rosdep2
rosdep update
```
4. Gazebo Ignition Fortress
```
sudo apt-get update && sudo apt-get install wget
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update && sudo apt-get install ignition-fortress
```

## Package installation

```
git clone https://github.com/Lynxmotion/lss_pro_tests.git
```

### Install dependencies

```
cd lss_pro_tests
rosdep install --from-path src -yi --rosdistro humble
cd src
vcs import < required.repos
cd ..
```

### Build instructions

```
export GZ_VERSION=fortress
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

### Initialization instructions

You will have to use this in every new session in which you wish to use these packages:

```
source install/setup.bash
```

## Description package

The pro_arm_description package contains the URDF description & SDF model of the robot and the mesh files for each component.

It contains scripts that convert xacro into the required URDF and SDF files.

To generate the **required** SDF file used for the simulation run:
```
bash src/pro_arm_description/scripts/xacro2sdf.bash
```

The script defaults to the 6DoF 550mm version. To generate the SDF of the 5DoF 900mm model run:
```
bash src/pro_arm_description/scripts/xacro2sdf.bash -d 5 -s 900
```

Similarly, if you want to generate the URDF files (not required) you can run:
```
bash src/pro_arm_description/scripts/xacro2urdf.bash
```
or
```
bash src/pro_arm_description/scripts/xacro2urdf.bash -d 5 -s 900
```
All the available launch files have the following configuration options:

**dof**: Which PRO Arm version to use
- options: 5, 6
- default: 6

**size**: Size of the model to use
- options: 550, 900
- default: 550

**View Model in Rviz**

```
ros2 launch pro_arm_description view.launch.py
```

**View in Gazebo Ignition**

```
ros2 launch pro_arm_description view_ign.launch.py dof:=5
```

### MoveIt package

The pro_arm_moveit package contains all the configuration and launch files for using the PRO Arm with the MoveIt2 Motion Planning Framework.

It offers different controller plugins for the manipulator ('fake', 'sim' and 'real')

If you want to generate the SRDF files (not required) you can run:
```
bash src/pro_arm_moveit/scripts/xacro2srdf.bash
```
or
```
bash src/pro_arm_description/scripts/xacro2urdf.bash -d 5 -s 900
```

The following launch files are available:

**Fake controllers (Rviz)**

```
ros2 launch pro_arm_moveit fake_arm_control.launch.py size:=900
```

**Simulated controllers (Rviz + Gazebo Ignition Simulation)**

```
ros2 launch pro_arm_moveit sim_arm_control.launch.py dof:=5
```
--------------------------- TODO START ---------------------------

**Real controller (RViz + Real Robot)**

Before controlling the real robot first follow these steps:

1. Update the firmware on the servos using [PRO Config](https://wiki.lynxmotion.com/info/wiki/lynxmotion/view/lynxmotion-smart-servo/lss-configuration-software/)

2. Follow the [initial setup](https://wiki.lynxmotion.com/info/wiki/lynxmotion/view/ses-software/lss-flowarm/?#HInitialSetup) and make sure the IDs are configured correctly

3. Calibrate the arm using [FlowArm](https://www.robotshop.com/products/lynxmotion-lss-lss-flowarm-app-download)

4. Set the *Gyre Direction* to CCW (-1) for all the motors

To control the arm:

1. Run the launch file

```
ros2 launch pro_arm_moveit real_arm_control.launch.py dof:=5
```

* Note: If the servos light up *Blue* they have been configured correctly if not try running:
```
sudo chmod 766 /dev/ttyUSB0
```

2. To activate the servos open another terminal and run:
```
ros2 topic pub --once /effort_controller/commands std_msgs/msg/Float64MultiArray "data:
- 6.8
- 6.8
- 6.8
- 6.8
- 6.8"
```
* Note: For the 6DoF version add an extra - 6.8

3. Now you are able to plan the trajectories using MoveIt2 and execute them with real hardware

To make the servos go Limp use:
```
ros2 topic pub --once /effort_controller/commands std_msgs/msg/Float64MultiArray "data:
- 0
- 0
- 0
- 0
- 0"
```
* Note: For the 6DoF version add an extra - 0

--------------------------- TODO END ---------------------------

### LSS Ignition MoveIt Example

**Follow Goal Demo (Simulation)**

The lss_ign_moveit_example package contains an example of a C++ implementation to follow a target. This is simulated in Gazebo Ignition, the target (box) can be moved in the world and the Arm will move to the desired position.

```
ros2 launch pro_sim_examples ex_cpp_follow_target.launch.py size:=900
```

Note: The 5DoF version does not have enough degrees of freedom to achieve all desired position + orientation targets. This implementation adjusts the goal orientation so it is always parallel to the base of the robot, this allows it to plan a trajectory "ignoring" the orientation.

**Move Object Demo (Simulation)**

The lss_ign_moveit_example package contains an example of a C++ implementation to move a box from a table to another. The motions are simulated in Gazebo Ignition.

```
ros2 launch pro_sim_examples ex_cpp_move_object.launch.py
```

## Author

- [Geraldine Barreto](http://github.com/geraldinebc)

## Resources

Read more about the LSS Robotic Arm in the [Wiki](https://wiki.lynxmotion.com/info/wiki/lynxmotion/view/ses-v2-arms/).

Purchase the LSS arm on [RobotShop](https://www.robotshop.com/collections/lynxmotion-smart-servos-articulated-arm).

Official Lynxmotion Smart Servo (LSS) Hardware Interface available [here](https://github.com/Lynxmotion/LSS-ROS2-Control).

If you want more details about the LSS protocol, go [here](https://wiki.lynxmotion.com/info/wiki/lynxmotion/view/lynxmotion-smart-servo/lss-communication-protocol/).

Have any questions? Ask them on the RobotShop [Community](https://community.robotshop.com/forum/c/lynxmotion/electronics-software/27).
