# SES-P ROS2 Arms Package

The SES-P-ROS2-Arms repository contains common packages that are used by both the physical and simulated SES PRO Arms (5DoF/6DoF & 550mm/900mm versions).

## Table of Contents

- [Prerequisites](#prerequisites)
- [Package installation](#package-installation)
- [Description package](#description-package)
- [MoveIt package](#moveit-package)
- [Simulated examples (C++)](#lss-ignition-moveit-example)
- [Author](#author)
- [Resources](#resources)

## Prerequisites

1. [Ubuntu 22.04 (Jammy Jellyfish)](https://releases.ubuntu.com/jammy/)
2. [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)
3. ROS2 dev tools:
```
sudo apt install python3-pip
sudo pip install colcon-common-extensions
sudo pip install vcstool
```
4. Rosdep: Used to install dependencies when building from sources
```
sudo apt install python3-rosdep2
rosdep update
```
5. [Gazebo Ignition Fortress](https://gazebosim.org/docs/fortress/install_ubuntu)

## Package installation

```
git clone https://github.com/Lynxmotion/SES-P-ROS2-Arms.git
mkdir -p src
mv SES-P-ROS2-Arms/* src
mv src SES-P-ROS2-Arms
```

### Install dependencies

```
cd SES-P-ROS2-Arms
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

To generate the **REQUIRED** SDF file used for the simulation run:
```
bash src/pro_arm_description/scripts/xacro2sdf.bash
```

All the available sdf generation scripts have the following configuration options:

**-d:** Which PRO Arm version to use
- options: 5, 6
- default: 6

**-s**: Size of the model to use
- options: 550, 900
- default: 550

**-g**: Gripper model to use
- options: none, pge_5040, cge_1010
- default: none

**-f**: Finger model to use
- options: 20, 40, 60, 80
- default: 40

The script defaults to the 6DoF 550mm model without gripper. 

To generate the SDF of the 6DoF 550mm model with the gripper PGE_5040, 60mm finger version run:
```
bash src/pro_arm_description/scripts/xacro2sdf.bash -g pge_5040 -f 60
```

To generate the SDF of the 5DoF 900mm model run:
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
The decription and moveit packages launch files have the following configuration options:

**dof**: Which PRO Arm version to use
- options: 5, 6
- default: 6

**size**: Size of the model to use
- options: 550, 900
- default: 550

**gripper**: Gripper model to use
- options: none, pge_5040, cge_1010
- default: none

**finger**: Finger separation model to use
- options: 20, 40, 60, 80
- default: 40

* The 20 model is only available for the cge_1010 gripper and the 80 is only available for the pge_5040 gripper

**View Model in Rviz**

```
ros2 launch pro_arm_description view.launch.py
```
<p align="center">
  <table align="center" border="0">
    <tr>
      <td align="center">
        <img src="https://github.com/Lynxmotion/PRO-ROS2-Arms/blob/main/images/rviz_view_550.jpg" height="210px"/>
        <br>size 550mm gripper none
      </td>
      <td align="center">
        <img src="https://github.com/Lynxmotion/PRO-ROS2-Arms/blob/main/images/rviz_view_550_pge_5040_40.jpg" height="210px"/>
        <br>size 550mm gripper pge_5040 finger 40
      </td>
      <td align="center">
        <img src="https://github.com/Lynxmotion/PRO-ROS2-Arms/blob/main/images/rviz_view_550_cge_1010_20.jpg" height="210px"/>
        <br>size 550mm gripper cge_1010 finger 20
      </td>
    </tr>
  </table>
  <table align="center" border="0">
    <tr>
      <td align="center">
        <img src="https://github.com/Lynxmotion/PRO-ROS2-Arms/blob/main/images/rviz_view_900.jpg" height="210px"/>
        <br>size 900mm gripper none
      </td>
      <td align="center">
        <img src="https://github.com/Lynxmotion/PRO-ROS2-Arms/blob/main/images/rviz_view_900_pge_5040_80.jpg" height="210px"/>
        <br>size 900mm gripper pge_5040 finger 80
      </td>
      <td align="center">
        <img src="https://github.com/Lynxmotion/PRO-ROS2-Arms/blob/main/images/rviz_view_900_cge_1010_60.jpg" height="210px"/>
        <br>size 900mm gripper cge_1010 finger 60
      </td>
    </tr>
  </table>
</p>

<p align="center">
  <img src="https://github.com/Lynxmotion/PRO-ROS2-Arms/blob/main/images/rviz_view_550_pge_5040_60.gif" width="600px"/>
</p>

**View in Gazebo Ignition**

```
ros2 launch pro_arm_description view_ign.launch.py size:=900 gripper:=pge_5040 finger:=60
```

<p align="center">
  <img src="https://github.com/Lynxmotion/PRO-ROS2-Arms/blob/main/images/ign_view_900_pge_5040_60.jpg" width="600px"/>
</p>

* Note that the generated SDF model is required.

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

**Base control launch file**


```
ros2 launch pro_arm_moveit move_arm.launch.py
```

<p align="center">
  <img src="https://github.com/Lynxmotion/PRO-ROS2-Arms/blob/main/images/move_arm_control.png" height="230px"/>
</p>


* Note: This also launches the emergency_stop_marker node that integrates an interactive marker to STOP/START the arm_trajectory_controller. This functions as an Emergency Stop button but the same behavior can be achieved by running the folowing commands on a different terminal:

```
ros2 run pro_arm_moveit trigger_emergency_stop.py start
```
```
ros2 run pro_arm_moveit trigger_emergency_stop.py stop
```

These commands are available for the rest of the pro_arm_moveit package.

**Fake controllers (Rviz)**

```
ros2 launch pro_arm_moveit fake_arm_control.launch.py
```

<p align="center">
  <table align="center" border="0">
    <tr>
      <td align="center">
        <img src="https://github.com/Lynxmotion/PRO-ROS2-Arms/blob/main/images/fake_arm_control.jpg" height="230px"/>
      </td>
      <td align="center">
        <img src="https://github.com/Lynxmotion/PRO-ROS2-Arms/blob/main/images/fake_arm_control.gif" height="230px"/>
      </td>
    </tr>
  </table>
</p>

**Simulated controllers (Rviz + Gazebo Ignition Simulation)**

```
ros2 launch pro_arm_moveit sim_arm_control.launch.py
```

<p align="center">
  <table align="center" border="0">
    <tr>
      <td align="center">
        <img src="https://github.com/Lynxmotion/PRO-ROS2-Arms/blob/main/images/sim_arm_control.jpg" height="200px"/>
      </td>
      <td align="center">
        <img src="https://github.com/Lynxmotion/PRO-ROS2-Arms/blob/main/images/sim_arm_control.gif" height="200px"/>
      </td>
    </tr>
  </table>
</p>

**Real controller (RViz + Real Robot)**

Before controlling the real robot first follow these steps:

1. Update the firmware on the servos using [PRO Config](https://wiki.lynxmotion.com/info/wiki/lynxmotion/view/lynxmotion-smart-servo/lss-configuration-software/)

2. Follow the [initial setup](https://wiki.lynxmotion.com/info/wiki/lynxmotion/view/ses-software/lss-flowarm/?#HInitialSetup) and make sure the IDs are configured correctly and the arm is calibrated

To control the arm:

1. Run the launch file

```
ros2 launch pro_arm_moveit real_arm_control.launch.py
```

* Note: If the servos light up *Blue* they have been configured correctly if not try running this first:
```
sudo chmod 666 /dev/ttyACM0
```

<p align="center">
  <table align="center" border="0">
    <tr>
      <td align="center">
        <img src="https://github.com/Lynxmotion/PRO-ROS2-Arms/blob/main/images/pro_arm_ros2_control.gif" height="300px"/>
        <br>Example 1: Real time
      </td>
      <td align="center">
        <img src="https://github.com/Lynxmotion/PRO-ROS2-Arms/blob/main/images/pro_arm_moveit_control.gif" height="300px"/>
        <br>Example 2: Speeded up 2x
      </td>
    </tr>
  </table>
</p>

* Note: The planner uses a velocity and acceleration scaling factor of 0.5, these can be adjusted through RViz. The custom hardware interface also supports setting maximum speed and acceleration values which are configured at the initial stage and can be adjusted in the *pro_arm.ros2_control* file.

### PRO Simulation Examples

**C++**

**Follow Goal Demo (Simulation)**

The pro_sim_examples package includes the follow_target demo which simulates the LSS arm in Gazebo Ignition and allows the user to interact with a box in the virtual environment. This example includes a C++ implementation that makes the arm track the target (box) whenever you change its location.

```
ros2 launch pro_sim_examples ex_cpp_follow_target.launch.py
```

Note: The 5DoF version does not have enough degrees of freedom to achieve all desired end-effector poses (position + orientation). This implementation first attempts to move to the desired pose, if unsuccessful it sets a position only target, this allows it to plan a trajectory "ignoring" the orientation.

<p align="center">
  <table align="center" border="0">
    <tr>
      <td align="center">
        <img src="https://github.com/Lynxmotion/PRO-ROS2-Arms/blob/main/images/follow_target_550_pge.jpg" height="260px"/>
      </td>
      <td align="center">
        <img src="https://github.com/Lynxmotion/PRO-ROS2-Arms/blob/main/images/follow_target_900_cge.gif" height="260px"/>
      </td>
    </tr>
  </table>
</p>

**Move Object Example (Simulation)**

The move_object example contains a C++ implementation to make the arm move a box from one table to another. The motions are simulated in Gazebo Ignition.

```
ros2 launch pro_sim_examples ex_cpp_move_object.launch.py
```

<p align="center">
  <table align="center" border="0">
    <tr>
      <td align="center">
        <img src="https://github.com/Lynxmotion/PRO-ROS2-Arms/blob/main/images/move_object_550.jpg" height="230px"/>
      </td>
      <td align="center">
        <img src="https://github.com/Lynxmotion/PRO-ROS2-Arms/blob/main/images/move_object_900.gif" height="230px"/>
      </td>
    </tr>
  </table>
</p>

## Author

- [Geraldine Barreto](http://github.com/geraldinebc)

## Resources

Read more about the PRO Robotic Arm in the [Wiki](https://wiki.lynxmotion.com/info/wiki/lynxmotion/view/ses-pro-arms/).

Purchase the PRO arm on [RobotShop](https://www.robotshop.com/collections/lynxmotion-ses-pro-robotic-arms).

Official Lynxmotion Smart Servo PRO (LSS-P) Hardware Interface available [here](https://github.com/Lynxmotion/LSS-P-ROS2-Hardware). 

If you want more details about the LSS-P communication protocol, visit this [website](https://wiki.lynxmotion.com/info/wiki/lynxmotion/view/lynxmotion-smart-servo-pro/lss-p-communication-protocol/).

Have any questions? Ask them on the RobotShop [Community](https://community.robotshop.com/forum/c/lynxmotion/electronics-software/27).
