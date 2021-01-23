# rb-rovi

Robotics and computer vision project of Advanced Robotics at SDU.

* [Overview](#overview)
* [Packages](#packages)
* [Getting Started](#getting-started)
	+ [Dependencies](#dependencies)
	+ [Configuration](#configuration)
	+ [Usage](#usage)
* [Versioning](#versioning)
* [License](#license)
* [Acknowledgments](#acknowledgments)

## Overview

A dynamic simulated environment using ROS/Gazebo framework is used to facilitate development of avision-based pick and place pipeline within a workcell consisting of a UR5 manipulator mounted ontoa specialized table with designated pick and place areas, equipped with various perception sensors. Read the [project report][rovi-report] for more information.

![rovi-workcell][img-rovi-workcell]

The dynamic simulation environment is established with interfaces that enable interaction with theenvironment and control of the manipulator and gripper. A reachability analysis is used to determine the optimal base mount location of the manipulator. Motion planning is used to facilitate object relocation via trajectory generation; both interpolation-based and collision-free methods are implemented and evaluated. Two vision-based pose-estimation methods are implemented and evaluated for localization of object within the workcell.

## Packages

This repository is structured as a `catkin` ROS workspace, consisting of the following packages:

- [`ur5_description`][pkg-ur5-description] - URDF description of the UR5 robot.
- [`ur5_dynamics`][pkg-ur5-dynamics] - Dynamics and kinematics library for the UR5 robot.
- [`ur5_controllers`][pkg-ur5-controllers] - ROS Control controllers for the UR5 robot.
- [`ur5_moveit_config`][pkg-ur5-moveit] - Moveit configuration for the UR5 robot.
- [`rovi_gazebo`][pkg-rovi-gazebo] - Integration of ROVI workcell into Gazebo with various interface methods.
- [`rovi_utils`][pkg-rovi-utils] - Various helper methods for the ROVI system.
- [`rovi_planner`][pkg-rovi-planner] - Methods relating to motion planning of the UR5 robot in the ROVI system using KDL and MoveIt.
- [`rovi_pose_estimator`][pkg-rovi-pose-estimator] - Vision-based pose estimation methods for objects in the ROVIsystem workcell.
- [`rovi_system`][pkg-rovi-system] - Integration of and testing ground for ROVI system components.

Description of each package can be found in the corresponding README (TODO).

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Dependencies

The system is tested on Ubuntu `18.04.3 LTS` and Ubuntu `20.04.3 LTS`. The project is compiled using `GNU C++17`, built with `catkin` (CMake `2.8.3`), mainly depending on:

* [ROS (noetic)][ros] - used as framework for robot operation
* [Gazebo][gazebo] - used as robot simulation environment
* [KDL][kdl] - used for dynamics and motion interpolation
* [MoveIt][moveit] - used for motion planning
* [OpenCV][opencv] - used for computer vision
* [PCL][pcl] - used for point clouds

### Configuration

Installation of the system/workspace/package; the following configuration steps are necessary:

1. Installing ROS
2. Installing external dependecies (OpenCV, PCL via `apt install`)
3. Cloning repository (catkin workspace)
4. Installing ROS dependencies and building catkin workspace

Once the catkin directory is cloned, `rosdep` can be used to install any ROS dependencies; start by [installing `rosdep`][rosdep]. Since `noetic` is relatively new, some packages must be installed from the `ros-testing` repository. To do this, open the `/etc/apt/sources.list.d/ros-latest.list` file as `sudo` and change `/ros/` to `/ros-testing/` - then run `sudo apt update` and `sudo apt upgrade`.

To install ROS dependecies, run `rosdep update` from the `ws` directory, followed by `rosdep install --from-paths src --ignore-src -r -y`. Once all dependencies are installed, the workspace can be built using `catkin_make` from the `ws` directory.

For auto-completion, linting etc., VS Code can be configured [as explained here][ros-vs-code]. VS Code should be launched from the `ws` directory (`code rb-rovi/ws`), where IntelliSense can then be automatically configured to fetch all necessary headers.

### Usage

Navigate to `../rb-rovi/ws/` and run `source devel/setup.bash` to load the necessary environment variables to access the commands to run the packages of this repository. The workspace simulation can then be launched by running `roslaunch rovi_gazebo workcell.launch`.

## Versioning

We use [SemVer][semver] for versioning. For the versions available, see the [releases on this repository][releases]. Furthermore, this [changelog] documents the most relevant changes.

## License

No license has been decided yet.

## Acknowledgments

- [RoboGnome][erdal-git] - configuration of VS Code to run flawlessly with ROS

<!-- LINKS -->

[rovi-report]: /assets/docs/rovi-report.pdf
[img-rovi-workcell]: /assets/img/workcell/rovi-workcell.png

[pkg-ur5-description]: /ws/src/ur5_description
[pkg-ur5-dynamics]: /ws/src/ur5_dynamics
[pkg-ur5-controllers]: /ws/src/ur5_controllers
[pkg-ur5-moveit]: /ws/src/ur5_moveit_config
[pkg-rovi-gazebo]: /ws/src/rovi_gazebo
[pkg-rovi-utils]: /ws/src/rovi_utils
[pkg-rovi-planner]: /ws/src/rovi_planner
[pkg-rovi-pose-estimator]: /ws/src/rovi_pose_estimator
[pkg-rovi-system]: /ws/src/rovi_system

[semver]: http://semver.org/
[releases]: about:blank
[changelog]: CHANGELOG.md
[wiki]: about:blank

[ros]: http://wiki.ros.org/noetic
[gazebo]: http://gazebosim.org
[kdl]: https://www.orocos.org/kdl.html
[moveit]: https://moveit.ros.org
[opencv]: https://opencv.org
[pcl]: https://pointclouds.org
[rosdep]: http://wiki.ros.org/rosdep#Installing_rosdep
[ros-vs-code]: https://github.com/RoboGnome/VS_Code_ROS

[erdal-git]: https://github.com/erdalpekel
[robognome-git]: https://github.com/RoboGnome

