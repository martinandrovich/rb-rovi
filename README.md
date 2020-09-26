# rb-rovi
Robotics and computer vision project of Advanced Robotics at SDU.

* [Overview](#overview)
* [Getting Started](#getting-started)
	+ [Dependencies](#dependencies)
	+ [Configuration](#configuration)
	+ [Installation](#installation)
	+ [Test](#test)
	+ [API & Troubleshooting](#api--troubleshooting)
* [Versioning](#versioning)
* [License](#license)
* [Acknowledgments](#acknowledgments)

## Overview

Short description and image here.

## Packages

This repository is structured as a `catkin` ROS workspace, consisting of the following packages:

```
rb-rovi/ws/src
    /ur_description
    /ur_controllers
    /rovi_gazebo
    /rovi_planner
    /rovi_pose_estimator
    /rovi_system
        manipulator_reachability.cpp
        pose_estimator_test.cpp
        pick_and_place_known.cpp
        pick_and_place_estimated.cpp
```

Description of each package can be found in the corresponding README.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Dependencies

The system is running a Ubuntu (`20.04.3 LTS`) distribtuion. The project is compiled using `C++17 GCC`, managed with CMake (`3.10.2`) and built with:

* [ROS (noetic)][ros] - used as framework for robot operation
* [Gazebo][gazebo] - used as robot simulation environment

### Configuration

Installation of the system/workspace/package; the following configuration steps are necessary:

1. Installing `ROS`
2. Cloning repository (catkin workspace)

Scripts for these steps will be available later.

### Usage

Guide on how to test if the installation if successful.

For auto-completion, linting etc. in VS Code the [ROS plugin][ros-vs-code-plugin] must be installed. VS Code should be launched from the `ws` directory (`code ../rb-rovi/ws`), where IntelliSense can then be automatically configured to fetch all necessary headers. In case of include errors, delete the `ws/.vscode` directory and reload VS Code.

### API & Troubleshooting

Text.

## Versioning

We use [SemVer][semver] for versioning. For the versions available, see the [releases on this repository][releases]. Furthermore, this [changelog] documents the most relevant changes.

## License

No license has been decided yet.

## Acknowledgments

- [Erdal Perkel][erdal-git] - integration of Franka Emika Panda into Gazebo

[semver]: http://semver.org/
[releases]: about:blank
[changelog]: CHANGELOG.md
[wiki]: about:blank

[ros]: http://wiki.ros.org/noetic/
[gazebo]: http://gazebosim.org/

[erdal-git]: https://github.com/erdalpekel

