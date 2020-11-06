## 2-10-2020

Daniel:
- Added the WSG end-effector to a seperate URDF, transmission and Gazebo file.
- Added a launch file in ur5_controllers, which launches the UR5 with the WSG end-effector.
- The .dae - visual - files does not work for the WSG end-effector, so one might add them later, for now there is used .stl files
- TODO: MoveIT

Daniel:
- https://moveit.ros.org/install/source/dependencies/
- package for OMPL https://packages.ubuntu.com/bionic/libompl-dev
- sudo apt -qq install libccd-dev
- https://www.programmersought.com/article/81104776324/