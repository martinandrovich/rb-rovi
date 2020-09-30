cmake_minimum_required(VERSION 2.8.3)
project(ur5_description)

find_package(catkin REQUIRED COMPONENTS
  xacro
)

catkin_package(CATKIN_DEPENDS xacro)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


## Mark cpp header files for installation
install(DIRECTORY meshes
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY robots
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
