cmake_minimum_required(VERSION 2.8.3)
project(gripper_description)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

install(DIRECTORY Media meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
