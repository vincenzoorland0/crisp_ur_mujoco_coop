cmake_minimum_required(VERSION 3.8)
project(vaacum_gripper_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch rviz vacuum_gripper
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
  PATTERN ".gitkeep" EXCLUDE
)

install(
  DIRECTORY vacuum_gripper/assets
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
