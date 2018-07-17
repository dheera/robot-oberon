cmake_minimum_required(VERSION 2.8.3)
project(imu_bno055)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs std_msgs)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES roboteq_transfer_command
#  CATKIN_DEPENDS roboteq_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(imu_bno055_node src/imu_bno055_node.cpp src/imu_bno055.cpp)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(imu_bno055_node
  ${catkin_LIBRARIES}
)

install(TARGETS imu_bno055_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

