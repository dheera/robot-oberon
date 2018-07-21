cmake_minimum_required(VERSION 2.8.3)
project(avoidance)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs std_msgs geometry_msgs serial)

catkin_package(INCLUDE_DIRS include)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(avoidance_node src/avoidance_node.cpp src/AvoidanceActivity.cpp)

include_directories(${catkin_INCLUDE_DIRS})

target_link_libraries(avoidance_node ${catkin_LIBRARIES})

install(TARGETS avoidance_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

