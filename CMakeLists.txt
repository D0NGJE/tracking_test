cmake_minimum_required(VERSION 3.0.2)
project(tracking_test)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
)
find_package(OpenCV REQUIRED)
find_package(GeographicLib REQUIRED)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(total src/total.cpp)
target_link_libraries(total ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(lxy2global src/lxy2global.cpp)
target_link_libraries(lxy2global ${catkin_LIBRARIES} ${GeographicLib_LIBRARIES} ${OpenCV_LIBS})
