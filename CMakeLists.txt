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
  include
  ${catkin_INCLUDE_DIRS}
  ${workspaceFolder}/tracking_test/src
)

add_executable(total src/total.cpp)
target_link_libraries(total ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(change_topic src/change_topic.cpp)
target_link_libraries(change_topic ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(make_rosbag src/make_rosbag.cpp)
target_link_libraries(make_rosbag ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(lxy2global src/lxy2global.cpp)
target_link_libraries(lxy2global ${catkin_LIBRARIES} ${GeographicLib_LIBRARIES} ${OpenCV_LIBS})

add_executable(clustering src/clustering.cpp)
target_link_libraries(clustering ${catkin_LIBRARIES} ${GeographicLib_LIBRARIES} ${OpenCV_LIBS})

add_executable(sort src/sort.cpp)
target_link_libraries(sort ${catkin_LIBRARIES} ${GeographicLib_LIBRARIES} ${OpenCV_LIBS})

add_executable(pctest src/pctest.cpp)
target_link_libraries(pctest ${catkin_LIBRARIES} ${GeographicLib_LIBRARIES} ${OpenCV_LIBS})

add_executable(make_ref_map src/make_ref_map.cpp)
target_link_libraries(make_ref_map ${catkin_LIBRARIES} ${GeographicLib_LIBRARIES} ${OpenCV_LIBS})