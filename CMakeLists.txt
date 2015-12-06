cmake_minimum_required(VERSION 2.8.3)
project(avoid_opt_by_contour C CXX)
aux_source_directory(./src SRC_LIST)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  ardrone_autonomy




)


catkin_package()

find_package(OpenCV)
include_directories(
  ${OpenCV_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}/opencv2 ${OpenCV_INCLUDE_DIRS}/opencv
)

include_directories(include ${catkin_INCLUDE_DIRS} .)
add_executable(avoid_opt_by_contour ${SRC_LIST})
target_link_libraries(avoid_opt_by_contour ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(avoid_opt_by_contour avoid_opt_by_contour_generate_messages_cpp)

