cmake_minimum_required(VERSION 3.16)
project(da360_ros_pipeline)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)


pkg_check_modules(AVCODEC REQUIRED libavcodec)
pkg_check_modules(AVFORMAT REQUIRED libavformat)
pkg_check_modules(AVUTIL REQUIRED libavutil)
pkg_search_module(SWSCALE REQUIRED libswscale)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${AVCODEC_INCLUDE_DIRS}
  ${AVFORMAT_INCLUDE_DIRS}
  ${AVUTIL_INCLUDE_DIRS}
  ${SWSCALE_INCLUDE_DIRS}
)

add_executable(panorama_node src/panorama_node.cpp)
target_compile_features(panorama_node PRIVATE cxx_std_17)
ament_target_dependencies(panorama_node
  rclcpp
  rcl_interfaces
  sensor_msgs
  std_msgs
  cv_bridge
  OpenCV
)
target_link_libraries(panorama_node
  ${OpenCV_LIBRARIES}
  ${AVCODEC_LIBRARIES}
  ${AVFORMAT_LIBRARIES}
  ${AVUTIL_LIBRARIES}
  ${SWSCALE_LIBRARIES}
)
install(TARGETS panorama_node DESTINATION lib/${PROJECT_NAME})


ament_python_install_package(${PROJECT_NAME})
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/cubemap_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
