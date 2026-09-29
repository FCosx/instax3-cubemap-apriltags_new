cmake_minimum_required(VERSION 3.16)
project(insta360_ros_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# The vendor SDK is intentionally not part of this repository.  Point this
# variable at an extracted SDK or at a separately installed SDK layout that
# contains include/camera, include/stream, and lib/libCameraSDK.so.
set(INSTA360_SDK_DIR "$ENV{INSTA360_SDK_DIR}" CACHE PATH
  "External Insta360 CameraSDK root")

set(_insta360_sdk_hints)
if(INSTA360_SDK_DIR)
  list(APPEND _insta360_sdk_hints
    "${INSTA360_SDK_DIR}"
    "${INSTA360_SDK_DIR}/include"
  )
endif()

find_path(INSTA360_CAMERA_INCLUDE_DIR
  NAMES camera/camera.h
  HINTS ${_insta360_sdk_hints}
)
find_path(INSTA360_STREAM_INCLUDE_DIR
  NAMES stream/stream_delegate.h
  HINTS ${_insta360_sdk_hints}
)
find_library(INSTA360_CAMERA_SDK_LIBRARY
  NAMES CameraSDK libCameraSDK
  HINTS
    "${INSTA360_SDK_DIR}/lib"
    "${INSTA360_SDK_DIR}"
)

if(NOT INSTA360_CAMERA_INCLUDE_DIR OR
   NOT INSTA360_STREAM_INCLUDE_DIR OR
   NOT INSTA360_CAMERA_SDK_LIBRARY)
  message(FATAL_ERROR
    "Insta360 CameraSDK was not found. The SDK is intentionally excluded "
    "from this project. Set INSTA360_SDK_DIR to a directory containing "
    "camera/camera.h, stream/stream_delegate.h, and lib/libCameraSDK.so.")
endif()

get_filename_component(INSTA360_SDK_LIBRARY_DIR
  "${INSTA360_CAMERA_SDK_LIBRARY}" DIRECTORY)

add_executable(${PROJECT_NAME} src/insta360_ros_driver.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE
  "${INSTA360_CAMERA_INCLUDE_DIR}"
  "${INSTA360_STREAM_INCLUDE_DIR}"
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
)
target_link_libraries(${PROJECT_NAME}
  "${INSTA360_CAMERA_SDK_LIBRARY}"
)
set_target_properties(${PROJECT_NAME} PROPERTIES
  BUILD_RPATH "${INSTA360_SDK_LIBRARY_DIR}"
  INSTALL_RPATH "${INSTA360_SDK_LIBRARY_DIR}"
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS scripts/setup_udev.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
