cmake_minimum_required(VERSION 3.8)
project(slamware_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ========================================
#  ROS2 dependencies
# ========================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

# ========================================
#  Slamware SDK  (set SLAMWARE_SDK_ROOT)
# ========================================
# Try environment variable first, then a cmake cache variable
if(DEFINED ENV{SLAMWARE_SDK_ROOT})
  set(SLAMWARE_SDK_ROOT "$ENV{SLAMWARE_SDK_ROOT}" CACHE PATH "Slamware SDK root")
elseif(NOT DEFINED SLAMWARE_SDK_ROOT)
  message(FATAL_ERROR
    "SLAMWARE_SDK_ROOT is not set.\n"
    "Set the environment variable before building:\n"
    "  export SLAMWARE_SDK_ROOT=~/slamware_sdk/slamware_sdk_linux-x86_64-gcc11\n"
    "Or pass it to cmake:\n"
    "  colcon build --cmake-args -DSLAMWARE_SDK_ROOT=/path/to/sdk")
endif()

# Resolve ~ in path
string(REPLACE "~" "$ENV{HOME}" SLAMWARE_SDK_ROOT "${SLAMWARE_SDK_ROOT}")

if(NOT EXISTS "${SLAMWARE_SDK_ROOT}/include")
  message(FATAL_ERROR "Slamware SDK include/ not found at: ${SLAMWARE_SDK_ROOT}/include")
endif()
if(NOT EXISTS "${SLAMWARE_SDK_ROOT}/lib")
  message(FATAL_ERROR "Slamware SDK lib/ not found at: ${SLAMWARE_SDK_ROOT}/lib")
endif()

message(STATUS "Slamware SDK: ${SLAMWARE_SDK_ROOT}")

# Collect all static libraries from SDK
file(GLOB SLAMWARE_LIBS "${SLAMWARE_SDK_ROOT}/lib/*.a")
if(NOT SLAMWARE_LIBS)
  # Try shared libraries as fallback
  file(GLOB SLAMWARE_LIBS "${SLAMWARE_SDK_ROOT}/lib/*.so")
endif()
if(NOT SLAMWARE_LIBS)
  message(FATAL_ERROR "No library files found in ${SLAMWARE_SDK_ROOT}/lib/")
endif()

message(STATUS "Slamware SDK libraries: ${SLAMWARE_LIBS}")

# ========================================
#  Build the node
# ========================================
add_executable(slamware_bridge_node
  src/slamware_bridge_node.cpp
)

# SDK headers must come first (ships its own Boost 1.53)
target_include_directories(slamware_bridge_node BEFORE PRIVATE
  ${SLAMWARE_SDK_ROOT}/include
)

# Use link group to resolve circular dependencies in static libs
target_link_libraries(slamware_bridge_node
  -Wl,--start-group
  ${SLAMWARE_LIBS}
  -Wl,--end-group
  pthread
  rt
  dl
)

# Suppress warnings from SDK's old Boost headers
target_compile_options(slamware_bridge_node PRIVATE
  -Wno-deprecated-declarations
  -Wno-unused-parameter
)

ament_target_dependencies(slamware_bridge_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  tf2_ros
  tf2
)

# ========================================
#  Install
# ========================================
install(TARGETS slamware_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
