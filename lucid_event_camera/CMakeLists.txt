cmake_minimum_required(VERSION 3.8)
project(lucid_event_camera)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(event_camera_msgs REQUIRED)

# --- Arena SDK root ---
set(ARENA_SDK_ROOT /home/seaonics/ArenaSDK_v0.1.78_Linux_ARM64/ArenaSDK_Linux_ARM64)

# --- Include directories ---
include_directories(
  include
  ${ARENA_SDK_ROOT}/include
  ${ARENA_SDK_ROOT}/GenICam/library/CPP/include
)

# --- Library directories (.so files) ---
link_directories(
  ${ARENA_SDK_ROOT}/lib
  ${ARENA_SDK_ROOT}/GenICam/library/lib/Linux64_ARM
)

# --- Executable (BOTH SOURCES) ---
add_executable(triton_event_node
  src/triton_event_node.cpp
  #src/test_node.cpp
)

target_include_directories(triton_event_node PRIVATE
  ${ARENA_SDK_ROOT}/include
  ${ARENA_SDK_ROOT}/GenICam/library/CPP/include
)

target_link_libraries(triton_event_node
  arena
  gentl
  GCBase_gcc54_v3_3_LUCID
  GenApi_gcc54_v3_3_LUCID
  Log_gcc54_v3_3_LUCID
  XmlParser_gcc54_v3_3_LUCID
  MathParser_gcc54_v3_3_LUCID
  NodeMapData_gcc54_v3_3_LUCID
  log4cpp_gcc54_v3_3_LUCID
)

ament_target_dependencies(triton_event_node
  rclcpp
  sensor_msgs
  event_camera_msgs
)

install(TARGETS triton_event_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
