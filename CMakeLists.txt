cmake_minimum_required(VERSION 3.8)
project(px4_drone)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)

include_directories(include)

# --- Soporte Python ---
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  px4_drone/offboard_streamer.py
  DESTINATION lib/${PROJECT_NAME}
)

# --- Soporte C++ ---
add_executable(offboard_control src/offboard_control.cpp)
target_include_directories(offboard_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(offboard_control rclcpp px4_msgs)
install(TARGETS offboard_control DESTINATION lib/${PROJECT_NAME})

add_executable(offboard_position_handshake src/offboard_position_handshake.cpp)
target_include_directories(offboard_position_handshake PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(offboard_position_handshake rclcpp px4_msgs)
install(TARGETS offboard_position_handshake DESTINATION lib/${PROJECT_NAME})

# takeoff_position_hold_outdoor y takeoff_position_hold_indoor comparten toda
# la logica de despegue/hold/aterrizaje via takeoff_position_hold_base.cpp;
# solo difieren en que fuente de posicion exigen antes de despegar.
add_executable(takeoff_position_hold_outdoor
  src/takeoff_position_hold_outdoor.cpp
  src/takeoff_position_hold_base.cpp)
target_include_directories(takeoff_position_hold_outdoor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(takeoff_position_hold_outdoor rclcpp px4_msgs)
install(TARGETS takeoff_position_hold_outdoor DESTINATION lib/${PROJECT_NAME})

add_executable(takeoff_position_hold_indoor
  src/takeoff_position_hold_indoor.cpp
  src/takeoff_position_hold_base.cpp)
target_include_directories(takeoff_position_hold_indoor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(takeoff_position_hold_indoor rclcpp px4_msgs)
install(TARGETS takeoff_position_hold_indoor DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
