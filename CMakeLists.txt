cmake_minimum_required(VERSION 3.8)
project(px4_drone)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

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

add_executable(ev_odometry_bridge src/ev_odometry_bridge.cpp)
target_include_directories(ev_odometry_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(ev_odometry_bridge rclcpp px4_msgs tf2_ros geometry_msgs)
install(TARGETS ev_odometry_bridge DESTINATION lib/${PROJECT_NAME})

add_executable(ev_offboard_handshake src/ev_offboard_handshake.cpp)
target_include_directories(ev_offboard_handshake PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(ev_offboard_handshake rclcpp px4_msgs)
install(TARGETS ev_offboard_handshake DESTINATION lib/${PROJECT_NAME})

add_executable(takeoff_position_hold_ev
  src/takeoff_position_hold_ev.cpp
  src/takeoff_position_hold_base.cpp
  src/obstacle_guard.cpp)
target_include_directories(takeoff_position_hold_ev PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(takeoff_position_hold_ev rclcpp px4_msgs sensor_msgs geometry_msgs)
install(TARGETS takeoff_position_hold_ev DESTINATION lib/${PROJECT_NAME})

# Reproduce barridos grabados a traves de ObstacleGuard, sin ROS: sirve para
# validar la parada por obstaculo con rosbags reales antes de volar
# (tools/obstacle_guard/bag_to_scans.py genera la entrada).
add_executable(obstacle_guard_replay
  src/obstacle_guard_replay.cpp
  src/obstacle_guard.cpp)
target_include_directories(obstacle_guard_replay PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
install(TARGETS obstacle_guard_replay DESTINATION lib/${PROJECT_NAME})

# --- Relay de barridos a tamano fijo (LD19 -> slam_toolbox) ---
# El LD19 publica un numero variable de rayos y karto rechaza los que no
# coinciden con el primero; con esto pasan todos. Ver include/px4_drone/scan_fixed_size.hpp.
add_executable(scan_fixed_size src/scan_fixed_size_node.cpp)
target_include_directories(scan_fixed_size PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(scan_fixed_size rclcpp sensor_msgs)
install(TARGETS scan_fixed_size DESTINATION lib/${PROJECT_NAME})

add_executable(test_scan_fixed_size tools/scan_fixed_size/test_scan_fixed_size.cpp)
target_include_directories(test_scan_fixed_size PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(test_scan_fixed_size sensor_msgs)
install(TARGETS test_scan_fixed_size DESTINATION lib/${PROJECT_NAME})

# --- Nodo MPC (opcional) ---
# Se compila solo si (a) acados esta instalado (ACADOS_SOURCE_DIR o ~/acados) y
# (b) el solver se ha generado en ESTA maquina con
#     python3 scripts/mpc/generate_cycle_mpc.py
# El codigo generado NO se versiona: acados cambia de API entre versiones, asi
# que cada maquina genera el suyo contra su propia instalacion.
set(ACADOS_DIR "$ENV{ACADOS_SOURCE_DIR}")
if(NOT ACADOS_DIR)
  set(ACADOS_DIR "$ENV{HOME}/acados")
endif()
set(MPC_GEN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/scripts/mpc/generated/c_generated_code")
if(EXISTS "${ACADOS_DIR}/lib" AND EXISTS "${MPC_GEN_DIR}/acados_solver_px4_cycle_mpc.h")
  message(STATUS "px4_drone: compilando takeoff_hold_land_mpc (acados en ${ACADOS_DIR})")
  add_executable(takeoff_hold_land_mpc
    src/takeoff_hold_land_mpc.cpp
    src/takeoff_position_hold_base.cpp
    src/obstacle_guard.cpp
    src/mpc_cycle.cpp)
  target_include_directories(takeoff_hold_land_mpc PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${MPC_GEN_DIR}
    ${ACADOS_DIR}/include
    ${ACADOS_DIR}/include/acados
    ${ACADOS_DIR}/include/blasfeo/include
    ${ACADOS_DIR}/include/hpipm/include)
  target_link_libraries(takeoff_hold_land_mpc
    ${MPC_GEN_DIR}/libacados_ocp_solver_px4_cycle_mpc.so
    ${ACADOS_DIR}/lib/libacados.so)
  set_target_properties(takeoff_hold_land_mpc PROPERTIES
    BUILD_RPATH "${MPC_GEN_DIR};${ACADOS_DIR}/lib"
    INSTALL_RPATH "${MPC_GEN_DIR};${ACADOS_DIR}/lib")
  ament_target_dependencies(takeoff_hold_land_mpc rclcpp px4_msgs sensor_msgs geometry_msgs std_msgs)
  install(TARGETS takeoff_hold_land_mpc DESTINATION lib/${PROJECT_NAME})
  # Las dependencias transitivas de acados necesitan la ruta al cargar ROS.
  ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/acados.dsv.in")
else()
  message(WARNING
    "px4_drone: NO se compila takeoff_hold_land_mpc. "
    "Falta acados en ${ACADOS_DIR} o el solver generado en ${MPC_GEN_DIR} "
    "(generalo con: python3 scripts/mpc/generate_cycle_mpc.py)")
endif()

# --- Nodo CTBR: NMPC con actitud que publica tasas + empuje (opcional) ---
# Mismo criterio que el anterior, con su propio solver:
#     python3 scripts/mpc/generate_ctbr_mpc.py
set(CTBR_GEN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/scripts/mpc/generated_ctbr/c_generated_code")
if(EXISTS "${ACADOS_DIR}/lib" AND EXISTS "${CTBR_GEN_DIR}/acados_solver_px4_ctbr_mpc.h")
  message(STATUS "px4_drone: compilando takeoff_hold_land_ctbr (acados en ${ACADOS_DIR})")
  add_executable(takeoff_hold_land_ctbr
    src/takeoff_hold_land_ctbr.cpp
    src/takeoff_position_hold_base.cpp
    src/obstacle_guard.cpp
    src/mpc_ctbr.cpp)
  target_include_directories(takeoff_hold_land_ctbr PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CTBR_GEN_DIR}
    ${ACADOS_DIR}/include
    ${ACADOS_DIR}/include/acados
    ${ACADOS_DIR}/include/blasfeo/include
    ${ACADOS_DIR}/include/hpipm/include)
  target_link_libraries(takeoff_hold_land_ctbr
    ${CTBR_GEN_DIR}/libacados_ocp_solver_px4_ctbr_mpc.so
    ${ACADOS_DIR}/lib/libacados.so)
  set_target_properties(takeoff_hold_land_ctbr PROPERTIES
    BUILD_RPATH "${CTBR_GEN_DIR};${ACADOS_DIR}/lib"
    INSTALL_RPATH "${CTBR_GEN_DIR};${ACADOS_DIR}/lib")
  ament_target_dependencies(takeoff_hold_land_ctbr rclcpp px4_msgs sensor_msgs geometry_msgs std_msgs)
  install(TARGETS takeoff_hold_land_ctbr DESTINATION lib/${PROJECT_NAME})
  ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/acados.dsv.in")
else()
  message(WARNING
    "px4_drone: NO se compila takeoff_hold_land_ctbr. "
    "Falta acados en ${ACADOS_DIR} o el solver generado en ${CTBR_GEN_DIR} "
    "(generalo con: python3 scripts/mpc/generate_ctbr_mpc.py)")
endif()

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
