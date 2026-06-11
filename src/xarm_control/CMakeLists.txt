cmake_minimum_required(VERSION 3.8)
project(xarm_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ──────────────────────────────────────────────────────────────
find_package(ament_cmake               REQUIRED)
find_package(rclcpp                    REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools       REQUIRED)
find_package(moveit_kinematics         REQUIRED)
find_package(geometry_msgs             REQUIRED)
find_package(control_msgs              REQUIRED)
find_package(std_srvs                  REQUIRED)
find_package(moveit_msgs               REQUIRED)
find_package(tf2                       REQUIRED)
find_package(tf2_geometry_msgs         REQUIRED)
find_package(std_msgs                  REQUIRED)
find_package(xarm_msgs                 REQUIRED)

set(COMMON_DEPS
  rclcpp
  geometry_msgs
  control_msgs
  std_srvs
  moveit_msgs
  moveit_ros_planning_interface
  moveit_visual_tools
  moveit_kinematics
  tf2
  tf2_geometry_msgs
  xarm_msgs
  std_msgs
)

# ── Shared library (all class implementations) ────────────────────────────────
add_library(xarm_control_lib STATIC
  src/xarm_control.cpp
)
target_compile_features(xarm_control_lib PUBLIC c_std_99 cxx_std_17)
target_include_directories(xarm_control_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(xarm_control_lib ${COMMON_DEPS})

# ── cartesian_control executable ─────────────────────────────────────────────
add_executable(cartesian_control src/cartesian_control.cpp)
target_compile_features(cartesian_control PUBLIC c_std_99 cxx_std_17)
target_include_directories(cartesian_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(cartesian_control xarm_control_lib)
ament_target_dependencies(cartesian_control ${COMMON_DEPS})

# ── moveit_servo_keyboard executable ─────────────────────────────────────────
add_executable(moveit_servo_keyboard src/moveit_servo_node.cpp)
target_compile_features(moveit_servo_keyboard PUBLIC c_std_99 cxx_std_17)
target_include_directories(moveit_servo_keyboard PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(moveit_servo_keyboard xarm_control_lib)
ament_target_dependencies(moveit_servo_keyboard ${COMMON_DEPS})

# ── free_control_node executable ─────────────────────────────────────────────
add_executable(free_control_node src/free_control_node.cpp)
target_compile_features(free_control_node PUBLIC c_std_99 cxx_std_17)
target_include_directories(free_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(free_control_node xarm_control_lib)
ament_target_dependencies(free_control_node ${COMMON_DEPS})

# ── Install ───────────────────────────────────────────────────────────────────
install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  xarm_control_lib
  cartesian_control
  moveit_servo_keyboard
  free_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# ── Tests ─────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
