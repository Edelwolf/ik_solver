cmake_minimum_required(VERSION 3.8)
project(ik_solver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(interfaces_for_ik_service REQUIRED)
find_package(rclcpp REQUIRED)

# add iclude directories
include_directories(
  include/ik_solver
  # /home/flehm/opw_kinematics/include
)


# Add executable
add_executable(ik_solver src/ik_solver.cpp)

# Add include directory
target_include_directories(ik_solver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# Add dependencies to executable
ament_target_dependencies(
ik_solver 
rclcpp 
interfaces_for_ik_service)

# Install executable
install(TARGETS ik_solver
  DESTINATION lib/${PROJECT_NAME}
)

# Export package configuration
ament_export_dependencies(
rclcpp 
interfaces_for_ik_service)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()



