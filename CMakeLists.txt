cmake_minimum_required(VERSION 3.8)
project(ik_solver)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
# find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)

# Füge das Unterverzeichnis hinzu, das das untergeordnete Paket enthält
find_package(ik_interfaces)





# rosidl_generate_interfaces(${PROJECT_NAME}
#   "interfaces/msg/JointState.msg"
#   "interfaces/srv/CalcIK.srv"
#   DEPENDENCIES geometry_msgs
# )


# add iclude directories
include_directories(
  include/src   # todo: hat das einen effekt?
  include/ik_solver
  include/ik_interfaces
  ${EIGEN3_INCLUDE_DIRS}
  # /home/flehm/opw_kinematics/include
  
)


# Add executable
add_executable(ik_solver_node src/ik_solver.cpp)
# add_executable(${PROJECT_NAME}_node src/ik_solver_node.cpp)

# Add include directory
target_include_directories(
  ik_solver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${EIGEN3_INCLUDE_DIRS}
)

# Add dependencies to executable
ament_target_dependencies(
  # ${PROJECT_NAME}_node
ik_solver_node
rclcpp 
geometry_msgs
Eigen3
ik_interfaces
)

# We add it to be able to use other modules of the scripts folder
install(DIRECTORY
  src
  DESTINATION share/${PROJECT_NAME}
)

# Install executable
install(TARGETS 
ik_solver_node
  DESTINATION lib/${PROJECT_NAME}
)

# Export package configuration
ament_export_dependencies(
  rclcpp 
  geometry_msgs
  Eigen3)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()



