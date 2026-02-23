cmake_minimum_required(VERSION 3.8)
project(robin_hardware_opcua)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robin_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)

# Find open62541 installed library
find_package(open62541 REQUIRED)

# OPC UA Bridge node (generic, config-driven)
add_executable(opcua_bridge_node
  src/opcua_bridge_node.cpp
)
target_link_libraries(opcua_bridge_node
  open62541::open62541
  yaml-cpp
)
ament_target_dependencies(opcua_bridge_node
  rclcpp
  std_srvs
  std_msgs
  robin_interfaces
)

# Install targets
install(TARGETS opcua_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
