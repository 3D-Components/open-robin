cmake_minimum_required(VERSION 3.8)
project(robin_moveit_config)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install all configuration directories
install(DIRECTORY config srdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
