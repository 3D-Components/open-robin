cmake_minimum_required(VERSION 3.8)
project(robin_sim_gz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(gz-common5 REQUIRED)
find_package(sdformat14 REQUIRED)

set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})
set(GZ_TRANSPORT_VER ${gz-transport13_VERSION_MAJOR})
set(GZ_MSGS_VER ${gz-msgs10_VERSION_MAJOR})
set(GZ_MATH_VER ${gz-math7_VERSION_MAJOR})

add_library(robin_sim_plugin SHARED
  src/robin_sim_plugin.cc
)
target_include_directories(robin_sim_plugin PRIVATE include)
target_link_libraries(robin_sim_plugin
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-transport${GZ_TRANSPORT_VER}::gz-transport${GZ_TRANSPORT_VER}
  gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER}
  gz-math${GZ_MATH_VER}::gz-math${GZ_MATH_VER}
  ${sdformat14_LIBRARIES}
)
target_include_directories(robin_sim_plugin PRIVATE ${sdformat14_INCLUDE_DIRS})

install(TARGETS robin_sim_plugin
  LIBRARY DESTINATION lib
)

ament_package()