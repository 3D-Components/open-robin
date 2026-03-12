cmake_minimum_required(VERSION 3.8)
project(robin_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PlanExperiment.srv"
  "srv/ApproveExperimentPlan.srv"
  "srv/SetFloat32.srv"
  "srv/SetInt32.srv"
  "srv/SetUInt8.srv"
  "srv/SetTcpMode.srv"
  "srv/StartWeld.srv"
  "srv/FindSurface.srv"
  "srv/CalibrateStickout.srv"
  "srv/CalibratePlatePlane.srv"
  "msg/WelderData.msg"
  "msg/BeadGeometry.msg"
  "msg/WeldProgression.msg"
  "msg/FroniusSample.msg"
  "msg/ActiveBead.msg"
  "msg/ExperimentBeadSpec.msg"
  "msg/ExperimentBead.msg"
  "msg/WeldPlate.msg"
  "msg/PlateLayout.msg"
  "msg/WeldBead.msg"
  "action/WeldExperiment.action"
  "action/ExecuteBead.action"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs sensor_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
