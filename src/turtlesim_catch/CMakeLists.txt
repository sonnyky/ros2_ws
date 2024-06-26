cmake_minimum_required(VERSION 3.8)
project(turtlesim_catch)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(turtlesim_catch_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

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

include_directories(${turtlesim_catch_interfaces_INCLUDE_DIRS})

add_executable(turtle_controller src/turtle_controller.cpp)
ament_target_dependencies(turtle_controller rclcpp turtlesim geometry_msgs turtlesim_catch_interfaces)

add_executable(turtle_spawner src/turtle_spawner.cpp)
ament_target_dependencies(turtle_spawner rclcpp turtlesim turtlesim_catch_interfaces)

install(TARGETS
  turtle_controller
  turtle_spawner
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
