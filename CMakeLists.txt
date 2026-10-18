cmake_minimum_required(VERSION 3.8)
project(rabbits_interface)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rabbits_msgs REQUIRED)
find_package(rabcl REQUIRED)

include_directories(include)

# Executables
add_executable(gamepad_node
  src/gamepad_node.cpp
)
ament_target_dependencies(gamepad_node
  rclcpp
  sensor_msgs
  rabbits_msgs
  rabcl
)

add_executable(game_client_node
  src/game_client_node.cpp
)
ament_target_dependencies(game_client_node
  rclcpp
  sensor_msgs
  rabbits_msgs
  rabcl
)

install(TARGETS
  gamepad_node
  game_client_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # Libraries for tests (main() excluded via preprocessor)
  add_library(gamepad_lib STATIC src/gamepad_node.cpp)
  ament_target_dependencies(gamepad_lib rclcpp sensor_msgs rabbits_msgs rabcl)
  target_compile_definitions(gamepad_lib PRIVATE RABBITS_INTERFACE_EXCLUDE_MAIN)

  add_library(game_client_lib STATIC src/game_client_node.cpp)
  ament_target_dependencies(game_client_lib rclcpp sensor_msgs rabbits_msgs rabcl)
  target_compile_definitions(game_client_lib PRIVATE RABBITS_INTERFACE_EXCLUDE_MAIN)

  ament_add_gtest(test_game_client test/test_game_client.cpp)
  target_link_libraries(test_game_client game_client_lib)

  ament_add_gtest(test_gamepad test/test_gamepad.cpp)
  target_link_libraries(test_gamepad gamepad_lib)
endif()

ament_package()
