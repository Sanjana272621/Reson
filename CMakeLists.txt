cmake_minimum_required(VERSION 3.8)
project(reson)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

include_directories(include)

add_executable(csv_replay_node src/csv_replay_node.cpp)
ament_target_dependencies(csv_replay_node rclcpp std_msgs)

add_executable(fft_node src/fft_node.cpp)
ament_target_dependencies(fft_node rclcpp std_msgs)

add_executable(diagnosis_node src/diagnosis_node.cpp)
ament_target_dependencies(diagnosis_node rclcpp std_msgs diagnostic_msgs)

install(TARGETS
  csv_replay_node
  fft_node
  diagnosis_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config data
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/analysis.py
  scripts/compute_baseline.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
