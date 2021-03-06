cmake_minimum_required(VERSION 3.5)
project(path_plan_manager)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
    include
    ${Eigen3_INCLUDE_DIRS}
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)

find_package(path_planning REQUIRED)

# build
add_executable(planning_node 
              src/planning_node.cpp
              src/astar_replan_fsm.cpp
              src/planner_manager.cpp)
ament_target_dependencies(planning_node ament_cmake rclcpp std_msgs Eigen3 path_planning)

# install 
install(TARGETS planning_node 
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()