cmake_minimum_required(VERSION 3.5)
project(elevator_control_cui)

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

add_compile_options(-lncursesw)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(elevator_interfaces REQUIRED)
find_package(Curses REQUIRED)

set(CURSES_NEED_WIDE TRUE)
SET(CMAKE_CXX_FLAGS "-lncursesw")

include_directories(include)
include_directories(${CURSES_INCLUDE_DIR})

add_executable(elevator_control_cui
    #src/tke_controller_node.cpp
    #src/port_handler.cpp
    #src/packet_handler.cpp
    #src/ev_info.cpp
    src/main_node.cpp
    )

target_link_libraries(elevator_control_cui ${CURSES_LIBRARIES} -lncursesw)


ament_target_dependencies(elevator_control_cui
    ament_cmake
    ament_index_cpp
    rclcpp
    std_msgs
    elevator_interfaces
)


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

install(TARGETS
  elevator_control_cui
  DESTINATION lib/${PROJECT_NAME})

ament_package()
