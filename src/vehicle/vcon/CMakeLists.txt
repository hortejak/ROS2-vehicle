cmake_minimum_required(VERSION 3.8)
project(vcon)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(interfaces REQUIRED)

include_directories(include)

add_executable(vcon_publisher_cpp src/vcon_publisher_cpp.cpp)

# Link the libraries
target_link_libraries(vcon_publisher_cpp
  ament_index_cpp::ament_index_cpp
  yaml-cpp
)

ament_target_dependencies(vcon_publisher_cpp rclcpp interfaces)

install(TARGETS
  vcon_publisher_cpp
  DESTINATION lib/${PROJECT_NAME})


# Install Python modules
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  vcon/vcon_publisher_py.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  VCON.yaml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
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
