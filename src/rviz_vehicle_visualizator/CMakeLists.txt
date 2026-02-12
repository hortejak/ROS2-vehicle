cmake_minimum_required(VERSION 3.8)
project(rviz_vehicle_visualizator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(interfaces REQUIRED)

include_directories(include)

# add cpp executables
add_executable(rviz_vehicle_visualizator_cpp src/vehicle_visualizator_cpp.cpp)

# Link the libraries
target_link_libraries(rviz_vehicle_visualizator_cpp)

ament_target_dependencies(rviz_vehicle_visualizator_cpp rclcpp interfaces)

install(TARGETS
  rviz_vehicle_visualizator_cpp
  DESTINATION lib/${PROJECT_NAME})


# Install Python modules
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  rviz_vehicle_visualizator/vehicle_visualizator_py.py
  DESTINATION lib/${PROJECT_NAME}
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
