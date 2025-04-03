cmake_minimum_required(VERSION 3.8)
project(tracker_params)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(yaml-cpp REQUIRED)
if (NOT yaml-cpp_FOUND)
    message(FATAL_ERROR "Not found yaml-cpp, you should install yaml-cpp")
endif ()


include_directories(${YAML_CPP_INCLUDE_DIR})
# Include C++ directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)
include_directories(/usr/local/include/yaml-cpp)

# Declare C++ executables
add_executable(founder src/founder_node.cpp)
add_executable(tracker src/tracker_node.cpp)

# Link libraries
ament_target_dependencies(founder rclcpp std_msgs ament_index_cpp)
ament_target_dependencies(tracker rclcpp std_msgs)

target_link_libraries(founder ${YAML_CPP_LIBRARIES})
target_link_libraries(tracker ${YAML_CPP_LIBRARIES})

# Install C++ executables
install(TARGETS founder tracker
  DESTINATION lib/${PROJECT_NAME})

# Install configuration files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
        FILES_MATCHING PATTERN "*.yaml"
        PATTERN ".gitignore" EXCLUDE
)

# Install include directory (if any)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
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
