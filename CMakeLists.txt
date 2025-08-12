cmake_minimum_required(VERSION 3.8)
project(rosout_mcp_usecase)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)

# Create trigger_processor executable
add_executable(trigger_processor src/trigger_processor.cpp)
ament_target_dependencies(trigger_processor rclcpp std_srvs)

# Create sub_processor executable
add_executable(sub_processor src/sub_processor.cpp)
ament_target_dependencies(sub_processor rclcpp std_srvs)

# Install executables
install(TARGETS
  trigger_processor
  sub_processor
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
