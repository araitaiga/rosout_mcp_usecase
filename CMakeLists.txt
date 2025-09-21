cmake_minimum_required(VERSION 3.8)
project(rosout_mcp_usecase)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Create trigger_processor executable
ament_auto_add_executable(trigger_processor src/trigger_processor.cpp)

# Create sub_processor executable
ament_auto_add_executable(sub_processor src/sub_processor.cpp)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()
