cmake_minimum_required(VERSION 3.8)
project(ball_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package( OpenCV REQUIRED )
find_package(cv_bridge REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(trace_command src/trace_command.cpp)
target_link_libraries(trace_command ${OpenCV_LIBS})
ament_target_dependencies(trace_command rclcpp sensor_msgs geometry_msgs cv_bridge)
target_include_directories(trace_command PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(trace_command PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS trace_command
  DESTINATION lib/${PROJECT_NAME})

  
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
