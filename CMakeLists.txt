cmake_minimum_required(VERSION 3.8)
project(unitree_actuator_sdk_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorCmd.msg"
  "msg/MotorData.msg"
)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
add_executable(motor src/motor.cpp)
ament_target_dependencies(motor rclcpp)

target_link_libraries(motor ${PROJECT_SOURCE_DIR}/lib/libUnitreeMotorSDK_Arm64.so)

target_link_libraries(motor "${cpp_typesupport_target}")
install(TARGETS
  motor
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY
  ${PROJECT_SOURCE_DIR}/lib/
  DESTINATION lib/
)

ament_package()
