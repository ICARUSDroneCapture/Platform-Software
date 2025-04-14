set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
cmake_minimum_required(VERSION 3.10.0)
project(icarus_arm_control)

# Compiler options for GCC and Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_PREFIX_PATH "/home/icarus/ros2_ws/install")
set(INERTIAL_LIB_PATH "/home/icarus/ros2_ws/src/Platform-Software/inertial-sense-sdk/build/libInertialSenseSDK.a")

# Find necessary packages
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(inertial_sense_ros2 REQUIRED)
find_package(odrive_can REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(Matplot++ REQUIRED)
find_package(Threads)

# Generate ROS2 interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/PublishData.msg"
    "msg/IntegratedAngles.msg"
    DEPENDENCIES std_msgs geometry_msgs diagnostic_msgs builtin_interfaces
)

# Export necessary directories and dependencies
ament_export_include_directories(include)
ament_export_dependencies(rclcpp sensor_msgs inertial_sense_ros2 odrive_can geometry_msgs std_msgs diagnostic_msgs rosidl_default_runtime)

# Include directories
include_directories(include)

# Build using C11 and CXX20
set_property(TARGET ${PROJECT_NAME} PROPERTY C_STANDARD 11)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 20)
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fms-extensions -Wl,--no-as-needed -DPLATFORM_IS_LINUX")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fms-extensions -Wl,--no-as-needed -DPLATFORM_IS_LINUX")

# Declare the typesupport target once
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

# Controller node
add_executable(controller_node 
    src/controller.cpp
    src/main.cpp
    src/ControlHelper.cpp
    src/ISUtilities.cpp
)
ament_target_dependencies(controller_node rclcpp inertial_sense_ros2 odrive_can std_msgs geometry_msgs nav_msgs diagnostic_msgs sensor_msgs std_srvs)
target_link_libraries(controller_node "${cpp_typesupport_target}" Matplot++::matplot ${INERTIAL_LIB_PATH} ${YAML_CPP_LIBRARIES} pthread)

# Data logging node
add_executable(logger_node 
    src/logger.cpp
    src/ISUtilities.cpp
)
ament_target_dependencies(logger_node rclcpp inertial_sense_ros2 odrive_can std_msgs geometry_msgs nav_msgs diagnostic_msgs sensor_msgs std_srvs)
target_link_libraries(logger_node "${cpp_typesupport_target}" Matplot++::matplot ${INERTIAL_LIB_PATH} ${YAML_CPP_LIBRARIES} pthread)

# Integrator node
add_executable(integrator_node src/imu_pimu_listener.cpp)
ament_target_dependencies(integrator_node rclcpp sensor_msgs inertial_sense_ros2 odrive_can geometry_msgs nav_msgs diagnostic_msgs std_srvs)
target_link_libraries(integrator_node "${cpp_typesupport_target}" Matplot++::matplot ${INERTIAL_LIB_PATH} ${YAML_CPP_LIBRARIES} pthread)

#Gain input node
add_executable(gain_input_node src/gain_input.cpp)
ament_target_dependencies(gain_input_node rclcpp std_msgs)
target_link_libraries(gain_input_node "${cpp_typesupport_target}" Matplot++::matplot ${INERTIAL_LIB_PATH} ${YAML_CPP_LIBRARIES} pthread)


# Install targets
install(TARGETS
    controller_node
    logger_node
    integrator_node
    gain_input_node
    DESTINATION lib/${PROJECT_NAME})

ament_package()
