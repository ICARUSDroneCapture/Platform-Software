set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
cmake_minimum_required(VERSION 3.10.0)
project(icarus_arm_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#find_package(rosidl_typesupport_c REQUIRED)
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
find_package(std_srvs REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(Matplot++ REQUIRED)
find_package(Threads)

rosidl_generate_interfaces(${PROJECT_NAME}
#    "msg/GTime.msg"
#    "msg/SatInfo.msg"
#    "msg/GPS.msg"
#    "msg/GPSInfo.msg"
#    "msg/PIMU.msg"
#    "msg/RTKInfo.msg"
#    "msg/RTKRel.msg"
#    "msg/GlonassEphemeris.msg"
#    "msg/GNSSEphemeris.msg"
#    "msg/GNSSObservation.msg"
#    "msg/GNSSObsVec.msg"
#    "msg/INL2States.msg"
#    "msg/DIDINS2.msg"
#    "msg/DIDINS1.msg"
#    "msg/DIDINS4.msg"
#    "srv/FirmwareUpdate.srv"
#    "srv/RefLLAUpdate.srv"
#    "msg/ControlMessage.msg"
#    "msg/ControllerStatus.msg"
#    "msg/ODriveStatus.msg"
#    "srv/AxisState.srv"
    "msg/PublishData.msg"
    "msg/IntegratedAngles.msg"
    DEPENDENCIES std_msgs geometry_msgs diagnostic_msgs builtin_interfaces
)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp sensor_msgs inertial_sense_ros2 odrive_can geometry_msgs std_msgs diagnostic_msgs rosidl_default_runtime icarus_arm_control)

# We don't need the example projects
set(IGNORE_EXAMPLE_PROJECTS 1)

include_directories(include)

# Build using C11 and CXX20
set_property(TARGET ${PROJECT_NAME} PROPERTY C_STANDARD 11)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 20)
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fms-extensions -Wl,--no-as-needed -DPLATFORM_IS_LINUX" )
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fms-extensions -Wl,--no-as-needed -DPLATFORM_IS_LINUX")

# Custom controller node
add_executable(controller_node 
    src/controller.cpp
    src/main.cpp
    src/ControlHelper.cpp
    src/ISUtilities.cpp
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(controller_node "${cpp_typesupport_target}")
ament_target_dependencies(controller_node rclcpp inertial_sense_ros2 odrive_can std_msgs geometry_msgs nav_msgs diagnostic_msgs sensor_msgs std_srvs)
target_link_libraries(controller_node Matplot++::matplot ${YAML_CPP_LIBRARIES} pthread)

# Data logging node
add_executable(logger_node 
    src/logger.cpp
    src/ISUtilities.cpp
)

target_link_libraries(logger_node "${cpp_typesupport_target}")
ament_target_dependencies(logger_node rclcpp inertial_sense_ros2 odrive_can std_msgs geometry_msgs nav_msgs diagnostic_msgs sensor_msgs std_srvs)
target_link_libraries(logger_node Matplot++::matplot ${YAML_CPP_LIBRARIES} pthread)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

add_executable(integrator_node src/imu_pimu_listener.cpp)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

target_link_libraries(integrator_node "${cpp_typesupport_target}")

ament_target_dependencies(integrator_node
    rclcpp
    sensor_msgs
    inertial_sense_ros2
    odrive_can
    geometry_msgs
    nav_msgs
    diagnostic_msgs
    std_srvs
)

target_link_libraries(integrator_node  Matplot++::matplot ${YAML_CPP_LIBRARIES} pthread)

install(TARGETS
    controller_node
    logger_node
    integrator_node
    DESTINATION lib/${PROJECT_NAME})

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

ament_package()
