cmake_minimum_required(VERSION 3.5)

project(canbus VERSION 0.1.0)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
add_library(libcontrolcan SHARED IMPORTED)
set_target_properties(libcontrolcan PROPERTIES IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/lib/libcontrolcan.so)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(autoware_control_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(autoware_vehicle_msgs REQUIRED)
find_package(tier4_vehicle_msgs REQUIRED)
find_package(autoware_adapi_v1_msgs REQUIRED)
find_package(mycan_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ars408_msg REQUIRED)
include_directories(include)


add_executable(canbus_node src/canbus2_node.cpp)


target_link_libraries(canbus_node libcontrolcan)
ament_target_dependencies(canbus_node rclcpp std_msgs autoware_control_msgs builtin_interfaces  tier4_vehicle_msgs autoware_vehicle_msgs autoware_adapi_v1_msgs  sensor_msgs visualization_msgs mycan_msgs ars408_msg)





install(TARGETS
  canbus_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
