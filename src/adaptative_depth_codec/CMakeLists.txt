cmake_minimum_required(VERSION 3.8)
project(adaptative_depth_codec)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(coded_interfaces REQUIRED)


include_directories(
  include  
)


# Modifica esta línea para incluir ambos archivos en el mismo ejecutable
add_executable(adaptative_depth_codec_server src/adaptative_depth_codec_server.cpp src/utils/adaptative_depth_codec.cpp)
add_executable(adaptative_depth_codec_client src/adaptative_depth_codec_client.cpp src/utils/adaptative_depth_codec.cpp)

ament_target_dependencies(adaptative_depth_codec_server rclcpp sensor_msgs cv_bridge OpenCV coded_interfaces)
ament_target_dependencies(adaptative_depth_codec_client rclcpp sensor_msgs cv_bridge OpenCV coded_interfaces)


install(TARGETS
  adaptative_depth_codec_server
  adaptative_depth_codec_client
  DESTINATION lib/${PROJECT_NAME})

target_link_libraries(adaptative_depth_codec_server curl)
target_link_libraries(adaptative_depth_codec_client curl)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()