cmake_minimum_required(VERSION 3.5)
project(bitrate_topic_comparator)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Encuentra dependencias
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(coded_interfaces REQUIRED)

# Incluye los directorios
include_directories(include)

# Declara un ejecutable de C++ para el nodo
add_executable(bitrate_topic_comparator src/bitrate_topic_comparator.cpp)
ament_target_dependencies(bitrate_topic_comparator rclcpp std_msgs image_transport cv_bridge OpenCV coded_interfaces)

# Instala el nodo ejecutable
install(TARGETS
  bitrate_topic_comparator
  DESTINATION lib/${PROJECT_NAME}
)

# Configuraciones de ament para auto-generar entornos y dependencias
ament_package()
