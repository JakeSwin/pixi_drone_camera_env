cmake_minimum_required(VERSION 3.5)
project(uav_image)

# macOS RPATH fix
if(APPLE)
  set(CMAKE_MACOSX_RPATH ON)
  set(CMAKE_INSTALL_RPATH "@loader_path;@loader_path/../lib")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(uav_image src/uav_image.cpp)
ament_target_dependencies(uav_image rclcpp
OpenCV
cv_bridge
image_transport
sensor_msgs)

install(TARGETS uav_image
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
