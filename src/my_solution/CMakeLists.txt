cmake_minimum_required(VERSION 3.8)
project(my_solution)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(my_node targeting.cpp)

ament_target_dependencies(my_node
  rclcpp 
  std_msgs 
  sensor_msgs 
  cv_bridge 
  image_transport 
  OpenCV
)

install(TARGETS
  my_node
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
