cmake_minimum_required(VERSION 3.8)
project(image_transport_wrapper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)
add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/WrapperPublisher.cpp
)

set(dependencies
  rclcpp
  image_transport
  cv_bridge
  sensor_msgs
  std_msgs
  OpenCV
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})

# add the publisher example
add_executable(wrapper_publisher src/wrapper_publisher.cpp)
ament_target_dependencies(wrapper_publisher ${dependencies})
target_link_libraries(wrapper_publisher ${PROJECT_NAME})


# add the subscriber example
add_executable(image_subscriber src/image_subscriber.cpp)
target_link_libraries(image_subscriber
  cv_bridge::cv_bridge
  image_transport::image_transport
  opencv_highgui
  rclcpp::rclcpp
)

# add the subscriber example
add_executable(image_publisher src/image_publisher.cpp)
target_link_libraries(image_publisher
  cv_bridge::cv_bridge
  image_transport::image_transport
  opencv_highgui
  rclcpp::rclcpp
)

install(TARGETS wrapper_publisher image_subscriber image_publisher
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()
