cmake_minimum_required(VERSION 3.8)
project(imu_visualize)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Euler.msg"
  DEPENDENCIES std_msgs
)


install(FILES
  "package.xml"
  "imu_visualize/pg_euler_plot.py"
  DESTINATION "share/${PROJECT_NAME}"
)

install(DIRECTORY
  "imu_visualize/"
  DESTINATION "lib/python3.10/site-packages/imu_visualize"
)

install(DIRECTORY
  "launch"
  DESTINATION "share/${PROJECT_NAME}/"
)

install(DIRECTORY
  "config"
  DESTINATION "share/${PROJECT_NAME}/"
)

install(PROGRAMS
  "imu_visualize/imu_odometry_node.py"
  DESTINATION "lib/${PROJECT_NAME}"
)

ament_package()
