cmake_minimum_required(VERSION 3.16)
project(launch_example)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# 파이썬 노드 파일 설치 및 실행 권한 부여
install(PROGRAMS
  src/pub.py
  src/sub.py
  DESTINATION lib/${PROJECT_NAME}
)

# 런치 파일 설치
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()