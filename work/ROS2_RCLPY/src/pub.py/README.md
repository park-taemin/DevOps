
# ROS2 Launch 실습 과제

## 프로젝트 소개

ROS2 Launch 시스템을 이용하여  
여러 노드를 동시에 실행하고, Launch 파일의 다양한 기능을 학습하는 실습입니다.

실습 내용:
- 실습과제 1 : Python 기반 Publisher / Subscriber Launch 실습
- 실습과제 3 : SLAMTEC LiDAR Launch 파일 분석

---

# 개발 환경

- Ubuntu 24.04
- ROS2 Jazzy
- Python3
- rclpy

---

# 실습과제 1

# ROS2 Launch Example (Python)

## 프로젝트 구조

```bash
launch_example/
├── launch/
│   └── my_launch.py
├── src/
│   ├── pub.py
│   └── sub.py
├── CMakeLists.txt
└── package.xml
```

---

# 실행 흐름

```text
Publisher Node
    ↓
hello_topic
    ↓
Subscriber Node
```

Publisher 노드는 `"Hello world! (Python)"` 메시지를 발행하고  
Subscriber 노드는 해당 메시지를 수신하여 출력합니다.

---

# Launch File

## my_launch.py

Launch 파일을 사용하여 여러 노드를 동시에 실행합니다.

또한 namespace를 이용하여 두 개의 그룹을 독립적으로 실행합니다.

- launch_example1
- launch_example2

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # 첫 번째 그룹
        Node(
            package='launch_example',
            namespace='launch_example1',
            executable='pub.py',
            name='pub'
        ),

        Node(
            package='launch_example',
            namespace='launch_example1',
            executable='sub.py',
            name='sub'
        ),

        # 두 번째 그룹
        Node(
            package='launch_example',
            namespace='launch_example2',
            executable='pub.py',
            name='pub'
        ),

        Node(
            package='launch_example',
            namespace='launch_example2',
            executable='sub.py',
            name='sub'
        )
    ])
```

---

# Publisher Node

## pub.py

### 주요 기능

- `hello_topic` 토픽 생성
- `"Hello world! (Python)"` 메시지 발행
- 1초 주기로 반복 실행

### 핵심 코드

```python
publisher = node.create_publisher(
    String,
    'hello_topic',
    10
)
```

### Timer 설정

```python
timer = node.create_timer(
    1.0,
    timer_callback
)
```

1초마다 메시지를 발행합니다.

---

# Subscriber Node

## sub.py

### 주요 기능

- `hello_topic` 토픽 구독
- 수신한 메시지 출력

### 핵심 코드

```python
subscription = node.create_subscription(
    String,
    'hello_topic',
    listener_callback,
    10
)
```

메시지를 수신하면 callback 함수가 실행됩니다.

---

# CMakeLists.txt

Python 노드와 launch 파일을 install 디렉터리에 설치하도록 설정했습니다.

```cmake
install(PROGRAMS
  src/pub.py
  src/sub.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
```

---

# package.xml

Launch 기능 사용을 위해 `ros2launch` 의존성을 추가했습니다.

```xml
<exec_depend>ros2launch</exec_depend>
```

---

# 빌드 방법

```bash
cd ~/ros2_ws

colcon build --packages-select launch_example

source install/setup.bash
```

---

# 실행 방법

## Launch 파일 실행

```bash
ros2 launch launch_example my_launch.py
```

---

# 실행 결과

```bash
[INFO] [pub]: Publishing: "Hello world! (Python)"
[INFO] [sub]: Received message: "Hello world! (Python)"
```

두 개의 namespace 그룹이 동시에 실행됩니다.

예시:

```bash
/launch_example1/hello_topic
/launch_example2/hello_topic
```

---

# 실습과제 3

# SLAMTEC LiDAR Launch 파일 분석

## 프로젝트 개요

`sllidar_ros2` 패키지의 launch 파일을 분석하여  
ROS2 Launch 시스템에서 사용되는 다양한 기능을 확인하는 실습입니다.

해당 launch 파일은:

- LiDAR 노드 실행
- RViz 자동 실행
- 파라미터 설정
- 명령행 인자 설정
- LaunchConfiguration 사용

등의 기능을 포함하고 있습니다.

---

# LaunchConfiguration

```python
channel_type = LaunchConfiguration(
    'channel_type',
    default='serial'
)
```

LaunchConfiguration은 launch 파일 실행 시 외부에서 값을 전달받기 위한 변수입니다.

예시:

```bash
ros2 launch sllidar_ros2 view_sllidar_c1_launch.py \
serial_port:=/dev/ttyUSB1
```

---

# 주요 파라미터 설명

| 파라미터 | 설명 | 기본값 |
|---|---|---|
| channel_type | LiDAR 연결 방식 | serial |
| serial_port | 연결된 USB 포트 | /dev/ttyUSB0 |
| serial_baudrate | 통신 속도 | 460800 |
| frame_id | TF 프레임 이름 | laser |
| inverted | 스캔 데이터 반전 여부 | false |
| angle_compensate | 각도 보정 사용 여부 | true |
| scan_mode | 스캔 모드 | Standard |

---

# DeclareLaunchArgument

```python
DeclareLaunchArgument(
    'serial_port',
    default_value=serial_port,
    description='Specifying usb port'
)
```

사용자가 launch 실행 시 값을 변경할 수 있도록 인자를 선언하는 부분입니다.

---

# RViz 설정 파일 경로 지정

```python
rviz_config_dir = os.path.join(
    get_package_share_directory('sllidar_ros2'),
    'rviz',
    'sllidar_ros2.rviz'
)
```

패키지 내부의 RViz 설정 파일 경로를 자동으로 가져오는 코드입니다.

---

# LiDAR Node 실행

```python
Node(
    package='sllidar_ros2',
    executable='sllidar_node',
    name='sllidar_node',
)
```

LiDAR 드라이버 노드를 실행하여 LaserScan 데이터를 발행합니다.

---

# parameters 설정

```python
parameters=[{
    'channel_type': channel_type,
    'serial_port': serial_port,
    'serial_baudrate': serial_baudrate,
    'frame_id': frame_id,
    'inverted': inverted,
    'angle_compensate': angle_compensate,
    'scan_mode': scan_mode
}]
```

Node 실행 시 사용할 파라미터를 전달합니다.

예시:
- USB 포트 설정
- 통신 속도 설정
- 스캔 보정 설정
- 프레임 이름 설정

---

# RViz 실행

```python
Node(
    package='rviz2',
    executable='rviz2',
    arguments=['-d', rviz_config_dir]
)
```

RViz를 자동 실행하고 저장된 설정 파일을 불러옵니다.

---

# Launch 파일 실행 방법

```bash
ros2 launch sllidar_ros2 view_sllidar_c1_launch.py
```

---

# 명령행 인자 변경 예시

## USB 포트 변경

```bash
ros2 launch sllidar_ros2 view_sllidar_c1_launch.py \
serial_port:=/dev/ttyUSB1
```

---

## Baudrate 변경

```bash
ros2 launch sllidar_ros2 view_sllidar_c1_launch.py \
serial_baudrate:=115200
```

---

## Scan Mode 변경

```bash
ros2 launch sllidar_ros2 view_sllidar_c1_launch.py \
scan_mode:=Express
```

---

# 정리

이번 실습을 통해 다음 내용을 학습하였다.

- ROS2 Launch 시스템
- Publisher / Subscriber 구조
- Namespace 설정
- LaunchConfiguration
- DeclareLaunchArgument
- parameter 전달
- RViz 자동 실행
- 명령행 인자 설정

ROS2 Launch 시스템을 사용하면  
복잡한 노드 실행과 설정을 효율적으로 관리할 수 있음을 확인하였다.
