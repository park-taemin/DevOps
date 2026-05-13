
# 🚀 ROS 2 Launch 실습 과제 1 (Python)

본 프로젝트는 ROS 2에서 Python을 사용하여 Publisher와 Subscriber 노드를 구현하고, `launch` 파일을 통해 다중 노드를 서로 다른 네임스페이스(`namespace`)에서 동시에 실행하고 관리하는 실습입니다.

## 1. 프로젝트 목표
- `rclpy`를 활용한 토픽 통신(Topic Communication) 이해.
- `launch` 파일을 이용한 노드 실행 자동화 및 네임스페이스 분리 학습.
- `ament_cmake` 환경에서 Python 스크립트 빌드 및 설치 설정 숙달.

## 2. 패키지 구조
```text
launch_example/
├── CMakeLists.txt          # 패키지 빌드 및 설치 설정
├── package.xml             # 의존성 및 패키지 메타데이터
├── launch/
│   └── my_launch.py        # 다중 노드 실행을 위한 런치 파일
└── src/
    ├── pub.py              # "Hello world! (Python)" 메시지 발행 노드
    └── sub.py              # 메시지 수신 및 로그 출력 노드
3. 코드 구성 설명
🔹 [pub.py] Publisher
Topic: hello_topic

Message Type: std_msgs/msg/String

주기: 1.0초 마다 "Hello world! (Python)" 발행

🔹 [sub.py] Subscriber
Topic: hello_topic 구독

기능: 수신된 데이터를 터미널에 Received message 형식으로 출력

🔹 [my_launch.py] Launch File
두 개의 네임스페이스(launch_example1, launch_example2)를 생성합니다.

각 네임스페이스마다 pub와 sub 노드를 한 쌍씩 실행하여 총 4개의 노드가 구동됩니다.

네임스페이스 분리를 통해 동일한 토픽 이름을 사용하더라도 데이터 흐름이 서로 간섭받지 않도록 설계되었습니다.

4. 빌드 및 실행 가이드
1) 워크스페이스 빌드
Bash
# 워크스페이스 이동
$ cd ~/ros2_ws/

# 패키지 빌드
$ colcon build --symlink-install --packages-select launch_example

# 환경 변수 로드
$ source install/local_setup.bash
2) 실행
Bash
# Launch 파일을 이용한 전체 노드 실행
$ ros2 launch launch_example my_launch.py
5. 결과 확인 방법
노드 및 토픽 리스트 확인
실행 중인 터미널 외에 새 터미널을 열고 다음 명령어를 입력합니다.

Bash
# 실행 중인 노드 확인 (네임스페이스 포함)
$ ros2 node list

# 실행 중인 토픽 확인
$ ros2 topic list
시각화 (rqt_graph)
노드 간의 관계와 네임스페이스 분리 구조를 시각적으로 확인합니다.

Bash
$ rqt_graph
rqt_graph 상에서 동일한 구조의 통신망이 두 그룹(launch_example1, launch_example2)으로 나뉘어 동작하는 것을 볼 수 있습니다.


---

### 💡 팁: GitHub 업로드 시 참고
* **실행 권한**: `src/pub.py`와 `src/sub.py` 파일에 실행 권한이 있어야 합니다. (`chmod +x src/*.py`)
* **이미지 추가**: 실습 결과물인 `rqt_graph` 캡처 사진을 GitHub 저장소에 올리고, 위 `README.md` 하단에 이미지 링크를 추가하면 더욱 완벽한 과제 제출물이 됩니다.
