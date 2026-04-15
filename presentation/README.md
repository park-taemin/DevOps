# 30_ROS2 장애물 회피 주행

## 프로젝트 개요

라이다(LIDAR) 센서를 활용하여 장애물을 실시간으로 감지하고 자율주행하는 시스템을 구현하였다.
총 3단계로 구성되며, 이전 단계의 결과물을 기반으로 점진적으로 발전시키는 방식으로 진행하였다.

| 단계 | 내용 |
|------|------|
| 과제 1 | 키보드로 로봇을 원격 조종하며 LIDAR 스캔 영상을 MP4로 저장 (lidarsave) |
| 과제 2 | 저장된 영상을 재생하며 장애물 회피 알고리즘을 시뮬레이션으로 검증 (lidarsim) |
| 과제 3 | 실제 LIDAR와 연결하여 장애물 회피 자율주행 구현 (lidardrive) |

RPi5(로봇)와 WSL PC(알고리즘 처리)가 동일 네트워크에서 ROS2 토픽으로 통신한다.

---

## 하드웨어 구성

| 항목 | RPi5 (로봇) | WSL PC |
|------|------------|--------|
| 보드 / OS | Raspberry Pi 5 / Ubuntu 24.04 | Windows 11 / WSL2 Ubuntu 24.04 |
| ROS2 | Jazzy | Jazzy |
| LIDAR | SLAMTEC Rplidar C1 | — |
| 모터 | 다이내믹셀 MX-12W | — |
| 통신 | Wi-Fi | Wi-Fi (WSL2 Bridge) |

---

## ROS2 토픽 구성

| 토픽 | 메시지 타입 | 발행 | 구독 |
|------|------------|------|------|
| `/scan` | `sensor_msgs/LaserScan` | `sllidar_node` (RPi5) | `sllidar_client` (WSL) |
| `vel_cmd_topic` | `geometry_msgs/Vector3` | `sllidar_client` (WSL) | `node_dxlsub` (RPi5) |

---

## 실습 과제 1 — LIDAR 스캔 영상 저장 (lidarsave)

> 키보드로 로봇을 원격 조종하면서 LIDAR 스캔 영상을 MP4로 저장

### Block Diagram

![block1 diagram](https://github.com/user-attachments/assets/b447482a-3bbc-4fd2-917e-00373826db66)


### 노드 구성

| 노드 | 패키지 | 실행 PC | 역할 |
|------|--------|---------|------|
| `sllidar_node` | sllidar_ros2 | RPi5 | LIDAR 스캔 데이터 발행 |
| `node_dxlsub` | dxl_nano | RPi5 | 속도 명령 수신 → 모터 구동 |
| `scanCb` (lidarsave) | lidarsave | WSL | 스캔 수신 → 시각화 → 영상 저장 |
| `node_dxlpub` | dxl_wsl | WSL | 키보드 입력 → 속도 명령 발행 |

### 코드 설명

#### `scanCb()` — 스캔 수신 콜백

```
/scan (LaserScan) → visualizeScan() → saveVideo() → scan_video.mp4
```

/scan 토픽을 수신하면 스캔 데이터 개수를 확인하고 시각화 및 영상 저장 함수를 순서대로 호출한다.

```cpp
if(scan->ranges.size() > 0) count = scan->ranges.size();
cv::Mat image = visualizeScan(scan, count);
saveVideo(image);
```

---

#### `visualizeScan()` — 스캔 시각화

```
LaserScan → 극좌표 → 직교좌표 변환 → 500×500 이미지 → cv::imshow → cv::Mat 반환
```

500×500 흰색 이미지를 생성하고 LIDAR 측정값을 극좌표 → 직교좌표로 변환하여 빨간 점으로 표시한다.
중심(250, 250)이 로봇 위치이며 1m = 100px 비율로 표시한다.

```cpp
cv::Mat image(500, 500, CV_8UC3, cv::Scalar(255, 255, 255));
double scale = 100.0; // 1m = 100px

int x = 250 + (int)(distance * scale * sin(angle_rad));
int y = 250 + (int)(distance * scale * cos(angle_rad));
cv::circle(image, cv::Point(x, y), 2, cv::Scalar(0, 0, 255), -1);
```

---

#### `saveVideo()` — 동영상 저장

```
cv::Mat image → VideoWriter 초기화(최초 1회) → 프레임 기록 → scan_video.mp4
```

첫 프레임 수신 시 VideoWriter를 초기화하고 이후 매 프레임을 MP4 파일에 저장한다.
LIDAR가 10Hz로 데이터를 발행하므로 영상도 10fps로 저장된다.

```cpp
if (!is_video_init) {
    video_writer.open(".../ros2_basic_30_test1.mp4",
                      cv::VideoWriter::fourcc('M','J','P','G'), 10, cv::Size(500,500), true);
    is_video_init = true;
}
video_writer.write(image);
```

---

#### `node_dxlpub` — 키보드 원격제어

```
키보드 입력 → node_dxlpub → vel_cmd_topic (Vector3) → node_dxlsub → 다이내믹셀
```

| 키 | 동작 | 왼쪽 바퀴 (vel.x) | 오른쪽 바퀴 (vel.y) |
|----|------|-------------------|---------------------|
| `f` | 전진 | +50 rpm | -50 rpm |
| `b` | 후진 | -50 rpm | +50 rpm |
| `l` | 좌회전 | -50 rpm | -50 rpm |
| `r` | 우회전 | +50 rpm | +50 rpm |
| `s` / `space` | 정지 | 0 | 0 |



---

## 실습 과제 2 — 시뮬레이션 장애물 회피 (lidarsim)

> 과제 1에서 저장한 영상을 활용하여 실제 LIDAR 없이 알고리즘을 검증

### Block Diagram

![block 2](https://github.com/user-attachments/assets/9c87b205-0c15-425a-a850-bf2182d17b21)


### 노드 구성

| 노드 | 패키지 | 실행 PC | 역할 |
|------|--------|---------|------|
| `sllidar_client` (lidarsim) | lidarsim | WSL | 영상 읽기 → 장애물 탐지 → 속도 명령 발행 |
| `node_dxlsub` | dxl_nano | RPi5 | 속도 명령 수신 → 모터 구동 |

### 코드 설명

#### `timer_callback()` — 메인 처리 루프 (100ms 주기)

```
scan_video.mp4 → 프레임 읽기 → 이진화/ROI 추출 → 장애물 탐지 → 속도 계산 → vel_cmd_topic
```

동영상에서 100ms마다 한 프레임씩 읽어 장애물 탐지 및 속도 명령을 처리한다.

```cpp
cv::Mat frame;
cap_ >> frame;
if (frame.empty()) {
    cap_.set(cv::CAP_PROP_POS_FRAMES, 0); // 영상 끝나면 처음부터 반복
    return;
}
```

---

#### `preprocess_image()` — 이진화 및 ROI 추출

```
BGR 이미지 → 그레이스케일 → 이진화(반전) → 상단 절반 ROI 반환
```

장애물(어두운 픽셀)을 흰색(255)으로 반전하고, 로봇 전방(상단 250px)만 분석 영역으로 추출한다.

```cpp
cv::cvtColor(result, gray, cv::COLOR_BGR2GRAY);
cv::threshold(gray, binary, 120, 255, cv::THRESH_BINARY_INV);
return binary(cv::Rect(0, 0, 500, 250));
```

---

#### `find_target_line()` — 장애물 클러스터 탐색

```
cv::Mat roi → connectedComponentsWithStats → 좌/우 최단거리 장애물 인덱스 반환
```

화면 중심(250, 250) 기준으로 좌/우를 나누어 각 방향에서 가장 가까운 장애물 클러스터를 하나씩 선택한다.

```cpp
if (cx < 250) { // 좌측 장애물
    if (dist < min_dist_l) { l_idx = i; }
} else {        // 우측 장애물
    if (dist < min_dist_r) { r_idx = i; }
}
```

| 방향 | 미탐지 기본값 | 효과 |
|------|-------------|------|
| 좌측 | (50, 50) | 우측으로 회피 유도 |
| 우측 | (450, 50) | 좌측으로 회피 유도 |

---

#### `draw_result()` — 결과 시각화

| 색상 | 의미 |
|------|------|
| 초록 화살표 | 로봇 → 좌측 장애물 방향 |
| 빨간 화살표 | 로봇 → 우측 장애물 방향 |
| 파란 화살표 | 로봇 → 목표 방향 (좌·우 중간) |

---

#### 속도 명령 계산

```
error = 250 - (좌측 x + 우측 x) / 2
vel.x = 50 - k * error   (왼쪽 바퀴)
vel.y = -(50 + k * error) (오른쪽 바퀴)
```

| 상황 | error | 결과 |
|------|-------|------|
| 장애물이 좌측에 치우침 | 양수(+) | 왼쪽 감속, 오른쪽 가속 → 좌회전 |
| 장애물이 우측에 치우침 | 음수(-) | 왼쪽 가속, 오른쪽 감속 → 우회전 |
| 정중앙 | 0 | 직진 |

- 게인 `k = 1.5` / `s` 키: 주행 시작 / `q` 키: 정지



---

## 실습 과제 3 — 실제 장애물 회피 주행 (lidardrive)

> 실제 LIDAR 데이터를 실시간으로 처리하여 장애물 사이를 자율 주행

### Block Diagram

![lidardrive3](https://github.com/user-attachments/assets/90b54aba-5fdc-4c52-8e59-120f49063d75)




### 노드 구성

| 노드 | 패키지 | 실행 PC | 역할 |
|------|--------|---------|------|
| `sllidar_node` | sllidar_ros2 | RPi5 | LIDAR 스캔 데이터 발행 |
| `node_dxlsub` | dxl_nano | RPi5 | 속도 명령 수신 → 모터 구동 |
| `sllidar_client` (lidardrive) | lidardrive | WSL | 스캔 수신 → 장애물 탐지 → 속도 명령 발행 |

> lidarsim과 달리 타이머 없이 `/scan` 콜백에서 직접 처리 (LIDAR 주기 10Hz)

### 코드 설명

#### `mysub_callback()` — 스캔 수신 및 처리

```
/scan (LaserScan) → 스캔 이미지 생성 → 방향 보정 → 장애물 탐지 → 속도 명령 발행
```

LIDAR 데이터를 받아 500×500 흰색 이미지에 극좌표 → 직교좌표로 변환하여 점을 표시한다.

**① 스캔 이미지 생성**
```cpp
float scale = 8.0;
float x = 250 + scan->ranges[i] * (10.0*scale) * sin(angle);
float y = 250 - scan->ranges[i] * (10.0*scale) * cos(angle);
cv::circle(scan_video, cv::Point(x,y), 1, cv::Scalar(0,0,255), -1);
```

**② 방향 보정**

LIDAR 좌표계와 이미지 좌표계의 차이를 보정하여 이미지 상단이 로봇 전방이 되도록 한다.

```cpp
cv::Mat rotate = cv::getRotationMatrix2D(cv::Point(250,250), 180, 1);
cv::warpAffine(scan_video, result, rotate, scan_video.size());
cv::flip(result, result, 1);
```

---

#### `preprocess_image()` — 이진화 및 ROI 추출

```
cv::Mat result → 그레이스케일 → 이진화(반전) → 상단 절반 ROI 반환
```

임계값 100으로 이진화하여 장애물을 검출하고 전방 영역(상단 250px)만 분석한다.
(lidarsim의 임계값 120보다 낮아 더 민감하게 장애물 검출)

```cpp
cv::threshold(gray, binary, 100, 255, cv::THRESH_BINARY_INV);
return binary(cv::Rect(0, 0, 500, 250));
```

---

#### `find_target_line()` — 장애물 클러스터 탐색

```
cv::Mat roi → connectedComponentsWithStats → 좌/우 최단거리 장애물 인덱스 반환
```

cx < 250 기준으로 좌/우 장애물을 분류하고, 각 방향에서 로봇과 가장 가까운 클러스터를 선택한다.

```cpp
if (cx < 250) {
    if (dist < min_dist_l) { min_dist_l = dist; l_idx = i; }
} else {
    if (dist < min_dist_r) { min_dist_r = dist; r_idx = i; }
}
```

| 방향 | 미탐지 기본값 | 효과 |
|------|-------------|------|
| 좌측 | (0, 0) | 우측으로 회피 유도 |
| 우측 | (500, 0) | 좌측으로 회피 유도 |

---

#### `draw_result()` — 결과 시각화

탐지된 장애물에 바운딩 박스와 화살표를 표시하고 목표 방향을 시각화한다.

```cpp
void LineDetector::draw_result(cv::Mat& result, ..., int left_idx, int right_idx) {

    // 탐지된 장애물에 바운딩 박스와 중심점 표시
    cv::rectangle(result, cv::Rect(left, top, width, height), color, 2);
    cv::circle(result, cv::Point(x, y), 3, color, -1);

    // 좌측 장애물 → 초록 화살표 (로봇 → 바운딩 박스 우하단)
    cv::Point l_box_bottom_right(left + width, top + height);
    cv::arrowedLine(result, robot_pos, l_box_bottom_right, cv::Scalar(0,255,0), 1);

    // 우측 장애물 → 빨간 화살표 (로봇 → 바운딩 박스 좌하단)
    cv::Point r_box_bottom_left(left, top + height);
    cv::arrowedLine(result, robot_pos, r_box_bottom_left, cv::Scalar(0,0,255), 1);

    // 목표 방향 → 파란 화살표 (로봇 → 좌/우 무게중심 중간점)
    int target_x = (tmp_pt_l.x + tmp_pt_r.x) / 2;
    cv::arrowedLine(result, robot_pos, cv::Point(target_x, 100), cv::Scalar(255,0,0), 1);
}
```

| 색상 | 대상 | 화살표 목표 |
|------|------|------------|
| 초록 | 좌측 장애물 | 바운딩 박스 우하단 꼭짓점 |
| 빨간 | 우측 장애물 | 바운딩 박스 좌하단 꼭짓점 |
| 파란 | 목표 방향 | 좌·우 무게중심 중간점 |

---

#### 속도 명령 계산

```
error = 250 - (좌측 무게중심 x + 우측 무게중심 x) / 2
```

| 조건 | 동작 |
|------|------|
| `error == 0` 또는 `\|error\| > 60` | 직진 (vel.x=50, vel.y=-50) |
| `0 < \|error\| ≤ 60` | 비례 조향 (게인 k=0.45) |

```cpp
if (error == 0 || error < -60 || error > 60) {
    vel.x = 50;
    vel.y = -50;
} else {
    vel.x = 50 - k * error;
    vel.y = -(50 + k * error);
}
```

- `s` 키: 주행 시작 / `q` 키: 정지

### 실행 결과

[camera view] (https://youtu.be/JcjDFo5NSPc)

[driving view] {https://youtu.be/mO01pdLnhD8)

[human view] (https://github.com/user-attachments/assets/d6ca01c1-8030-4c27-9bcc-79c06c35d625)
