
# lidarsave 패키지 설명

## 전체 프로그램 동작 설명

`lidarsave.cpp`는 ROS2 기반 LIDAR 스캔 데이터 시각화 및 녹화 노드이다. SLLIDAR가 발행하는 `sensor_msgs::msg::LaserScan` 메시지를 구독하여, 각 스캔 포인트를 2D 평면도 위에 점으로 찍어 실시간 시각화하고, 해당 영상을 MP4 파일로 저장한다.

- `scale = 100.0` → 1m = 100px, 최대 2.5m 범위 표시

처리 흐름:

1. **스캔 수신** — `scan` 토픽에서 `LaserScan` 메시지 수신
2. **시각화** — `visualizeScan()` : 극좌표 → 직교좌표 변환 후 500×500 이미지 생성 및 화면 출력, `cv::Mat` 반환
3. **녹화** — `saveVideo()` : 전달받은 `cv::Mat`을 VideoWriter로 MP4에 기록

---

## 데이터 흐름

```
/scan (LaserScan)
  │  ranges[], angle_min, angle_increment, scan_time
  ▼
scanCb()
  │
  ├──► visualizeScan(scan, count)
  │       │  극좌표 → 직교좌표 변환 → 이미지 생성 → cv::imshow()
  │       │
  │       └──► cv::Mat image (500×500)
  │                │
  │                ▼
  └──────────► saveVideo(image)
                    │
                    ▼
              scan_video.mp4
```

---

## 함수별 설명

### 1. 전역 변수 및 매크로

```cpp
#define RAD2DEG(x) ((x)*180./M_PI)

static cv::VideoWriter video_writer;
static bool is_video_init = false;  // 첫 프레임에서만 VideoWriter 초기화하기 위한 플래그
```

---

### 2. `visualizeScan()` — 스캔 시각화

**입력**: `LaserScan` 데이터 (`ranges[]`, `angle_min`, `angle_increment`), 포인트 수 `count`

**출력**: `cv::Mat image` (500×500 스캔 영상)

```cpp
static cv::Mat visualizeScan(sensor_msgs::msg::LaserScan::SharedPtr scan, int count) {
    // 500×500 흰색 배경 이미지 생성
    cv::Mat image(500, 500, CV_8UC3, cv::Scalar(255, 255, 255));

    // 중심(250,250) = 로봇 위치, 십자 표시
    cv::line(image, cv::Point(250, 245), cv::Point(250, 255), cv::Scalar(0, 0, 0), 1);
    cv::line(image, cv::Point(245, 250), cv::Point(255, 250), cv::Scalar(0, 0, 0), 1);

    // 거리-픽셀 환산 비율: 1m = 100px → 최대 표시 거리 2.5m
    double scale = 100.0;

    for (int i = 0; i < count; i++) {
        float distance = scan->ranges[i];
        float angle_rad = scan->angle_min + scan->angle_increment * i;

        if (std::isfinite(distance) && distance > 0) {
            // 극좌표 → 직교좌표 변환
            int x = 250 + (int)(distance * scale * sin(angle_rad));
            int y = 250 + (int)(distance * scale * cos(angle_rad));

            if (x >= 0 && x < 500 && y >= 0 && y < 500) {
                cv::circle(image, cv::Point(x, y), 2, cv::Scalar(0, 0, 255), -1); // 빨간 점
            }
        }
    }

    cv::imshow("LIDAR Scan", image); // 화면 출력
    cv::waitKey(1);

    return image; // cv::Mat을 saveVideo()로 전달
}
```

---

### 3. `saveVideo()` — 동영상 저장

**입력**: `cv::Mat image` (visualizeScan()이 반환한 스캔 영상)

```cpp
static void saveVideo(const cv::Mat& image) {
    // 첫 프레임에서만 VideoWriter 초기화
    if (!is_video_init) {
        video_writer.open("/home/linux/ros2_ws/video/ros2_basic_30_test1.mp4",
                          cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                          10, cv::Size(500, 500), true);
        is_video_init = true;
        printf("[SLLIDAR INFO]: Video recording started.\n");
    }

    if (video_writer.isOpened()) {
        video_writer.write(image); // 프레임 기록
    }
}
```

---

### 4. `scanCb()` — 스캔 수신 콜백

**입력**: `/scan` 토픽의 `LaserScan` 메시지

```cpp
static void scanCb(sensor_msgs::msg::LaserScan::SharedPtr scan) {
    int count = scan->scan_time / scan->time_increment;

    // ranges 배열의 실제 크기로 count 재설정 (불일치 방지)
    if(scan->ranges.size() > 0) count = scan->ranges.size();

    cv::Mat image = visualizeScan(scan, count); // 시각화 → cv::Mat 반환
    saveVideo(image);                           // cv::Mat → MP4 저장
}
```

---

### 5. `main()` — 진입점

```cpp
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("sllidar_client");

    // SensorDataQoS: LIDAR 드라이버 노드와 동일한 QoS로 구독
    auto lidar_info_sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
                          "scan", rclcpp::SensorDataQoS(), scanCb);

    rclcpp::spin(node);
    rclcpp::shutdown();

    // 종료 시 VideoWriter를 닫아 MP4 파일 정상 마무리
    if (video_writer.isOpened()) {
        video_writer.release();
    }

    return 0;
}
```
