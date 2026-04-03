# lidarplot 패키지 설명

## 전체 프로그램 동작 설명

`sllidar_client.cpp`는 ROS2 기반 LIDAR 스캔 데이터 시각화 및 녹화 노드이다. SLLIDAR(구 RPLIDAR)가 발행하는 `sensor_msgs::msg::LaserScan` 메시지를 구독하여, 각 스캔 포인트를 2D 평면도 위에 점으로 찍어 실시간 시각화하고, 해당 영상을 MP4 파일로 저장한다.

이 노드는 라인 트래킹 파이프라인(`pub → sub → dxl_sub`)과는 독립적으로 동작하며, 로봇 주변 장애물 분포를 실시간으로 확인하는 디버깅·모니터링 용도이다.

처리 흐름:

1. **스캔 수신** — `scan` 토픽에서 `LaserScan` 메시지를 수신하여 거리·각도 데이터 배열을 얻음
2. **좌표 변환** — 극좌표(거리, 각도)를 직교좌표(x, y 픽셀)로 변환하여 500×500 이미지 위에 매핑
3. **시각화** — 유효한 스캔 포인트를 빨간 점으로 표시하고, 중심(로봇 위치)을 십자로 표시
4. **녹화** — 첫 프레임 수신 시 VideoWriter를 초기화하고, 이후 매 프레임을 MP4 파일에 기록

---

## 함수별 설명

### 1. 전역 변수 및 매크로

```cpp
// 라디안 → 도 변환 매크로
#define RAD2DEG(x) ((x)*180./M_PI)

// 동영상 저장용 전역 객체
// 콜백 함수가 자유 함수이므로 클래스 멤버 대신 static 전역으로 관리
static cv::VideoWriter video_writer;
static bool is_video_init = false;  // 첫 프레임에서만 VideoWriter 초기화하기 위한 플래그
```

---

### 2. `scanCb()` — 스캔 데이터 수신 콜백

```cpp
static void scanCb(sensor_msgs::msg::LaserScan::SharedPtr scan) {
    // 스캔 시간과 포인트 간 시간 간격으로 데이터 개수 계산
    int count = scan->scan_time / scan->time_increment;
    
    // ranges 배열의 실제 크기로 count를 재설정
    // 계산값과 실제 데이터 개수가 불일치할 수 있으므로 안전 조치
    if(scan->ranges.size() > 0) count = scan->ranges.size();

    printf("[SLLIDAR INFO]: I heard a laser scan %s[%d]:\n",
           scan->header.frame_id.c_str(), count);
    printf("[SLLIDAR INFO]: angle_range : [%f, %f]\n",
           RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));

    // ── 시각화 영상 준비 ──
    // 500×500 흰색 배경 이미지 생성
    // 중심(250, 250)이 로봇 위치, 반경 250px = 5m
    cv::Mat image(500, 500, CV_8UC3, cv::Scalar(255, 255, 255));
    
    cv::Point center(250, 250);

    // 중심에 검은색 십자 표시 — 로봇 위치 기준점
    cv::line(image, cv::Point(250, 245), cv::Point(250, 255), cv::Scalar(0, 0, 0), 1);
    cv::line(image, cv::Point(245, 250), cv::Point(255, 250), cv::Scalar(0, 0, 0), 1);

    // 거리-픽셀 환산 비율: 1m = 50px
    // 이미지 반경 250px ÷ 50px/m = 최대 표시 거리 5m
    double scale = 50.0; 

    for (int i = 0; i < count; i++) {
        float distance = scan->ranges[i];         // 거리 (m)
        float angle_rad = scan->angle_min + scan->angle_increment * i;  // 각도 (rad)
        
        // 유효한 거리 데이터만 처리 (inf, nan, 0 제외)
        if (std::isfinite(distance) && distance > 0) {
            // 극좌표 → 직교좌표 변환 (이미지 좌표계 기준)
            // x축: sin(angle) — 양의 각도가 오른쪽
            // y축: cos(angle) — 0도가 아래쪽(이미지 y 증가 방향)
            // 주의: 일반적 로봇 좌표계(0도=전방, 반시계 양수)와
            //       이미지 좌표계(y 아래 증가)의 차이를 반영
            int x = 250 + (int)(distance * scale * sin(angle_rad));
            int y = 250 + (int)(distance * scale * cos(angle_rad));

            // 이미지 범위(0~499) 내에 있는 포인트만 표시
            if (x >= 0 && x < 500 && y >= 0 && y < 500) {
                // 빨간 점(반지름 2px, 채워진 원)으로 장애물 위치 표시
                cv::circle(image, cv::Point(x, y), 2, cv::Scalar(0, 0, 255), -1);
            }
        }
    }

    // 화면에 스캔 영상 출력
    cv::imshow("LIDAR Scan", image);
    cv::waitKey(1);  // 1ms 대기 — OpenCV GUI 이벤트 처리에 필수

    // ── 동영상 녹화 ──
    // 첫 프레임 수신 시 VideoWriter 초기화 (한 번만 실행)
    if (!is_video_init) {
        // MJPG 코덱, 10fps, 500×500 해상도로 MP4 저장
        // 저장 경로는 하드코딩 — 실행 환경에 따라 수정 필요
        video_writer.open("/home/linux/ros2_ws/video/scan_video.mp4",
                          cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                          10, cv::Size(500, 500), true);
        is_video_init = true;
        printf("[SLLIDAR INFO]: Video recording started.\n");
    }

    // VideoWriter가 정상 열린 경우 현재 프레임 기록
    if (video_writer.isOpened()) {
        video_writer.write(image);
    }
}
```

---

### 3. `main()` — 진입점

```cpp
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("sllidar_client");

    // "scan" 토픽 구독
    // SensorDataQoS: 센서 데이터에 최적화된 QoS 프리셋
    // BestEffort + KeepLast(5) + Volatile이 기본 설정
    // LIDAR 드라이버 노드(sllidar_ros2)가 동일 QoS로 발행하므로 호환됨
    auto lidar_info_sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
                          "scan", rclcpp::SensorDataQoS(), scanCb);

    // 콜백 반복 호출 (Ctrl+C까지 블로킹)
    rclcpp::spin(node);

    rclcpp::shutdown();

    // 종료 시 VideoWriter를 닫아 파일을 정상적으로 마무리
    // 닫지 않으면 MP4 헤더가 기록되지 않아 파일이 재생 불가할 수 있음
    if (video_writer.isOpened()) {
        video_writer.release();
    }

    return 0;
}
```
