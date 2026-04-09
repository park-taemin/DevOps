# lidardrive 패키지 설명

## 전체 프로그램 동작 설명

`lidardrive.cpp`는 실제 LIDAR 스캔 데이터를 실시간으로 구독하여 장애물을 검출하고, 장애물 사이를 통과하는 속도 명령을 계산해 발행하는 자율주행 노드이다.

lidarsim이 저장된 동영상으로 시뮬레이션하는 것과 달리, lidardrive는 `/scan` 토픽을 직접 구독하여 실제 로봇을 제어한다.

처리 흐름:

1. **스캔 수신** — `/scan` 토픽에서 `LaserScan` 메시지를 수신
2. **스캔 → 이미지 변환** — 극좌표를 직교좌표로 변환하여 500×500 스캔 영상 생성 후 180° 회전 및 좌우 반전으로 방향 보정
3. **전처리** — 이진화 후 상단 절반(ROI) 추출
4. **장애물 탐지** — Connected Components로 좌·우 가장 가까운 장애물 클러스터 탐색
5. **시각화** — 좌측(초록), 우측(빨간), 목표 방향(파란) 화살표 표시
6. **속도 명령 계산** — 좌·우 장애물 중앙과 화면 중심의 오차로 속도 계산, 오차가 없거나 너무 크면 직진
7. **발행 및 저장** — `vel_cmd_topic`으로 속도 명령 발행, 결과 영상 저장

---

## 파일 구조

| 파일 | 역할 |
|------|------|
| `lidardrive.hpp` | `LineDetector` 클래스 선언 |
| `lidardrive.cpp` | 클래스 구현 (콜백, 전처리, 탐지, 시각화) |
| `main.cpp` | 노드 진입점 |

---

## 클래스 멤버 설명 (`LineDetector`)

```cpp
class LineDetector : public rclcpp::Node {
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;  // /scan 구독
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_;     // 속도 명령 발행
    cv::Point tmp_pt_l, tmp_pt_r;   // 좌·우 장애물 위치 (이전 프레임 유지)
    geometry_msgs::msg::Vector3 vel; // 발행할 속도 명령
    bool mode;                       // true: 주행, false: 정지
    double k = 0.45;                 // 제어 게인
};
```

---

## 함수별 설명

### 1. 생성자 `LineDetector()`

```cpp
LineDetector::LineDetector()
    : Node("sllidar_client"), tmp_pt_l(125, 125), tmp_pt_r(375, 125), mode(false) {

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    // /scan 토픽 구독 (실제 LIDAR)
    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", qos_profile,
        bind(&LineDetector::mysub_callback, this, placeholders::_1));

    pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("vel_cmd_topic", qos_profile);
}
```

- lidarsim과 달리 타이머 대신 스캔 콜백 방식 사용
- 좌·우 초기 위치를 화면 안쪽(125, 125), (375, 125)으로 설정

---

### 2. `mysub_callback()` — 스캔 수신 콜백

```cpp
void LineDetector::mysub_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    // 1. 스캔 → 500×500 이미지 생성
    cv::Mat scan_video(500, 500, CV_8UC3, cv::Scalar(255, 255, 255));
    float scale = 8.0;  // 표시 범위 조정

    for (int i = 0; i < count; i++) {
        float x = 250 + scan->ranges[i] * (10.0 * scale) * sin(angle);
        float y = 250 - scan->ranges[i] * (10.0 * scale) * cos(angle);
        cv::circle(scan_video, cv::Point(x, y), 1, cv::Scalar(0, 0, 255), -1);
    }

    // 2. 방향 보정: 180° 회전 후 좌우 반전
    // LIDAR 좌표계와 이미지 좌표계의 방향 차이를 보정
    cv::Mat rotate = cv::getRotationMatrix2D(cv::Point(250, 250), 180, 1);
    cv::Mat result;
    cv::warpAffine(scan_video, result, rotate, scan_video.size());
    cv::flip(result, result, 1);

    // 3. 전처리 → 장애물 탐지 → 시각화
    cv::Mat roi = preprocess_image(result);
    cv::connectedComponentsWithStats(roi, labels, stats, centroids);
    auto targets = find_target_line(roi, stats, centroids);
    draw_result(result, stats, centroids, targets.first, targets.second);

    // 4. 오차 계산 및 속도 명령
    int error = 250 - ((tmp_pt_l.x + tmp_pt_r.x) / 2);

    if (mode) {
        // 오차가 0이거나 너무 크면(-60~60 범위 이탈) 직진
        if (error == 0 || error < -60 || error > 60) {
            vel.x = 50;
            vel.y = -50;
        } else {
            vel.x = 50 - k * error;
            vel.y = -(50 + k * error);
        }
    } else { vel.x = 0; vel.y = 0; }

    pub_->publish(vel);
}
```

- `scale = 8.0`: `10.0 * scale = 80px/m`, 반경 250px ÷ 80 ≈ 3.1m 범위 표시
- 직진 조건(`error == 0 || |error| > 60`): 장애물이 정중앙이거나 한쪽으로 너무 치우치면 직진으로 통과 시도

---

### 3. `preprocess_image()` — 이진화 및 ROI 추출

```cpp
cv::Mat LineDetector::preprocess_image(const cv::Mat& result) {
    cv::Mat gray, binary;
    cv::cvtColor(result, gray, cv::COLOR_BGR2GRAY);

    // 밝기 100 이하 → 255 (장애물), 이상 → 0 (빈 공간)
    cv::threshold(gray, binary, 100, 255, cv::THRESH_BINARY_INV);

    // 상단 절반(y: 0~250)만 ROI — 로봇 전방 영역만 탐색
    return binary(cv::Rect(0, 0, 500, 250));
}
```

---

### 4. `find_target_line()` — 좌·우 장애물 탐색

```cpp
std::pair<int, int> LineDetector::find_target_line(...) {
    cv::Point robot_pos(250, 250);

    for (int i = 1; i < cnt; i++) {
        // x < 250이면 좌측, 이상이면 우측으로 분류
        // 각각 로봇과 가장 가까운 클러스터 선택
    }

    // 장애물 미탐지 시 기본값
    if (l_idx == -1) tmp_pt_l = cv::Point(0, 0);     // 왼쪽 구석
    if (r_idx == -1) tmp_pt_r = cv::Point(500, 0);   // 오른쪽 구석
}
```

- lidarsim과 달리 최소 면적 필터 없음 (모든 클러스터 대상)
- 장애물 미탐지 시 화면 가장자리를 기본값으로 사용 → 목표 방향이 자동으로 정중앙(250)을 향함

---

### 5. `draw_result()` — 시각화

```cpp
void LineDetector::draw_result(...) {
    cv::Point robot_pos(250, 250);

    // 각 장애물의 바운딩 박스 표시
    cv::rectangle(result, cv::Rect(left, top, width, height), color, 2);
    cv::circle(result, cv::Point(cx, cy), 3, color, -1);

    // 초록 화살표: 로봇 → 좌측 장애물 바운딩 박스 우하단
    cv::arrowedLine(result, robot_pos, l_box_bottom_right, cv::Scalar(0, 255, 0), 1);

    // 빨간 화살표: 로봇 → 우측 장애물 바운딩 박스 좌하단
    cv::arrowedLine(result, robot_pos, r_box_bottom_left, cv::Scalar(0, 0, 255), 1);

    // 파란 화살표: 로봇 → 좌·우 중간 목표 방향
    int target_x = (tmp_pt_l.x + tmp_pt_r.x) / 2;
    cv::arrowedLine(result, robot_pos, cv::Point(target_x, 100), cv::Scalar(255, 0, 0), 1);
}
```

- lidarsim과 달리 클러스터의 **무게중심**이 아닌 **바운딩 박스 꼭짓점**을 화살표 목표로 사용

---

### 6. `main()` — 진입점

```cpp
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = make_shared<LineDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```
