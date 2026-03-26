
# lanefollow 패키지 설명

## 전체 프로그램 동작 설명

`lanefollow.cpp`는 ROS2 기반 양쪽 차선 추적 및 제어 노드이다. linetracer 패키지(단일 라인 추적)의 확장 버전으로, 좌측과 우측 차선을 독립적으로 추적하여 두 차선의 중점을 기준으로 주행 오차를 계산한다.

`linetracer.cpp`가 하나의 추적 좌표(`last_line_x_`, `last_line_y_`)로 단일 라인을 따라가는 반면, `lanefollow.cpp`는 좌/우 각각의 추적 좌표 쌍(`last_left_x/y_`, `last_right_x/y_`)을 유지하며, ROI를 좌측/우측 영역으로 나누어 독립적으로 블롭을 탐색한다. 오차 계산 방식도 다르다:

처리 파이프라인은 동일한 4단계 구조를 따른다:

1. **전처리 (`setROI`)** — 하단 1/4 ROI 추출, 그레이스케일 변환, 밝기 보정, 이진화
2. **차선 탐색 (`findLanes`)** — CCL 후 좌/우 영역에서 각각 2단계 최근접 블롭 탐색
3. **시각화 (`drawResult`)** — 좌/우 검출 차선과 추적 좌표를 컬러로 표시
4. **제어 (`image_callback`)** — 차선 중점 기반 오차로 P 제어 속도 계산 및 발행

---

## 함수별 설명

### 1. `LineTrackerProcessor()` — 생성자

```cpp
LineTrackerProcessor() : Node("lane_tracker_node"), mode_(false), k_(0.14), base_vel_(100) {
    // ROS2 파라미터 선언: k(비례 게인), base_vel(기본 속도)
    this->declare_parameter("k", 0.14);
    this->declare_parameter("base_vel", 100);
    k_ = this->get_parameter("k").as_double();
    base_vel_ = this->get_parameter("base_vel").as_int();

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    raw_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        "Image_Topic", qos_profile,
        std::bind(&LineTrackerProcessor::image_callback, this, std::placeholders::_1));

    vel_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("vel_cmd_topic", qos_profile);

    // 좌/우 차선 초기 추적 좌표
    // 640px 폭 기준으로 좌측 150px, 우측 540px 부근에 차선이 있다고 가정
    last_left_x_ = 150.0;
    last_left_y_ = 45.0;
    last_right_x_ = 540.0;
    last_right_y_ = 45.0;

    RCLCPP_INFO(this->get_logger(), "차선 분석 및 제어 노드 시작. 's': 주행, 'q': 정지");
}
```

---

### 2. `setROI()` — 전처리 (ROI 설정 및 이진화)

```cpp
cv::Mat setROI(cv::Mat &frame) {
    // 프레임 하단 1/4 영역을 ROI로 설정
    cv::Rect roi_rect(0, frame.rows * 3 / 4, frame.cols, frame.rows / 4);
    cv::Mat roi = frame(roi_rect).clone();

    // 그레이스케일 변환 후 평균 밝기를 100으로 보정
    cv::cvtColor(roi, roi, cv::COLOR_BGR2GRAY);
    roi += cv::Scalar(100) - cv::mean(roi);

    // 임계값 150 기준 이진화 — 흰색 차선만 추출
    cv::threshold(roi, roi, 150, 255, cv::THRESH_BINARY);

    return roi;
}
```

---

### 3. `findLanes()` — 차선 탐색 (좌/우 독립 2단계 추적)

```cpp
void findLanes(cv::Mat &bin_roi, cv::Mat &stats, cv::Mat &centroids, int &l_idx, int &r_idx) {
    // Connected Component Labeling 수행
    int n_labels = cv::connectedComponentsWithStats(bin_roi, labels_, stats, centroids);

    // ROI 중앙선 — 이 기준으로 블롭을 좌측/우측 차선 후보로 분류
    const double mid_line = bin_roi.cols / 2.0;

    // ─── [1단계] 좌/우 영역에서 각각 가장 가까운 후보 탐색 ───
    int l_min_idx = -1, r_min_idx = -1;
    double l_min_dist = mid_line, r_min_dist = mid_line;

    for (int i = 1; i < n_labels; i++) {  // i=0은 배경 제외
        int area = stats.at<int>(i, cv::CC_STAT_AREA);
        // 면적 300 이상만 후보 — sub.cpp(100)보다 높은 임계값
        // 양쪽 차선 환경에서 노이즈가 더 많을 수 있으므로 필터 강화
        if (area < 300) continue;

        double cx = centroids.at<double>(i, 0);
        double cy = centroids.at<double>(i, 1);

        if (cx < mid_line) {
            // 좌측 영역: 이전 좌측 추적 좌표와의 거리 계산
            // 좌/우가 분리되어 있으므로 탐색 반경을 줄여도 충분
            double dist = cv::norm(cv::Point2d(cx, cy) - cv::Point2d(last_left_x_, last_left_y_));
            if (dist < l_min_dist && dist <= 80.0) { l_min_dist = dist; l_min_idx = i; }
        } else {
            // 우측 영역: 이전 우측 추적 좌표와의 거리 계산
            double dist = cv::norm(cv::Point2d(cx, cy) - cv::Point2d(last_right_x_, last_right_y_));
            if (dist < r_min_dist && dist <= 90.0) { r_min_dist = dist; r_min_idx = i; }
        }
    }

    // 1단계에서 후보를 찾은 쪽만 추적 좌표 갱신
    // 좌/우 독립적으로 갱신되므로 한쪽 차선이 사라져도 다른 쪽에 영향 없음
    if (l_min_idx != -1) {
        last_left_x_ = centroids.at<double>(l_min_idx, 0);
        last_left_y_ = centroids.at<double>(l_min_idx, 1);
    }
    if (r_min_idx != -1) {
        last_right_x_ = centroids.at<double>(r_min_idx, 0);
        last_right_y_ = centroids.at<double>(r_min_idx, 1);
    }

    // ─── [2단계] 갱신된 좌표에서 가장 가까운 블롭을 최종 확정 ───
    // 50px 이내에서만 유효 인덱스로 인정 
    l_idx = -1; r_idx = -1;
    double l_best = 50.0, r_best = 50.0;

    for (int i = 1; i < n_labels; i++) {
        double cx = centroids.at<double>(i, 0);
        double cy = centroids.at<double>(i, 1);

        // 모든 블롭에 대해 좌/우 추적 좌표와의 거리를 동시에 계산
        double d_l = cv::norm(cv::Point2d(cx, cy) - cv::Point2d(last_left_x_, last_left_y_));
        double d_r = cv::norm(cv::Point2d(cx, cy) - cv::Point2d(last_right_x_, last_right_y_));

        if (d_l < l_best) { l_best = d_l; l_idx = i; }
        if (d_r < r_best) { r_best = d_r; r_idx = i; }
    }
}
```

---

### 4. `drawResult()` — 시각화

```cpp
cv::Mat drawResult(cv::Mat &bin_roi, cv::Mat &stats, int l_idx, int r_idx) {
    cv::Mat display;
    cv::cvtColor(bin_roi, display, cv::COLOR_GRAY2BGR);

    for (int i = 1; i < stats.rows; i++) {
        int area = stats.at<int>(i, cv::CC_STAT_AREA);
        if (area < 300) continue;  

        int l = stats.at<int>(i, 0), t = stats.at<int>(i, 1);
        int w = stats.at<int>(i, 2), h = stats.at<int>(i, 3);

        // 좌측 또는 우측 확정 차선이면 빨간색, 나머지 후보는 파란색
        // linetracer.cpp에서는 단일 best_idx만 비교했으나 여기서는 l_idx, r_idx 두 개를 비교
        cv::Scalar color = (i == l_idx || i == r_idx) ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 0, 0);
        cv::rectangle(display, cv::Rect(l, t, w, h), color, 2);
    }

    // 좌/우 추적 좌표를 각각 빨간 점으로 표시
    cv::circle(display, cv::Point(static_cast<int>(last_left_x_), static_cast<int>(last_left_y_)),
               3, cv::Scalar(0, 0, 255), -1);
    cv::circle(display, cv::Point(static_cast<int>(last_right_x_), static_cast<int>(last_right_y_)),
               3, cv::Scalar(0, 0, 255), -1);

    return display;
}
```

---

### 5. `image_callback()` — 메인 콜백

```cpp
void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    auto startTime = std::chrono::steady_clock::now();

    // 매 콜백마다 파라미터 재로드 (런타임 동적 변경 지원)
    k_ = this->get_parameter("k").as_double();
    base_vel_ = this->get_parameter("base_vel").as_int();

    checkKeyboard();

    cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    if (frame.empty()) return;

    // ── 파이프라인 실행 ──
    cv::Mat bin_roi = setROI(frame);
    cv::Mat stats, centroids;
    int l_idx, r_idx;

    findLanes(bin_roi, stats, centroids, l_idx, r_idx);  // 좌/우 독립 탐색
    cv::Mat display = drawResult(bin_roi, stats, l_idx, r_idx);

    // error = 중심 - 차선중점 
    // 즉, 차체 위치 기준 오차 → P 제어 공식
    double lane_midpoint = (last_left_x_ + last_right_x_) / 2.0;
    double error = (bin_roi.cols / 2.0) - lane_midpoint;

    geometry_msgs::msg::Vector3 vel_msg;
    if (mode_) {
        // P 제어: 오차에 비례하여 좌/우 속도 차이 생성
        vel_msg.x = base_vel_ - error * k_;
        vel_msg.y = -(base_vel_ + error * k_);
    } else {
        vel_msg.x = 0; vel_msg.y = 0;
    }
    vel_pub_->publish(vel_msg);

    cv::imshow("1. Raw Video", frame);
    cv::imshow("2. Binary Debug", display);
    cv::waitKey(1);

    auto endTime = std::chrono::steady_clock::now();
    float totalTime = std::chrono::duration<float, std::milli>(endTime - startTime).count();
    RCLCPP_INFO(this->get_logger(), "err:%.2lf lvel:%.2f rvel:%.2f time:%.2f",
                error, vel_msg.x, vel_msg.y, totalTime);
}
```

---

### 6. `checkKeyboard()` — 키보드 입력 처리

```cpp
void checkKeyboard() {
    // kbhit()로 비차단 키 입력 확인 후 getch()로 문자 읽기
    if (kbhit()) {
        char ch = getch();
        if (ch == 'q') { mode_ = false; RCLCPP_WARN(this->get_logger(), "STOP"); }
        else if (ch == 's') { mode_ = true; RCLCPP_INFO(this->get_logger(), "START"); }
    }
}
```

---

### 7. `getch()` / `kbhit()` — 유틸리티 (터미널 비차단 입력)

```cpp
// getch(): ICANON/ECHO 해제하여 한 글자 즉시 읽기
int getch() {
    struct termios oldt, newt; int ch;
    tcgetattr(STDIN_FILENO, &oldt); newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

// kbhit(): O_NONBLOCK으로 비차단 키 확인, 있으면 ungetc로 되돌림
bool kbhit() {
    struct termios oldt, newt; int ch, oldf;
    tcgetattr(STDIN_FILENO, &oldt); newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
    if (ch != EOF) { ungetc(ch, stdin); return true; }
    return false;
}
```

---

### 8. `main()` — 진입점

```cpp
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LineTrackerProcessor>());
    rclcpp::shutdown();
    return 0;
}
```
