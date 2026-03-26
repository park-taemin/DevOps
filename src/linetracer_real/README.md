
# linetracer 패키지 설명

## 전체 프로그램 동작 설명

`linetracer.cpp`는 ROS2 기반 단일 라인 추적 및 모터 제어 노드이다. `pub.cpp`가 발행하는 `CompressedImage` 메시지를 구독하여, 영상 내 주행 라인을 검출·추적한 뒤 오차 기반 좌/우 바퀴 속도 명령(`geometry_msgs::msg::Vector3`)을 발행한다.

처리 파이프라인은 4단계로 구성된다:

1. **전처리 (`setROI`)** — 수신 프레임의 하단 1/4 ROI를 추출하고, 그레이스케일 변환 → 밝기 보정 → 이진화를 수행하여 라인 후보 영역을 분리한다.
2. **라인 탐색 (`findLine`)** — 이진 ROI에 Connected Component Labeling을 적용하여 블롭을 분류하고, 이전 추적 좌표 기준 2단계 최근접 탐색으로 라인 블롭을 확정한다.
3. **시각화 (`drawResult`)** — 검출된 라인과 후보 블롭을 색상별 바운딩 박스로 표시하고, 추적 좌표를 점으로 찍어 디버그 화면을 생성한다.
4. **제어 (`image_callback`)** — 추적 좌표와 ROI 중심 간 오차를 계산하여 P 제어 방식으로 좌/우 속도를 결정하고 발행한다. 키보드 입력('s'/'q')으로 주행/정지를 토글한다.

---

## 함수별 설명

### 1. `LineTrackerProcessor()` — 생성자

```cpp
LineTrackerProcessor() : Node("line_tracker_node"), mode_(false), k_(0.15), base_vel_(100) {
    // 비례 게인 k=0.15 — 저속 주행에 맞춘 부드러운 조향
    this->declare_parameter("k", 0.15);
    // 기본 속도 50 — 저속 정밀 주행용
    this->declare_parameter("base_vel", 50);

    k_ = this->get_parameter("k").as_double();
    base_vel_ = this->get_parameter("base_vel").as_int();

    // BestEffort QoS — pub.cpp의 발행 QoS와 일치시켜 통신 보장
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    // "Image_Topic" 구독, 콜백 등록
    raw_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        "Image_Topic", qos_profile,
        std::bind(&LineTrackerProcessor::image_callback, this, std::placeholders::_1));

    // 계산된 좌/우 속도를 발행할 퍼블리셔 생성
    // dxl_nano.cpp가 이 토픽을 구독하여 모터를 구동
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("vel_cmd_topic", qos_profile);

    // 첫 프레임에서 라인 탐색의 시작 기준점 역할
    last_line_x_ = 320.0;
    last_line_y_ = 45.0;

    RCLCPP_INFO(this->get_logger(), "분석 및 제어 노드 시작. 's': 주행, 'q': 정지");
}
```

---

### 2. `setROI()` — 전처리 (ROI 설정 및 이진화)

```cpp
cv::Mat setROI(cv::Mat &frame) {
    // 프레임 하단 1/4 영역만 ROI로 설정
    // 라인은 카메라 하단부에 위치하므로 상단 3/4는 불필요
    cv::Rect roi_rect(0, frame.rows * 3 / 4, frame.cols, frame.rows / 4);
    cv::Mat roi = frame(roi_rect).clone();  // clone으로 원본과 분리

    cv::cvtColor(roi, roi, cv::COLOR_BGR2GRAY);
    // 평균 밝기를 100으로 보정 — 조명 변화에 따른 이진화 품질 저하 방지
    roi += cv::Scalar(100) - cv::mean(roi);

    // 밝기 보정 후에도 남는 미세 노이즈를 추가로 제거하는 효과
    cv::threshold(roi, roi, 150, 255, cv::THRESH_BINARY);

    return roi;
}
```

---

### 3. `findLine()` — 라인 탐색 (2단계 최근접 블롭 확정)

```cpp
int findLine(cv::Mat &bin_roi, cv::Mat &stats, cv::Mat &centroids) {
    // Connected Component Labeling 수행
    // labels_: 픽셀별 블롭 번호, stats: 블롭별 위치/크기/면적, centroids: 블롭별 중심좌표
    int n_labels = cv::connectedComponentsWithStats(bin_roi, labels_, stats, centroids);

    // ─── [1단계] 이전 추적 좌표에서 150px 이내 최근접 블롭 탐색 ───
    int min_index = -1;
    double min_dist = static_cast<double>(bin_roi.cols);  // 최대 거리를 영상 너비로 초기화

    for (int i = 1; i < n_labels; i++) {  // i=0은 배경이므로 제외
        int area = stats.at<int>(i, cv::CC_STAT_AREA);
        if (area > 100) {  // 면적 100 이상인 블롭만 후보로 간주
            double cx = centroids.at<double>(i, 0);
            double cy = centroids.at<double>(i, 1);
            // 이전 추적 좌표와의 유클리드 거리 계산
            double dist = cv::norm(cv::Point2d(cx, cy) - cv::Point2d(last_line_x_, last_line_y_));

            // 150px 이내이면서 가장 가까운 후보 갱신
            if (dist < min_dist && dist <= 150.0) {
                min_dist = dist;
                min_index = i;
            }
        }
    }

    // 갱신 조건: 100px 이내일 때만 추적 좌표를 갱신
    // 탐색 범위(150px)보다 갱신 범위(100px)를 좁게 설정하여
    // 멀리 있는 블롭에 추적점이 급격히 이동하는 것을 방지
    if (min_index != -1 && min_dist <= 100.0) {
        last_line_x_ = centroids.at<double>(min_index, 0);
        last_line_y_ = centroids.at<double>(min_index, 1);
    }

    // ─── [2단계] 갱신된 추적 좌표에서 가장 가까운 블롭을 최종 확정 ───
    int idx = -1;
    double best = static_cast<double>(bin_roi.cols);

    for (int i = 1; i < stats.rows; i++) {
        double cx = centroids.at<double>(i, 0);
        double cy = centroids.at<double>(i, 1);
        double d = cv::norm(cv::Point2d(cx, cy) - cv::Point2d(last_line_x_, last_line_y_));

        if (d < best) {
            best = d;
            idx = i;
        }
    }

    // 확정 거리 10px — 추적 좌표와 거의 일치하는 블롭만 유효로 인정
    if (best > 10.0) {
        idx = -1;
    }
    return idx;
}
```

---

### 4. `drawResult()` — 시각화

```cpp
cv::Mat drawResult(cv::Mat &bin_roi, cv::Mat &stats, int best_idx) {
    cv::Mat display;
    // 이진 영상을 3채널 컬러로 변환하여 색상 표시가 가능하도록 함
    cv::cvtColor(bin_roi, display, cv::COLOR_GRAY2BGR);

    for (int i = 1; i < stats.rows; i++) {
        int area = stats.at<int>(i, cv::CC_STAT_AREA);
        if (area < 100) continue;  // 면적 100 미만 블롭은 시각화에서도 제외

        // 블롭의 바운딩 박스 좌표 추출
        int l = stats.at<int>(i, 0), t = stats.at<int>(i, 1);
        int w = stats.at<int>(i, 2), h = stats.at<int>(i, 3);

        if (i == best_idx) {
            // 최종 검출된 라인: 빨간색 두꺼운 박스
            cv::rectangle(display, cv::Rect(l, t, w, h), cv::Scalar(0, 0, 255), 2);
        } else {
            // 기타 후보 블롭: 파란색 얇은 박스 (노이즈 확인용)
            cv::rectangle(display, cv::Rect(l, t, w, h), cv::Scalar(255, 0, 0), 1);
        }
    }
    // 현재 추적 좌표를 빨간 점으로 표시
    // 라인 검출 실패 시에도 마지막 유효 위치를 보여줌
    cv::circle(display, cv::Point(static_cast<int>(last_line_x_), static_cast<int>(last_line_y_)),
               3, cv::Scalar(0, 0, 255), -1);

    return display;
}
```

---

### 5. `image_callback()` — 메인 콜백

```cpp
void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    auto startTime = std::chrono::steady_clock::now();  // 처리 시간 측정 시작

    // 매 콜백마다 파라미터를 다시 읽어옴
    // ros2 param set으로 런타임 중 k, base_vel 값을 변경할 수 있음
    k_ = this->get_parameter("k").as_double();
    base_vel_ = this->get_parameter("base_vel").as_int();

    checkKeyboard();  // 's'(시작) / 'q'(정지) 키 입력 확인

    // CompressedImage 메시지의 JPEG 바이트를 OpenCV Mat으로 디코딩
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    if (frame.empty()) return;  // 디코딩 실패 시 콜백 종료

    // ── 파이프라인 실행 ──
    cv::Mat bin_roi = setROI(frame);          // 1. 전처리
    cv::Mat stats, centroids;
    int best_idx = findLine(bin_roi, stats, centroids);  // 2. 라인 탐색
    cv::Mat display = drawResult(bin_roi, stats, best_idx);  // 3. 시각화

    // ── 오차 계산 및 속도 명령 발행 ──
    // ROI 중심에서 추적 좌표를 빼는 방식 (차체 기준 보정)
    // 양수: 차체가 라인 오른쪽으로 치우침 → 좌회전 보정 필요
    // 음수: 차체가 라인 왼쪽으로 치우침 → 우회전 보정 필요
    double error = (bin_roi.cols / 2.0) - last_line_x_;

    geometry_msgs::msg::Vector3 vel_msg;
    if (mode_) {
        // P 제어: 오차에 비례하여 좌/우 바퀴 속도 차이를 생성
        // error > 0 → 왼쪽 속도 감소, 오른쪽 속도 증가(절대값) → 좌회전
        vel_msg.x = base_vel_ - error * k_;   // 왼쪽 바퀴 속도
        vel_msg.y = -(base_vel_ + error * k_);  // 오른쪽 바퀴 속도 (부호 반전)
    } else {
        // mode_ == false: 정지 상태
        vel_msg.x = 0; vel_msg.y = 0;
    }
    vel_pub_->publish(vel_msg);

    // 디버그용 윈도우 출력
    cv::imshow("1. Raw Video", frame);
    cv::imshow("2. Binary Debug", display);
    cv::waitKey(1);

    // 처리 시간 측정 및 로깅
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
        if (ch == 'q') {
            mode_ = false;  // 정지 모드 전환
            RCLCPP_WARN(this->get_logger(), "STOP");
        }
        else if (ch == 's') {
            mode_ = true;   // 주행 모드 전환
            RCLCPP_INFO(this->get_logger(), "START");
        }
    }
}
```

---

### 7. `getch()` / `kbhit()` — 유틸리티 (터미널 비차단 입력)

```cpp
// getch(): 터미널 에코 없이 한 글자 즉시 읽기
// ICANON(줄 단위 입력), ECHO(입력 표시) 플래그를 일시 해제하여 구현
int getch() {
    struct termios oldt, newt; int ch;
    tcgetattr(STDIN_FILENO, &oldt); newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

// kbhit(): 키 입력 대기 없이 버퍼에 키가 있는지 확인
// O_NONBLOCK 플래그로 getchar()를 비차단 호출하고,
// 읽은 문자가 있으면 ungetc()로 버퍼에 되돌려 놓음
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
    rclcpp::init(argc, argv);  // ROS2 초기화
    // LineTrackerProcessor 노드를 생성하고, 콜백이 반복 호출되도록 spin
    rclcpp::spin(std::make_shared<LineTrackerProcessor>());
    rclcpp::shutdown();  // 종료 시 ROS2 정리
    return 0;
}
```
