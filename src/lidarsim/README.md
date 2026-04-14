# lidarsim 패키지 설명

## 전체 프로그램 동작 설명

`sub.cpp`는 lidarsave 패키지로 저장한 LIDAR 스캔 동영상을 재생하면서, 프레임마다 장애물을 검출하고 회피 속도 명령을 계산하여 발행하는 시뮬레이션 노드이다.

실제 LIDAR 없이도 저장된 동영상 파일로 장애물 회피 알고리즘을 검증할 수 있다.

처리 흐름:

1. **프레임 읽기** — 저장된 MP4 동영상을 100ms 타이머로 한 프레임씩 읽음 (10fps)
2. **전처리** — 프레임을 이진화하고 상단 절반(ROI)만 잘라 장애물 영역 추출
3. **장애물 탐지** — Connected Components로 레이블링 후 로봇 중심(250, 250)에서 좌·우 가장 가까운 클러스터 탐색
4. **시각화** — 좌측(초록), 우측(빨간), 목표 방향(파란) 화살표를 프레임 위에 표시
5. **속도 명령 계산** — 좌·우 장애물 중앙과 화면 중심의 오차로 좌·우 바퀴 속도 계산
6. **발행 및 저장** — `vel_cmd_topic`으로 속도 명령 발행, 결과 영상을 MP4로 저장

---

## 파일 구조

| 파일 | 역할 |
|------|------|
| `sub.hpp` | `LineDetector` 클래스 선언 |
| `sub.cpp` | 클래스 구현 (콜백, 전처리, 탐지, 시각화) |
| `main.cpp` | 노드 진입점 |

---

## 클래스 멤버 설명 (`LineDetector`)

```cpp
class LineDetector : public rclcpp::Node {
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_;  // 속도 명령 퍼블리셔
    rclcpp::TimerBase::SharedPtr timer_;   // 100ms 주기 타이머
    cv::VideoCapture cap_;                 // 동영상 파일 읽기
    cv::VideoWriter video_writer;          // 결과 영상 저장
    cv::Point tmp_pt_l, tmp_pt_r;         // 좌·우 장애물 위치 (이전 프레임 유지)
    geometry_msgs::msg::Vector3 vel;       // 발행할 속도 명령
    bool mode;                             // true: 주행, false: 정지
    double k = 1.5;                        // 제어 게인
    cv::Mat labels;                        // connectedComponents 레이블 맵
};
```

---

## 함수별 설명

### 1. 생성자 `LineDetector()`

```cpp
LineDetector::LineDetector()
    : Node("sllidar_client"), tmp_pt_l(50, 50), tmp_pt_r(450, 50), mode(false) {

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("vel_cmd_topic", qos);

    // lidarsave가 저장한 동영상 파일 열기
    cap_.open("/home/linux/ros2_ws/video/ros2_basic_30_test1.mp4");

    // 10fps에 맞춰 100ms 타이머 설정
    timer_ = this->create_wall_timer(100ms, std::bind(&LineDetector::timer_callback, this));
}
```

- 좌·우 장애물 초기 위치를 화면 좌측 상단(50, 50), 우측 상단(450, 50)으로 설정
- 동영상 파일을 미리 열어 타이머 콜백에서 순서대로 읽을 수 있도록 준비

---

### 2. `timer_callback()` — 메인 처리 루프

```cpp
void LineDetector::timer_callback() {
    cv::Mat frame;
    cap_ >> frame;  // 동영상에서 한 프레임 읽기

    if (frame.empty()) {
        cap_.set(cv::CAP_PROP_POS_FRAMES, 0);  // 영상 끝나면 처음으로 되돌림
        return;
    }

    // 1. 전처리 및 레이블링
    cv::Mat roi = preprocess_image(frame);
    cv::Mat stats, centroids;
    cv::connectedComponentsWithStats(roi, labels, stats, centroids);

    // 2. 장애물 탐지 및 시각화
    auto targets = find_target_line(roi, stats, centroids);
    draw_result(frame, stats, centroids, targets.first, targets.second);

    // 3. 오차 계산 및 키보드 입력 처리
    int error = 250 - ((tmp_pt_l.x + tmp_pt_r.x) / 2);
    if (kbhit()) { int ch = getch(); if (ch == 'q') mode = false; else if (ch == 's') mode = true; }

    // 4. 속도 명령 계산
    // 직진속도 50rpm 기준, 오차에 게인을 곱해 좌·우 속도 차이 생성
    if (mode) {
        vel.x = 50.0 - k * error;   // 왼쪽 바퀴
        vel.y = -(50.0 + k * error); // 오른쪽 바퀴
    } else { vel.x = 0; vel.y = 0; }

    pub_->publish(vel);

    // 5. 결과 영상 저장 (mp4v 코덱)
    if (!video_writer.isOpened()) {
        video_writer.open("/home/linux/ros2_ws/video/ros2_basic_30_test2.mp4",
                          cv::VideoWriter::fourcc('m','p','4','v'), 10.0, frame.size(), true);
    }
    video_writer.write(frame);
    cv::imshow("Lidar Sim", frame);
    cv::waitKey(1);
}
```

---

### 3. `preprocess_image()` — 이진화 및 ROI 추출

```cpp
cv::Mat LineDetector::preprocess_image(const cv::Mat& result) {
    cv::Mat gray, binary;
    cv::cvtColor(result, gray, cv::COLOR_BGR2GRAY);

    // 밝기 120 이하 → 255 (장애물), 이상 → 0 (빈 공간)
    cv::threshold(gray, binary, 120, 255, cv::THRESH_BINARY_INV);

    // 상단 절반(y: 0~250)만 ROI로 사용 — 로봇 전방 영역만 탐색
    return binary(cv::Rect(0, 0, 500, 250));
}
```

---

### 4. `find_target_line()` — 좌·우 장애물 탐색

```cpp
std::pair<int, int> LineDetector::find_target_line(...) {
    cv::Point robot_pos(250, 250);

    for (int i = 1; i < cnt; i++) {
        if (stats.at<int>(i, 4) < 5) continue;  // 너무 작은 클러스터 무시
        // 로봇과 너무 가까운 점도 무시 (화살표 그리기 오류 방지)
        if (dist < 10.0) continue;

        // x < 250이면 좌측, 이상이면 우측 장애물로 분류
        // 각각 로봇과 가장 가까운 클러스터를 선택
    }
}
```

- 반환값: `(left_idx, right_idx)` — 좌·우 장애물 클러스터의 인덱스

---

### 5. `draw_result()` — 시각화

```cpp
void LineDetector::draw_result(...) {
    cv::Point robot_pos(250, 250);

    // 초록 화살표: 로봇 → 좌측 장애물
    cv::arrowedLine(result, robot_pos, tmp_pt_l, cv::Scalar(0, 255, 0), 2);

    // 빨간 화살표: 로봇 → 우측 장애물
    cv::arrowedLine(result, robot_pos, tmp_pt_r, cv::Scalar(0, 0, 255), 2);

    // 파란 화살표: 로봇 → 좌·우 중간 목표 방향
    int target_x = (tmp_pt_l.x + tmp_pt_r.x) / 2;
    cv::arrowedLine(result, robot_pos, cv::Point(target_x, 100), cv::Scalar(255, 0, 0), 3);
}
```

---

### 6. `main()` — 진입점

```cpp
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LineDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```
