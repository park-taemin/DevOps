# linedetect_wsl 패키지 설명

## 1. 프로그램 동작 설명

ROS2 노드로서  `Image_Topic`에서 수신한 압축 영상 프레임을 복원한 뒤 ROI 영역에서 이진화 및 레이블링을 수행하여 추종 대상 라인을 검출한다. 이전 프레임의 라인 위치와 가장 가까운 객체를 선택하여 라인 위치를 갱신하고, 화면 중앙과의 오차(error)를 계산하여 로그로 출력한다. 처리 결과는 OpenCV 윈도우를 통해 확인

## 2. 함수별 설명

### LineTrackerProcessor() : 노드를 초기화하고, 영상 구독자 생성 및 초기 라인 추적 위치를 설정한다.

```cpp
LineTrackerProcessor() : Node("line_tracker_processor_node") {
    // QoS 프로파일을 best_effort 방식으로 설정
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    // "Image_Topic"을 구독하여 영상 수신 시 image_callback 호출
    raw_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        "Image_Topic", qos_profile,
        std::bind(&LineTrackerProcessor::image_callback, this, std::placeholders::_1));

    // 초기 라인 추적 위치를 영상 중앙 부근으로 설정
    last_line_x_ = 320.0;
    last_line_y_ = 45.0;

    RCLCPP_INFO(this->get_logger(), "WSL 기반 분석 노드가 시작되었습니다.");
}
```

### ~LineTrackerProcessor() : 노드 소멸 시 모든 OpenCV 윈도우를 닫는다.

```cpp
~LineTrackerProcessor() { cv::destroyAllWindows(); }
```

### image_callback() : 수신된 영상을 복원하고, ROI 이진화 → 레이블링 → 라인 검출 → 오차 계산 → 결과 시각화를 수행한다.

```cpp
void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    // 처리 시간 측정 시작
    auto startTime = std::chrono::steady_clock::now();

    // 1. 압축 영상 데이터를 OpenCV Mat으로 복원
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    if (frame.empty()) return;

    // 2. ROI 설정(영상 하단 270~360 영역) 및 전처리
    cv::Rect roi_rect(0, 270, 640, 90);
    cv::Mat roi = frame(roi_rect);
    cv::Mat gray, bin, display;

    // 그레이스케일 변환
    cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
    // 평균 밝기를 기준으로 밝기 보정 (밝기를 100 근처로 맞춤)
    double avg = cv::mean(gray)[0];
    gray.convertTo(gray, -1, 1, 100 - static_cast<int>(avg));
    // 임계값 128로 이진화
    cv::threshold(gray, bin, 128, 255, cv::THRESH_BINARY);

    // 이진 이미지를 컬러로 변환 (시각화용)
    cv::cvtColor(bin, display, cv::COLOR_GRAY2BGR);

    // 3. 연결 요소 레이블링으로 객체 검출
    cv::Mat labels, stats, centroids;
    int n_labels = cv::connectedComponentsWithStats(bin, labels, stats, centroids);

    double min_dist = 1e9;
    int best_idx = -1;

    for (int i = 1; i < n_labels; i++) {
        // 면적이 100~15000 범위 밖이면 노이즈로 판단하여 무시
        int area = stats.at<int>(i, cv::CC_STAT_AREA);
        if (area < 100 || area > 15000) continue;

        // 각 객체의 바운딩 박스를 파란색으로 표시
        int left = stats.at<int>(i, cv::CC_STAT_LEFT);
        int top = stats.at<int>(i, cv::CC_STAT_TOP);
        int width = stats.at<int>(i, cv::CC_STAT_WIDTH);
        int height = stats.at<int>(i, cv::CC_STAT_HEIGHT);
        cv::rectangle(display, cv::Rect(left, top, width, height), cv::Scalar(255, 0, 0), 2);

        // 이전 프레임의 라인 위치와의 거리를 계산하여 가장 가까운 객체 선택
        double cx = centroids.at<double>(i, 0);
        double cy = centroids.at<double>(i, 1);
        double dist = std::sqrt(std::pow(cx - last_line_x_, 2) + std::pow(cy - last_line_y_, 2));

        // 거리가 150 이내이고 가장 가까운 객체를 추종 대상으로 선택
        if (dist < min_dist && dist < 150.0) {
            min_dist = dist;
            best_idx = i;
        }
    }

    // 4. 선택된 라인의 위치를 업데이트하고 빨간색 박스 및 중심점으로 시각화
    if (best_idx != -1) {
        last_line_x_ = centroids.at<double>(best_idx, 0);
        last_line_y_ = centroids.at<double>(best_idx, 1);

        cv::rectangle(display, cv::Rect(stats.at<int>(best_idx, 0), stats.at<int>(best_idx, 1),
                      stats.at<int>(best_idx, 2), stats.at<int>(best_idx, 3)), cv::Scalar(0, 0, 255), 3);
        cv::circle(display, cv::Point(static_cast<int>(last_line_x_), static_cast<int>(last_line_y_)), 5, cv::Scalar(0, 0, 255), -1);
    }

    // 5. 화면 중앙과 라인 위치의 오차 계산 및 처리 시간 로그 출력
    double error = (bin.cols / 2.0) - last_line_x_;
    auto endTime = std::chrono::steady_clock::now();
    float totalTime = std::chrono::duration<float, std::milli>(endTime - startTime).count();

    RCLCPP_INFO(this->get_logger(), "Error: %.2f | Processing Time: %.2f ms", error, totalTime);

    // 6. 원본 ROI와 레이블링 결과를 OpenCV 윈도우로 표시
    cv::imshow("1. Raw ROI", frame);
    cv::imshow("2. Binary Debug (Labeling)", display);
    cv::waitKey(1);
}
```

### main() : ROS2를 초기화하고 LineTrackerProcessor 노드를 실행한다.

```cpp
int main(int argc, char** argv) {
    // ROS2 초기화
    rclcpp::init(argc, argv);

    // LineTrackerProcessor 노드를 생성하고 스핀(콜백 대기 루프) 실행
    rclcpp::spin(std::make_shared<LineTrackerProcessor>());

    // 노드 종료 시 ROS2 셧다운
    rclcpp::shutdown();
    return 0;
}
```
