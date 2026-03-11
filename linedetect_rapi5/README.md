# linedetect_rapi5 패키지 설명

## 1. 프로그램 동작 설명

ROS2 노드로 로컬에 저장된 MP4 영상 파일을 프레임 단위로 읽어 `CompressedImage` 타입의 ROS2 토픽으로 퍼블리시한다. 노드가 실행되면 지정된 영상 파일을 열고, 약 30FPS 주기의 타이머를 통해 반복적으로 프레임을 읽어 압축 이미지 메시지로 변환 후 전송한다. 영상이 끝나면 처음으로 되감아 무한 반복 재생한다.

## 2. 코드 설명

### VideoPublisher() : 노드를 초기화하고, 퍼블리셔 생성, 영상 파일 열기, 타이머 설정을 수행한다.

```cpp
VideoPublisher() : Node("video_publisher_node") {
    // QoS 프로파일을 best_effort 방식으로 설정 (영상 데이터는 일부 손실 허용)
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    // "Image_Topic" 이름의 CompressedImage 퍼블리셔 생성
    raw_image_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("Image_Topic", qos_profile);

    // 비디오 파일 열기
    cap_.open("/home/rapi5/ros2_ws/video/simulation/lanefollow_100rpm_ccw.mp4");

    // 영상 파일이 정상적으로 열리지 않으면 에러 로그 출력
    if (!cap_.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Could not open video file!");
    }

    // 33ms 주기(약 30FPS)로 timer_callback 함수를 반복 호출하는 타이머 생성
    timer_ = this->create_wall_timer(std::chrono::milliseconds(33), std::bind(&VideoPublisher::timer_callback, this));
}
```

### timer_callback() : 영상에서 한 프레임을 읽고, 압축 이미지 메시지로 변환하여 퍼블리시한다.

```cpp
void timer_callback() {
    // 영상에서 한 프레임을 읽어옴
    cv::Mat frame;
    cap_ >> frame;

    // 프레임이 비어있으면(영상 끝) 처음으로 되감아 다시 읽음 (무한 반복 재생)
    if (frame.empty()) {
        cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
        cap_ >> frame;
    }

    // cv_bridge를 사용하여 OpenCV 프레임을 CompressedImage 메시지로 변환 후 퍼블리시
    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toCompressedImageMsg();
    raw_image_pub_->publish(*msg);
}
```

### main() : ROS2를 초기화하고 VideoPublisher 노드를 실행한다.

```cpp
int main(int argc, char** argv) {
    // ROS2 초기화
    rclcpp::init(argc, argv);

    // VideoPublisher 노드를 생성하고 스핀(콜백 대기 루프) 실행
    rclcpp::spin(std::make_shared<VideoPublisher>());

    // 노드 종료 시 ROS2 셧다운
    rclcpp::shutdown();
    return 0;
}
```
