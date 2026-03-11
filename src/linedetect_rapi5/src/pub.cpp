#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"
#include <chrono>

class VideoPublisher : public rclcpp::Node {
public:
    VideoPublisher() : Node("video_publisher_node") {
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
        raw_image_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("Image_Topic", qos_profile);
        
        // 비디오 파일 열기
        cap_.open("/home/rapi5/ros2_ws/video/simulation/lanefollow_100rpm_ccw.mp4"); 
        // cap_.open("/home/rapi5/ros2_ws/video/simulation/lanefollow_100rpm_cw.mp4"); 
        // cap_.open("/home/rapi5/ros2_ws/video/simulation/5_lt_cw_100rpm_out.mp4"); 
        // cap_.open("/home/rapi5/ros2_ws/video/simulation/7_lt_ccw_100rpm_in.mp4"); 
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open video file!");
        }

        // 30FPS 주기로 영상만 전송
        timer_ = this->create_wall_timer(std::chrono::milliseconds(33), std::bind(&VideoPublisher::timer_callback, this));
    }

private:
    void timer_callback() {
        cv::Mat frame;
        cap_ >> frame;

        if (frame.empty()) {
            cap_.set(cv::CAP_PROP_POS_FRAMES, 0); // 무한 반복 재생
            cap_ >> frame;
        }

        // 압축 영상 메시지 생성 및 전송 
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toCompressedImageMsg();
        raw_image_pub_->publish(*msg);
    }

    cv::VideoCapture cap_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr raw_image_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoPublisher>());
    rclcpp::shutdown();
    return 0;
}