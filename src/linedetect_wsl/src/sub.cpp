#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "std_msgs/msg/float64.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"
#include <cmath>
#include <chrono>

class LineTrackerProcessor : public rclcpp::Node {
public:
    LineTrackerProcessor() : Node("line_tracker_processor_node") {
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

        // 원본 영상 구독
        raw_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "Image_Topic", qos_profile,
            std::bind(&LineTrackerProcessor::image_callback, this, std::placeholders::_1));

        // 초기 라인 위치 설정
        last_line_x_ = 320.0;
        last_line_y_ = 45.0;
        
        RCLCPP_INFO(this->get_logger(), "WSL 기반 분석 노드가 시작되었습니다.");
    }

    ~LineTrackerProcessor() { cv::destroyAllWindows(); }

private:
    void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        auto startTime = std::chrono::steady_clock::now(); // 처리 시간 측정 시작

        // 1. 영상 복원
        cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
        if (frame.empty()) return;

        // 2. ROI 설정 및 전처리
        cv::Rect roi_rect(0, 270, 640, 90); //
        cv::Mat roi = frame(roi_rect);
        cv::Mat gray, bin, display;

        cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
        double avg = cv::mean(gray)[0];
        gray.convertTo(gray, -1, 1, 100 - static_cast<int>(avg)); // 밝기 보정
        cv::threshold(gray, bin, 128, 255, cv::THRESH_BINARY);
        
        cv::cvtColor(bin, display, cv::COLOR_GRAY2BGR); // 이진 배경 컬러화

        // 3. 레이블링 및 객체 검출
        cv::Mat labels, stats, centroids;
        int n_labels = cv::connectedComponentsWithStats(bin, labels, stats, centroids);

        double min_dist = 1e9;
        int best_idx = -1;

        for (int i = 1; i < n_labels; i++) {
            int area = stats.at<int>(i, cv::CC_STAT_AREA);
            if (area < 100 || area > 15000) continue; // 면적 노이즈 제거

            int left = stats.at<int>(i, cv::CC_STAT_LEFT);
            int top = stats.at<int>(i, cv::CC_STAT_TOP);
            int width = stats.at<int>(i, cv::CC_STAT_WIDTH);
            int height = stats.at<int>(i, cv::CC_STAT_HEIGHT);
            cv::rectangle(display, cv::Rect(left, top, width, height), cv::Scalar(255, 0, 0), 2); // Blue Box

            double cx = centroids.at<double>(i, 0);
            double cy = centroids.at<double>(i, 1);
            double dist = std::sqrt(std::pow(cx - last_line_x_, 2) + std::pow(cy - last_line_y_, 2));

            if (dist < min_dist && dist < 150.0) { // 근접 라인 선택
                min_dist = dist;
                best_idx = i;
            }
        }

        // 4. 라인 위치 업데이트 및 시각화
        if (best_idx != -1) {
            last_line_x_ = centroids.at<double>(best_idx, 0);
            last_line_y_ = centroids.at<double>(best_idx, 1);
            
            cv::rectangle(display, cv::Rect(stats.at<int>(best_idx, 0), stats.at<int>(best_idx, 1), 
                          stats.at<int>(best_idx, 2), stats.at<int>(best_idx, 3)), cv::Scalar(0, 0, 255), 3);
            cv::circle(display, cv::Point(static_cast<int>(last_line_x_), static_cast<int>(last_line_y_)), 5, cv::Scalar(0, 0, 255), -1);
        }

        // 5. 오차 계산 및 출력
        double error = (bin.cols / 2.0) - last_line_x_;

        // 6. 결과 윈도우 띄우기
        cv::imshow("1. Raw ROI", frame);
        cv::imshow("2. Binary Debug (Labeling)", display);
        cv::waitKey(1);
        auto endTime = std::chrono::steady_clock::now();
        float totalTime = std::chrono::duration<float, std::milli>(endTime - startTime).count();

        RCLCPP_INFO(this->get_logger(), "Error: %.2f | Processing Time: %.2f ms", error, totalTime);

    }

    double last_line_x_, last_line_y_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr raw_sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LineTrackerProcessor>());
    rclcpp::shutdown();
    return 0;
}