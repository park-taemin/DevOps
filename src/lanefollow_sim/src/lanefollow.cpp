#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"
#include <cmath>
#include <chrono>
#include <termios.h>
#include <fcntl.h>

class LineTrackerProcessor : public rclcpp::Node {
public:
    LineTrackerProcessor() : Node("lane_tracker_node"), mode_(false), k_(0.14), base_vel_(100) {
        // 파라미터 선언 및 초기화
        this->declare_parameter("k", 0.14);
        this->declare_parameter("base_vel", 100);
        k_ = this->get_parameter("k").as_double();
        base_vel_ = this->get_parameter("base_vel").as_int();

        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

        raw_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "Image_Topic", qos_profile,
            std::bind(&LineTrackerProcessor::image_callback, this, std::placeholders::_1));

        vel_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("vel_cmd_topic", qos_profile);

        // 초기 차선 위치 설정 (좌측: 160, 우측: 480 부근 가정)
        last_left_x_ = 150.0;
        last_left_y_ = 45.0;
        last_right_x_ = 540.0;
        last_right_y_ = 45.0;
        
        RCLCPP_INFO(this->get_logger(), "차선 분석 및 제어 노드 시작. 's': 주행, 'q': 정지");
    }

private:
    // --- 1. 전처리 함수 ---
    cv::Mat setROI(cv::Mat &frame) {
        cv::Rect roi_rect(0, frame.rows * 3 / 4, frame.cols, frame.rows / 4);
        cv::Mat roi = frame(roi_rect).clone();

        cv::cvtColor(roi, roi, cv::COLOR_BGR2GRAY);
        roi += cv::Scalar(100) - cv::mean(roi); 
        cv::threshold(roi, roi, 150, 255, cv::THRESH_BINARY);
        
        return roi;
    }

    // --- 2. 차선 탐색 함수 (좌/우 독립적 2단계 추적) ---
    void findLanes(cv::Mat &bin_roi, cv::Mat &stats, cv::Mat &centroids, int &l_idx, int &r_idx) {
        int n_labels = cv::connectedComponentsWithStats(bin_roi, labels_, stats, centroids);
        const double mid_line = bin_roi.cols / 2.0;

        // [1단계] 좌/우 각각 가장 가까운 후보 찾기
        int l_min_idx = -1, r_min_idx = -1;
        double l_min_dist = mid_line, r_min_dist = mid_line;

        for (int i = 1; i < n_labels; i++) {
            int area = stats.at<int>(i, cv::CC_STAT_AREA);
            if (area < 300) continue;

            double cx = centroids.at<double>(i, 0);
            double cy = centroids.at<double>(i, 1);

            if (cx < mid_line) { // 좌측 영역
                double dist = cv::norm(cv::Point2d(cx, cy) - cv::Point2d(last_left_x_, last_left_y_));
                if (dist < l_min_dist && dist <= 80.0) { l_min_dist = dist; l_min_idx = i; }
            } else { // 우측 영역
                double dist = cv::norm(cv::Point2d(cx, cy) - cv::Point2d(last_right_x_, last_right_y_));
                if (dist < r_min_dist && dist <= 90.0) { r_min_dist = dist; r_min_idx = i; }
            }
        }

        // 1차 갱신 (좌/우 각각)
        if (l_min_idx != -1) { last_left_x_ = centroids.at<double>(l_min_idx, 0); last_left_y_ = centroids.at<double>(l_min_idx, 1); }
        if (r_min_idx != -1) { last_right_x_ = centroids.at<double>(r_min_idx, 0); last_right_y_ = centroids.at<double>(r_min_idx, 1); }

        // [2단계] 확정된 위치에서 가장 가까운 blob 인덱스 반환 (검증용)
        l_idx = -1; r_idx = -1;
        double l_best = 50.0, r_best = 50.0;

        for (int i = 1; i < n_labels; i++) {
            double cx = centroids.at<double>(i, 0);
            double cy = centroids.at<double>(i, 1);
            
            double d_l = cv::norm(cv::Point2d(cx, cy) - cv::Point2d(last_left_x_, last_left_y_));
            double d_r = cv::norm(cv::Point2d(cx, cy) - cv::Point2d(last_right_x_, last_right_y_));

            if (d_l < l_best) { l_best = d_l; l_idx = i; }
            if (d_r < r_best) { r_best = d_r; r_idx = i; }
        }
    }

    // --- 3. 시각화 함수 ---
    cv::Mat drawResult(cv::Mat &bin_roi, cv::Mat &stats, int l_idx, int r_idx) {
        cv::Mat display;
        cv::cvtColor(bin_roi, display, cv::COLOR_GRAY2BGR);

        for (int i = 1; i < stats.rows; i++) {
            int area = stats.at<int>(i, cv::CC_STAT_AREA);
            if (area < 300) continue;

            int l = stats.at<int>(i, 0), t = stats.at<int>(i, 1), w = stats.at<int>(i, 2), h = stats.at<int>(i, 3);
            
            // 검출된 차선(좌/우)은 빨간색, 나머지는 파란색
            cv::Scalar color = (i == l_idx || i == r_idx) ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 0, 0);
            cv::rectangle(display, cv::Rect(l, t, w, h), color, 2);
        }
        
        // 현재 추적 중인 차선 무게중심 표시
        cv::circle(display, cv::Point(static_cast<int>(last_left_x_), static_cast<int>(last_left_y_)), 3, cv::Scalar(0, 0, 255), -1);
        cv::circle(display, cv::Point(static_cast<int>(last_right_x_), static_cast<int>(last_right_y_)), 3, cv::Scalar(0, 0, 255), -1);
        
        return display;
    }

    // --- 4. 메인 콜백 함수 ---
    void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        auto startTime = std::chrono::steady_clock::now();
        
        k_ = this->get_parameter("k").as_double();
        base_vel_ = this->get_parameter("base_vel").as_int();

        checkKeyboard();

        cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
        if (frame.empty()) return;

        cv::Mat bin_roi = setROI(frame);
        cv::Mat stats, centroids;
        int l_idx, r_idx;
        
        findLanes(bin_roi, stats, centroids, l_idx, r_idx);
        cv::Mat display = drawResult(bin_roi, stats, l_idx, r_idx);

        // [중요] 오차 계산: 영상 중심 - 차선 중점
        double lane_midpoint = (last_left_x_ + last_right_x_) / 2.0;
        double error = (bin_roi.cols / 2.0) - lane_midpoint;
        
        geometry_msgs::msg::Vector3 vel_msg;
        if (mode_) {
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
        RCLCPP_INFO(this->get_logger(), "err:%.2lf lvel:%.2f rvel:%.2f time:%.2f", error,vel_msg.x,vel_msg.y, totalTime);
    }

    // --- 유틸리티 및 키보드 제어 ---
    void checkKeyboard() {
        if (kbhit()) {
            char ch = getch();
            if (ch == 'q') { mode_ = false; RCLCPP_WARN(this->get_logger(), "STOP"); }
            else if (ch == 's') { mode_ = true; RCLCPP_INFO(this->get_logger(), "START"); }
        }
    }
    int getch() { struct termios oldt, newt; int ch; tcgetattr(STDIN_FILENO, &oldt); newt = oldt; newt.c_lflag &= ~(ICANON | ECHO); tcsetattr(STDIN_FILENO, TCSANOW, &newt); ch = getchar(); tcsetattr(STDIN_FILENO, TCSANOW, &oldt); return ch; }
    bool kbhit() { struct termios oldt, newt; int ch, oldf; tcgetattr(STDIN_FILENO, &oldt); newt = oldt; newt.c_lflag &= ~(ICANON | ECHO); tcsetattr(STDIN_FILENO, TCSANOW, &newt); oldf = fcntl(STDIN_FILENO, F_GETFL, 0); fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK); ch = getchar(); tcsetattr(STDIN_FILENO, TCSANOW, &oldt); fcntl(STDIN_FILENO, F_SETFL, oldf); if (ch != EOF) { ungetc(ch, stdin); return true; } return false; }

    double last_left_x_, last_left_y_, last_right_x_, last_right_y_;
    bool mode_;
    double k_;
    int base_vel_;
    cv::Mat labels_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr raw_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr vel_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LineTrackerProcessor>());
    rclcpp::shutdown();
    return 0;
}
