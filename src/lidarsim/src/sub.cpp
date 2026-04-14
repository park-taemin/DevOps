/*
 * SLLIDAR ROS2 CLIENT
 *
 * Copyright (c) 2009 - 2014 RoboPeak Team
 * http://www.robopeak.com
 * Copyright (c) 2014 - 2022 Shanghai Slamtec Co., Ltd.
 * http://www.slamtec.com
 *
 */

#include "lidarsim/sub.hpp"
using namespace std;

LineDetector::LineDetector() : Node("sllidar_client"), tmp_pt_l(50, 50), tmp_pt_r(450, 50), mode(false) {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("vel_cmd_topic", qos);

    // [중요] 동영상 파일 불러오기
    cap_.open("/home/linux/ros2_ws/video/ros2_basic_30_test1.mp4"); // 실제 파일 경로로 수정하세요.
    if (!cap_.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "동영상 파일을 열 수 없습니다! 경로를 확인하세요.");
    }

    // 10 FPS 주기에 맞춰 100ms 타이머 설정
    timer_ = this->create_wall_timer(100ms, std::bind(&LineDetector::timer_callback, this));

    k = 1.5;
    RCLCPP_INFO(this->get_logger(), "Lidar Sim Node Started. 's': Start, 'q': Stop");
}

LineDetector::~LineDetector() {
    cap_.release();
    if (video_writer.isOpened()) video_writer.release();
}

void LineDetector::timer_callback() {
    cv::Mat frame;
    cap_ >> frame; // 영상에서 한 프레임 읽기

    if (frame.empty()) {
        cap_.set(cv::CAP_PROP_POS_FRAMES, 0); // 영상이 끝나면 처음으로 되돌림
        return;
    }

    // 1. 전처리 및 레이블링
    cv::Mat roi = preprocess_image(frame);
    cv::Mat stats, centroids;
    cv::connectedComponentsWithStats(roi, labels, stats, centroids);

    // 2. 장애물 탐지 및 시각화
    auto targets = find_target_line(roi, stats, centroids);
    draw_result(frame, stats, centroids, targets.first, targets.second);

    // 3. 제어 명령 계산
    int error = 250 - ((tmp_pt_l.x + tmp_pt_r.x) / 2);
    if (kbhit()) { int ch = getch(); if (ch == 'q') mode = false; else if (ch == 's') mode = true; }

    if (mode) {
        vel.x = 50.0 - k * error;
        vel.y = -(50.0 + k * error);
    } else { vel.x = 0; vel.y = 0; }

    pub_->publish(vel); // 다이내믹셀 제어 노드로 전송

    // 4. 영상 저장 및 출력
    if (!video_writer.isOpened()) {
        video_writer.open("/home/linux/ros2_ws/video/ros2_basic_30_test2.mp4", cv::VideoWriter::fourcc('m','p','4','v'), 10.0, frame.size(), true);
    }
    video_writer.write(frame);
    cv::imshow("Lidar Sim", frame);
    cv::waitKey(1);
}
void LineDetector::draw_result(cv::Mat& result, const cv::Mat& stats, const cv::Mat& centroids, int left_idx, int right_idx)
{
    if (result.channels() == 1) cv::cvtColor(result, result, cv::COLOR_GRAY2BGR);
    cv::Point robot_pos(250, 250); 

    // 모든 화살표 로직을 명확하게 구분되는 색상으로 변경
    // 1. 좌측 장애물: 초록색 (두께 2로 강화)
    cv::arrowedLine(result, robot_pos, tmp_pt_l, cv::Scalar(0, 255, 0), 2); 

    // 2. 우측 장애물: 빨간색
    cv::arrowedLine(result, robot_pos, tmp_pt_r, cv::Scalar(0, 0, 255), 2); 

    // 3. 목표 방향: 파란색 (두께 3으로 강화)
    int target_x = (tmp_pt_l.x + tmp_pt_r.x) / 2;
    cv::Point target_pos(target_x, 100); 
    cv::arrowedLine(result, robot_pos, target_pos, cv::Scalar(255, 0, 0), 3);

    // 디버깅용 포인트 표시 (색상 반전)
    cv::circle(result, tmp_pt_l, 4, cv::Scalar(0, 255, 0), -1); 
    cv::circle(result, tmp_pt_r, 4, cv::Scalar(0, 0, 255), -1); 
}


// preprocess_image, find_target_line, getch, kbhit은 이전 로직과 동일

cv::Mat LineDetector::preprocess_image(const cv::Mat& result) {
    cv::Mat gray, binary;
    cv::cvtColor(result, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, binary, 120, 255, cv::THRESH_BINARY_INV);
    return binary(cv::Rect(0, 0, 500, 250));
}

std::pair<int, int> LineDetector::find_target_line(const cv::Mat& roi, const cv::Mat& stats, const cv::Mat& centroids) {
    int cnt = stats.rows;
    int l_idx = -1, r_idx = -1;
    double min_dist_l = 10000.0, min_dist_r = 10000.0;
    cv::Point robot_pos(250, 250);

    for (int i = 1; i < cnt; i++) {
        if (stats.at<int>(i, 4) < 5) continue; 
        int cx = cvRound(centroids.at<double>(i, 0));
        int cy = cvRound(centroids.at<double>(i, 1));
        double dist = cv::norm(robot_pos - cv::Point(cx, cy));

        // 로봇과 너무 가까운 점은 무시하여 arrowedLine이 화살표를 그릴 수 있도록 수정
        if (dist < 10.0) continue;

        if (cx < 250) { if (dist < min_dist_l) { min_dist_l = dist; l_idx = i; } }
        else { if (dist < min_dist_r) { min_dist_r = dist; r_idx = i; } }
    }
    if (l_idx != -1) tmp_pt_l = cv::Point(cvRound(centroids.at<double>(l_idx, 0)), cvRound(centroids.at<double>(l_idx, 1)));
    else tmp_pt_l = cv::Point(50, 50);
    if (r_idx != -1) tmp_pt_r = cv::Point(cvRound(centroids.at<double>(r_idx, 0)), cvRound(centroids.at<double>(r_idx, 1)));
    else tmp_pt_r = cv::Point(450, 50);
    return make_pair(l_idx, r_idx);
}

int LineDetector::getch() { struct termios oldt, newt; int ch; tcgetattr(0, &oldt); newt = oldt; newt.c_lflag &= ~(ICANON | ECHO); tcsetattr(0, TCSANOW, &newt); ch = getchar(); tcsetattr(0, TCSANOW, &oldt); return ch; }
bool LineDetector::kbhit() { struct termios oldt, newt; int ch, oldf; tcgetattr(0, &oldt); newt = oldt; newt.c_lflag &= ~(ICANON | ECHO); tcsetattr(0, TCSANOW, &newt); oldf = fcntl(0, F_GETFL, 0); fcntl(0, F_SETFL, oldf | O_NONBLOCK); ch = getchar(); tcsetattr(0, TCSANOW, &oldt); fcntl(0, F_SETFL, oldf); if (ch != EOF) { ungetc(ch, stdin); return true; } return false; }
