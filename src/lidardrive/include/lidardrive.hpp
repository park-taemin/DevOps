
#ifndef SUB_HPP_
#define SUB_HPP_

#include "rclcpp/rclcpp.hpp" // ROS 2 기본 헤더
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "opencv2/opencv.hpp" // OpenCV 헤더
#include <memory> 
#include <chrono>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <utility>
#include <math.h>
#define STDIN_FILENO 0 
#define RAD2DEG(x) ((x)*180./M_PI)

using namespace std;

class LineDetector : public rclcpp::Node { // 클래스 상속
public:
    LineDetector(); // 생성자
private:
    void mysub_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan); // 메인 콜백 함수 선언

    cv::Mat preprocess_image(const cv::Mat& frame_color); // 전처리 함수 선언
    std::pair<int, int> find_target_line(const cv::Mat& roi, const cv::Mat& stats, const cv::Mat& centroids); // 라인 탐색 함수 선언
    void draw_result(cv::Mat& result, const cv::Mat& stats, const cv::Mat& centroids, int l_idx, int r_idx); // 시각화 함수 선언
    int getch(void); // 엔터키 입력 없이 한 문자만 입력시 바로 리턴
    bool kbhit(void); // 키보드가 눌렸는지 체크해주는 함수
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_; // 서브스크라이버 변수
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_; // 퍼블리셔 변수
    cv::Point tmp_pt_l; // 라인 위치 기억할 변수
    cv::Point tmp_pt_r; // 라인 위치 기억할 변수
    bool first_run_; // 첫 실행인지 확인할 플래그
    bool mode = false;
    geometry_msgs::msg::Vector3 vel;
    double k = 0.45;
};

#endif // SUB_HPP_
