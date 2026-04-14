/*
 * SLLIDAR ROS2 CLIENT
 *
 * Copyright (c) 2009 - 2014 RoboPeak Team
 * http://www.robopeak.com
 * Copyright (c) 2014 - 2022 Shanghai Slamtec Co., Ltd.
 * http://www.slamtec.com
 *
 */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cmath>
#include <math.h>
// OpenCV 라이브러리 추가
#include <opencv2/opencv.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>

#define RAD2DEG(x) ((x)*180./M_PI)

// 동영상 저장을 위한 Writer 객체와 초기화 플래그 (static으로 선언)
static cv::VideoWriter video_writer;
static bool is_video_init = false;

// 스캔 데이터를 이미지로 변환하여 화면에 출력
static cv::Mat visualizeScan(sensor_msgs::msg::LaserScan::SharedPtr scan, int count) {
  cv::Mat image(500, 500, CV_8UC3, cv::Scalar(255, 255, 255));

  // 중심에 십자 표시 (로봇 위치)
  cv::line(image, cv::Point(250, 245), cv::Point(250, 255), cv::Scalar(0, 0, 0), 1);
  cv::line(image, cv::Point(245, 250), cv::Point(255, 250), cv::Scalar(0, 0, 0), 1);

  // 1m = 100px
  double scale = 100.0;

  for (int i = 0; i < count; i++) {
    float distance = scan->ranges[i];
    float angle_rad = scan->angle_min + scan->angle_increment * i;

    if (std::isfinite(distance) && distance > 0) {
      int x = 250 + (int)(distance * scale * sin(angle_rad));
      int y = 250 + (int)(distance * scale * cos(angle_rad));

      if (x >= 0 && x < 500 && y >= 0 && y < 500) {
        cv::circle(image, cv::Point(x, y), 2, cv::Scalar(0, 0, 255), -1);
      }
    }
  }

  cv::imshow("LIDAR Scan", image);
  cv::waitKey(1);

  return image;
}

// 이미지를 동영상 파일에 저장
static void saveVideo(const cv::Mat& image) {
  if (!is_video_init) {
    video_writer.open("/home/linux/ros2_ws/video/ros2_basic_30_test1.mp4",
                      cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 10, cv::Size(500, 500), true);
    is_video_init = true;
    printf("[SLLIDAR INFO]: Video recording started.\n");
  }

  if (video_writer.isOpened()) {
    video_writer.write(image);
  }
}

static void scanCb(sensor_msgs::msg::LaserScan::SharedPtr scan) {
  int count = scan->scan_time / scan->time_increment;

  // 안전을 위해 ranges 사이즈로 count 재설정 (데이터 개수 불일치 방지)
  if(scan->ranges.size() > 0) count = scan->ranges.size();

  printf("[SLLIDAR INFO]: I heard a laser scan %s[%d]:\n", scan->header.frame_id.c_str(), count);
  printf("[SLLIDAR INFO]: angle_range : [%f, %f]\n", RAD2DEG(scan->angle_min),
         RAD2DEG(scan->angle_max));

  cv::Mat image = visualizeScan(scan, count);
  saveVideo(image);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("sllidar_client");

  auto lidar_info_sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
                        "scan", rclcpp::SensorDataQoS(), scanCb);

  rclcpp::spin(node);

  rclcpp::shutdown();

  // 종료 시 비디오 파일 닫기
  if (video_writer.isOpened()) {
    video_writer.release();
  }

  return 0;
}
