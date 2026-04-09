#include "lidardrive/lidardrive.hpp"

LineDetector::LineDetector() : Node("sllidar_client"), tmp_pt_l(125, 125), tmp_pt_r(375, 125), first_run_(true), mode(false) {
    vel.x = 0;
    vel.y = 0;
    vel.z = 0;

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    
    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 
        qos_profile, 
        bind(&LineDetector::mysub_callback, this, placeholders::_1));
        
    pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(
        "vel_cmd_topic",
        qos_profile);
}

cv::Mat LineDetector::preprocess_image(const cv::Mat& result) {
    cv::Mat frame_gray;
    cv::cvtColor(result, frame_gray, cv::COLOR_BGR2GRAY);

    cv::Mat frame_binary;
    cv::threshold(frame_gray, frame_binary, 100, 255, cv::THRESH_BINARY_INV);

    return frame_binary(cv::Rect(0, 0, 500, 250));
}

std::pair<int, int> LineDetector::find_target_line(const cv::Mat& roi, const cv::Mat& stats, const cv::Mat& centroids) {
    int cnt = stats.rows;
    int l_idx = -1;
    int r_idx = -1;
    
    double min_dist_l = 100000.0;
    double min_dist_r = 100000.0;

    cv::Point robot_pos(250, 250); 

    for (int i = 1; i < cnt; i++) {

        int cx = cvRound(centroids.at<double>(i, 0));
        int cy = cvRound(centroids.at<double>(i, 1));
        cv::Point obj_pos(cx, cy);

        double dist = cv::norm(robot_pos - obj_pos);

        if (cx < 250) {
            if (dist < min_dist_l) {
                min_dist_l = dist;
                l_idx = i;
            }
        }
        else {
            if (dist < min_dist_r) {
                min_dist_r = dist;
                r_idx = i;
            }
        }
    }
    
    // [좌표 업데이트]
    if (l_idx != -1) {
        tmp_pt_l = cv::Point(cvRound(centroids.at<double>(l_idx, 0)), cvRound(centroids.at<double>(l_idx, 1)));
    } else {
        tmp_pt_l = cv::Point(0, 0); // 못 찾으면 왼쪽 구석으로
    }

    if (r_idx != -1) {
        tmp_pt_r = cv::Point(cvRound(centroids.at<double>(r_idx, 0)), cvRound(centroids.at<double>(r_idx, 1)));
    } else {
        tmp_pt_r = cv::Point(500, 0); // 못 찾으면 오른쪽 구석으로
    }

    return std::make_pair(l_idx, r_idx);
}

void LineDetector::draw_result(cv::Mat& result, const cv::Mat& stats, const cv::Mat& centroids, int left_idx, int right_idx)
{
    if (result.channels() == 1) cv::cvtColor(result, result, cv::COLOR_GRAY2BGR);

    int cnt = stats.rows;

    cv::Point robot_pos(250, 250); 

    for (int i = 1; i < cnt; i++) {
        int area = stats.at<int>(i, 4);
        if (area > 0) {
            if (i == left_idx || i == right_idx) {
                int x = cvRound(centroids.at<double>(i, 0));
                int y = cvRound(centroids.at<double>(i, 1));
                
                int left = stats.at<int>(i, 0);
                int top = stats.at<int>(i, 1);
                int width = stats.at<int>(i, 2);
                int height = stats.at<int>(i, 3);

                cv::Scalar color; 
                if (i == left_idx) color = cv::Scalar(0, 255, 0);
                else color = cv::Scalar(0, 0, 255);

                cv::rectangle(result, cv::Rect(left, top, width, height), color, 2);
                cv::circle(result, cv::Point(x, y), 3, color, -1);
            }
        }
    }

    if (left_idx != -1) {
        int left = stats.at<int>(left_idx, 0);
        int top = stats.at<int>(left_idx, 1);
        int width = stats.at<int>(left_idx, 2);
        int height = stats.at<int>(left_idx, 3);
        
        cv::Point l_box_bottom_right(left + width, top + height);
        
        cv::arrowedLine(result, robot_pos, l_box_bottom_right, cv::Scalar(0, 255, 0), 1);
    }

    if (right_idx != -1) {
        int left = stats.at<int>(right_idx, 0);
        int top = stats.at<int>(right_idx, 1);
        int height = stats.at<int>(right_idx, 3);
        
        cv::Point r_box_bottom_left(left, top + height);
        
        cv::arrowedLine(result, robot_pos, r_box_bottom_left, cv::Scalar(0, 0, 255), 1);
    }

    int target_x = (tmp_pt_l.x + tmp_pt_r.x) / 2;
    cv::Point target_pos(target_x, 100); 
    
    cv::arrowedLine(result, robot_pos, target_pos, cv::Scalar(255, 0, 0), 1);

    cv::circle(result, tmp_pt_l, 3, cv::Scalar(0, 255, 0), -1); 
    cv::circle(result, tmp_pt_r, 3, cv::Scalar(0, 0, 255), -1); 
}

// 키보드처리
int LineDetector::getch(void) 
{
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

bool LineDetector::kbhit(void) 
{
    struct termios oldt, newt;
    int ch;
    int oldf;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
    if (ch != EOF)
    {
        ungetc(ch, stdin);
        return true;
    }
    return false;
}

void LineDetector::mysub_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    auto startTime = chrono::steady_clock::now();
    int count = scan->scan_time / scan->time_increment;

    // 윈도우 창 생성 및 이름 지정, 흰색
    cv::Mat scan_video(500, 500, CV_8UC3, cv::Scalar(255, 255, 255));

    float scale = 8.0; // 보이는 범위 조정 scale
    for (int i = 0; i < count; i++) {
		// 극좌표 -> 직교좌표로 변환하기, x = r * cos(theta), y = r * sin(theta)
		float x = 250 + scan->ranges[i] * ((10.0 * scale) * std::sin(scan->angle_min + scan->angle_increment * i));
		float y = 250 - scan->ranges[i] * ((10.0 * scale) * std::cos(scan->angle_min + scan->angle_increment * i));
		
    // 스캔 영상 그리기 코드
		cv::circle(scan_video, cv::Point((int)x, (int)y), 1, cv::Scalar(0, 0, 255), -1);
    }
	cv::Mat rotate = cv::getRotationMatrix2D(cv::Point(250, 250), 180, 1);
	cv::Mat result;
	cv::warpAffine(scan_video, result, rotate, scan_video.size());
	cv::flip(result, result, 1);
	cv::namedWindow("Lidar 2D map");

    cv::Mat roi = preprocess_image(result);

    cv::Mat labels, stats, centroids;
    cv::connectedComponentsWithStats(roi, labels, stats, centroids);

    std::pair<int, int> targets = find_target_line(roi, stats, centroids);

    int left_idx = targets.first;
    int right_idx = targets.second;

    draw_result(result, stats, centroids, left_idx, right_idx);

    int error = 250 - ((tmp_pt_l.x + tmp_pt_r.x) / 2);

    if(kbhit()) {
        int ch = getch();
        if(ch == 'q') mode = false;
        else if(ch == 's') mode = true;
    }

    if (mode) {
        if (error == 0 || error < -60 || error > 60){
            vel.x = 50;
            vel.y = -50;
        }
        else {
            vel.x = 50 - k * error;
            vel.y = -(50 + k * error);
        }
    } 
    else {
        vel.x = 0;
        vel.y = 0;
    }

    pub_->publish(vel);

    auto endTime = chrono::steady_clock::now();
    float totalTime = chrono::duration<float, milli>(endTime - startTime).count();

    // 스캔 영상 화면 출력 및 동영상 저장 코드
    static cv::VideoWriter video_writer; // static으로 선언하여 함수가 종료되어도 객체 유지
    static bool is_writer_initialized = false; // 초기화 여부 확인 플래그

    // 아직 초기화가 안 됐다면 (첫 번째 프레임이 들어왔을 때)
    if (!is_writer_initialized) {
        // 저장할 파일 이름, 코덱, FPS, 프레임 크기(가로, 세로), 컬러 여부
        video_writer.open("/home/linux/ros2_ws/video/ros2_basic_30_test3.mp4", cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 10.0, cv::Size(result.cols, result.rows), true);
        
        if (video_writer.isOpened()) {
            is_writer_initialized = true; // 성공했으면 true로 바꿔야 함
            printf("Video Writer Initialized Successfully.\n");
        } else {
            printf("Failed to open Video Writer.\n");
        }
    }

    // 비디오 파일에 현재 프레임 쓰기
    if (is_writer_initialized) {
        video_writer.write(result);
    }


    RCLCPP_INFO(this->get_logger(), "Received Image : err:%d, leftvel: %.2f, rightvel: %.2f, time:%.2f ms", error, vel.x, vel.y, totalTime);

    cv::imshow("Lidar 2D map", result);
    cv::waitKey(1);
}
