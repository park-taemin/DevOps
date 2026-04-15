#include "lidardrive/lidardrive.hpp"

// =====================================================================
// 생성자 - 노드 초기화, 토픽 구독/발행 설정
// =====================================================================
LineDetector::LineDetector() : Node("sllidar_client"),  // 노드 이름: sllidar_client
    tmp_pt_l(125, 125),   // 좌측 장애물 초기 위치 (화면 좌측 상단)
    tmp_pt_r(375, 125),   // 우측 장애물 초기 위치 (화면 우측 상단)
    first_run_(true),     // 첫 실행 여부 플래그
    mode(false) {         // 주행 모드 초기값: false (정지 상태)

    vel.x = 0;  // 왼쪽 바퀴 속도 초기값 (rpm)
    vel.y = 0;  // 오른쪽 바퀴 속도 초기값 (rpm)
    vel.z = 0;  // 사용 안 함

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)); // 최근 10개 메시지 유지하는 QoS 설정

    // /scan 토픽 구독 설정 → 메시지 수신 시 mysub_callback() 자동 호출
    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan",           // 구독할 토픽 이름
        qos_profile,      // QoS 설정
        bind(&LineDetector::mysub_callback, this, placeholders::_1)); // 콜백 함수 등록

    // vel_cmd_topic 발행 설정 → RPi5의 node_dxlsub가 구독
    pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(
        "vel_cmd_topic",  // 발행할 토픽 이름
        qos_profile);     // QoS 설정
}

// =====================================================================
// preprocess_image() - 이진화 및 ROI 추출
// 입력: BGR 컬러 이미지 (result)
// 출력: 이진화된 전방 영역 이미지 (roi)
// =====================================================================
cv::Mat LineDetector::preprocess_image(const cv::Mat& result) {
    cv::Mat frame_gray;
    cv::cvtColor(result, frame_gray, cv::COLOR_BGR2GRAY); // BGR 컬러 → 흑백 변환

    cv::Mat frame_binary;
    cv::threshold(frame_gray, frame_binary, 100, 255, cv::THRESH_BINARY_INV);
    // 이진화(반전): 픽셀값 < 100(어두움=장애물) → 255(흰색)
    //               픽셀값 > 100(밝음=빈공간)   → 0(검정)
    // 임계값 100: lidarsim(120)보다 낮아 더 민감하게 장애물 검출

    return frame_binary(cv::Rect(0, 0, 500, 250));
    // 상단 절반(500×250)만 잘라서 반환 = 로봇 전방 영역(ROI)
    // 뒤쪽 장애물은 회피 불필요하므로 전방만 분석
}

// =====================================================================
// find_target_line() - 좌/우 최단거리 장애물 탐색
// 입력: roi(이진화 이미지), stats(클러스터 정보), centroids(무게중심)
// 출력: std::pair<int,int> (좌측 장애물 인덱스, 우측 장애물 인덱스)
// =====================================================================
std::pair<int, int> LineDetector::find_target_line(const cv::Mat& roi, const cv::Mat& stats, const cv::Mat& centroids) {
    int cnt = stats.rows;   // 탐지된 전체 클러스터 수 (배경 포함)
    int l_idx = -1;         // 좌측 장애물 인덱스 (-1 = 미탐지)
    int r_idx = -1;         // 우측 장애물 인덱스 (-1 = 미탐지)

    double min_dist_l = 100000.0;  // 좌측 최단거리 초기값 (매우 큰 값)
    double min_dist_r = 100000.0;  // 우측 최단거리 초기값 (매우 큰 값)

    cv::Point robot_pos(250, 250); // 로봇 위치 = 이미지 중심(250, 250)

    for (int i = 1; i < cnt; i++) { // i=0은 배경이므로 1부터 시작

        int cx = cvRound(centroids.at<double>(i, 0)); // i번째 클러스터 무게중심 x좌표
        int cy = cvRound(centroids.at<double>(i, 1)); // i번째 클러스터 무게중심 y좌표
        cv::Point obj_pos(cx, cy);                    // 클러스터 위치

        double dist = cv::norm(robot_pos - obj_pos);  // 로봇과 클러스터 사이 거리 계산

        if (cx < 250) {              // 무게중심이 화면 왼쪽 → 좌측 장애물
            if (dist < min_dist_l) { // 기존 최단거리보다 가까우면
                min_dist_l = dist;   // 최단거리 갱신
                l_idx = i;           // 좌측 장애물 인덱스 갱신
            }
        }
        else {                       // 무게중심이 화면 오른쪽 → 우측 장애물
            if (dist < min_dist_r) { // 기존 최단거리보다 가까우면
                min_dist_r = dist;   // 최단거리 갱신
                r_idx = i;           // 우측 장애물 인덱스 갱신
            }
        }
    }

    // 좌측 장애물 무게중심 좌표 업데이트
    if (l_idx != -1) {
        // 탐지 성공: 실제 무게중심 좌표로 업데이트
        tmp_pt_l = cv::Point(cvRound(centroids.at<double>(l_idx, 0)), cvRound(centroids.at<double>(l_idx, 1)));
    } else {
        tmp_pt_l = cv::Point(0, 0);   // 미탐지: 왼쪽 끝으로 설정 → 우측으로 회피 유도
    }

    // 우측 장애물 무게중심 좌표 업데이트
    if (r_idx != -1) {
        // 탐지 성공: 실제 무게중심 좌표로 업데이트
        tmp_pt_r = cv::Point(cvRound(centroids.at<double>(r_idx, 0)), cvRound(centroids.at<double>(r_idx, 1)));
    } else {
        tmp_pt_r = cv::Point(500, 0); // 미탐지: 오른쪽 끝으로 설정 → 좌측으로 회피 유도
    }

    return std::make_pair(l_idx, r_idx); // 좌/우 장애물 인덱스 반환
}

// =====================================================================
// draw_result() - 결과 시각화
// 탐지된 장애물에 바운딩 박스, 화살표, 목표 방향 표시
// =====================================================================
void LineDetector::draw_result(cv::Mat& result, const cv::Mat& stats, const cv::Mat& centroids, int left_idx, int right_idx)
{
    if (result.channels() == 1) cv::cvtColor(result, result, cv::COLOR_GRAY2BGR); // 흑백이면 컬러로 변환

    int cnt = stats.rows;          // 전체 클러스터 수
    cv::Point robot_pos(250, 250); // 로봇 위치 = 이미지 중심

    for (int i = 1; i < cnt; i++) {          // 배경(i=0) 제외하고 반복
        int area = stats.at<int>(i, 4);      // i번째 클러스터 면적(픽셀 수)
        if (area > 0) {                       // 면적이 있는 클러스터만 처리
            if (i == left_idx || i == right_idx) { // 좌측 또는 우측 장애물만 표시
                int x = cvRound(centroids.at<double>(i, 0)); // 무게중심 x
                int y = cvRound(centroids.at<double>(i, 1)); // 무게중심 y

                int left   = stats.at<int>(i, 0); // 바운딩 박스 왼쪽 x
                int top    = stats.at<int>(i, 1); // 바운딩 박스 위쪽 y
                int width  = stats.at<int>(i, 2); // 바운딩 박스 너비
                int height = stats.at<int>(i, 3); // 바운딩 박스 높이

                cv::Scalar color;
                if (i == left_idx) color = cv::Scalar(0, 255, 0); // 좌측 장애물: 초록색
                else               color = cv::Scalar(0, 0, 255); // 우측 장애물: 빨간색

                cv::rectangle(result, cv::Rect(left, top, width, height), color, 2); // 바운딩 박스 표시
                cv::circle(result, cv::Point(x, y), 3, color, -1);                   // 무게중심 점 표시
            }
        }
    }

    // 좌측 장애물 → 초록 화살표 (로봇 → 바운딩 박스 우하단 꼭짓점)
    if (left_idx != -1) {
        int left   = stats.at<int>(left_idx, 0); // 바운딩 박스 왼쪽 x
        int top    = stats.at<int>(left_idx, 1); // 바운딩 박스 위쪽 y
        int width  = stats.at<int>(left_idx, 2); // 바운딩 박스 너비
        int height = stats.at<int>(left_idx, 3); // 바운딩 박스 높이

        cv::Point l_box_bottom_right(left + width, top + height); // 바운딩 박스 우하단 꼭짓점

        cv::arrowedLine(result, robot_pos, l_box_bottom_right, cv::Scalar(0, 255, 0), 1); // 초록 화살표
    }

    // 우측 장애물 → 빨간 화살표 (로봇 → 바운딩 박스 좌하단 꼭짓점)
    if (right_idx != -1) {
        int left   = stats.at<int>(right_idx, 0); // 바운딩 박스 왼쪽 x
        int top    = stats.at<int>(right_idx, 1); // 바운딩 박스 위쪽 y
        int height = stats.at<int>(right_idx, 3); // 바운딩 박스 높이

        cv::Point r_box_bottom_left(left, top + height); // 바운딩 박스 좌하단 꼭짓점

        cv::arrowedLine(result, robot_pos, r_box_bottom_left, cv::Scalar(0, 0, 255), 1); // 빨간 화살표
    }

    int target_x = (tmp_pt_l.x + tmp_pt_r.x) / 2; // 좌/우 무게중심의 중간점 x = 통로 중심
    cv::Point target_pos(target_x, 100);            // 목표 방향 위치 (y=100 고정)

    cv::arrowedLine(result, robot_pos, target_pos, cv::Scalar(255, 0, 0), 1); // 파란 화살표 (목표 방향)

    cv::circle(result, tmp_pt_l, 3, cv::Scalar(0, 255, 0), -1); // 좌측 무게중심 초록 점
    cv::circle(result, tmp_pt_r, 3, cv::Scalar(0, 0, 255), -1); // 우측 무게중심 빨간 점
}

// =====================================================================
// getch() - Enter 없이 키보드 입력 1개 읽기
// =====================================================================
int LineDetector::getch(void)
{
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);        // 현재 터미널 설정 저장
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);     // 줄버퍼 모드 해제, 에코 끄기
    tcsetattr(STDIN_FILENO, TCSANOW, &newt); // 새 터미널 설정 적용
    ch = getchar();                        // 키 입력 1개 읽기
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // 원래 터미널 설정 복구
    return ch;                             // 입력된 키 반환
}

// =====================================================================
// kbhit() - 키보드 입력이 있는지 확인 (논블로킹)
// 반환: true(키 입력 있음), false(키 입력 없음)
// =====================================================================
bool LineDetector::kbhit(void)
{
    struct termios oldt, newt;
    int ch;
    int oldf;
    tcgetattr(STDIN_FILENO, &oldt);          // 현재 터미널 설정 저장
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);       // 줄버퍼 모드 해제, 에코 끄기
    tcsetattr(STDIN_FILENO, TCSANOW, &newt); // 새 터미널 설정 적용
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0); // 현재 파일 플래그 저장
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK); // 논블로킹 모드 설정 (입력 없어도 바로 반환)
    ch = getchar();                          // 키 입력 시도
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // 원래 터미널 설정 복구
    fcntl(STDIN_FILENO, F_SETFL, oldf);      // 원래 파일 플래그 복구
    if (ch != EOF) {         // 키 입력이 있으면
        ungetc(ch, stdin);   // 읽은 문자를 버퍼에 되돌려 놓기 (getch()에서 다시 읽을 수 있게)
        return true;         // 키 입력 있음
    }
    return false;            // 키 입력 없음
}

// =====================================================================
// mysub_callback() - /scan 토픽 수신 시 자동 호출 (10Hz = 0.1초마다)
// 전체 처리 흐름: 스캔 수신 → 이미지 생성 → 장애물 탐지 → 속도 계산 → 발행
// =====================================================================
void LineDetector::mysub_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    auto startTime = chrono::steady_clock::now(); // 처리 시작 시간 기록 (처리시간 측정용)
    int count = scan->scan_time / scan->time_increment; // 총 스캔 포인트 수 계산

    // ① 500×500 흰색 이미지 생성 (중심(250,250) = 로봇 위치)
    cv::Mat scan_video(500, 500, CV_8UC3, cv::Scalar(255, 255, 255));

    float scale = 8.0; // 표시 범위 조정 (1m ≈ 80px, 최대 약 3.1m 범위 표시)

    for (int i = 0; i < count; i++) {
        // 극좌표(거리, 각도) → 직교좌표(x, y) 변환
        // x = 중심 + 거리 * scale * sin(각도)
        float x = 250 + scan->ranges[i] * ((10.0 * scale) * std::sin(scan->angle_min + scan->angle_increment * i));
        // y = 중심 - 거리 * scale * cos(각도) (y축 반전: 위쪽이 양수 방향)
        float y = 250 - scan->ranges[i] * ((10.0 * scale) * std::cos(scan->angle_min + scan->angle_increment * i));

        cv::circle(scan_video, cv::Point((int)x, (int)y), 1, cv::Scalar(0, 0, 255), -1); // 빨간 점으로 장애물 표시
    }

    // ② 방향 보정: LIDAR 좌표계와 이미지 좌표계 차이를 보정
    cv::Mat rotate = cv::getRotationMatrix2D(cv::Point(250, 250), 180, 1); // 180° 회전 행렬 생성
    cv::Mat result;
    cv::warpAffine(scan_video, result, rotate, scan_video.size()); // 180° 회전 적용
    cv::flip(result, result, 1);      // 좌우 반전 → 이미지 상단 = 로봇 전방
    cv::namedWindow("Lidar 2D map"); // 표시 창 생성

    // ③ 전처리: 이진화 및 전방 ROI 추출
    cv::Mat roi = preprocess_image(result);

    // ④ 연결 컴포넌트 분석: 흰색 픽셀 덩어리(장애물) 탐지
    cv::Mat labels, stats, centroids;
    cv::connectedComponentsWithStats(roi, labels, stats, centroids);
    // labels   : 각 픽셀의 클러스터 번호
    // stats    : 각 클러스터의 위치/크기 정보 (바운딩 박스)
    // centroids: 각 클러스터의 무게중심 좌표

    // ⑤ 좌/우 최단거리 장애물 인덱스 탐색
    std::pair<int, int> targets = find_target_line(roi, stats, centroids);
    int left_idx  = targets.first;  // 좌측 장애물 인덱스
    int right_idx = targets.second; // 우측 장애물 인덱스

    // ⑥ 결과 시각화 (바운딩 박스, 화살표, 목표 방향 표시)
    draw_result(result, stats, centroids, left_idx, right_idx);

    // ⑦ error 계산: 통로 중심이 화면 중심(250)에서 얼마나 벗어났는지
    int error = 250 - ((tmp_pt_l.x + tmp_pt_r.x) / 2);
    // error > 0: 통로가 왼쪽 → 좌회전 필요
    // error < 0: 통로가 오른쪽 → 우회전 필요
    // error = 0: 통로가 정중앙 → 직진

    // ⑧ 키보드 입력 처리
    if(kbhit()) {           // 키 입력이 있으면
        int ch = getch();   // 입력된 키 읽기
        if(ch == 'q') mode = false; // q키: 정지 모드
        else if(ch == 's') mode = true; // s키: 주행 모드
    }

    // ⑨ 속도 명령 계산
    if (mode) { // 주행 모드일 때만 속도 명령 계산
        if (error == 0 || error < -60 || error > 60) {
            // error=0(정중앙) 또는 |error|>60(오탐지 의심) → 직진
            vel.x = 50;   // 왼쪽 바퀴 전진 속도 (rpm)
            vel.y = -50;  // 오른쪽 바퀴 전진 속도 (반대 방향이라 음수)
        }
        else {
            // 0 < |error| ≤ 60 → 비례 조향 (게인 k=0.45)
            vel.x = 50 - k * error;     // 왼쪽 바퀴: error 양수면 감속, 음수면 가속
            vel.y = -(50 + k * error);  // 오른쪽 바퀴: error 양수면 가속, 음수면 감속
        }
    }
    else {
        vel.x = 0; // 정지 모드: 모든 바퀴 정지
        vel.y = 0;
    }

    pub_->publish(vel); // vel_cmd_topic으로 속도 명령 발행 → RPi5의 node_dxlsub가 수신

    auto endTime = chrono::steady_clock::now(); // 처리 종료 시간 기록
    float totalTime = chrono::duration<float, milli>(endTime - startTime).count(); // 처리 시간 계산 (ms)

    // ⑩ 동영상 저장
    static cv::VideoWriter video_writer;       // static: 함수 종료 후에도 객체 유지
    static bool is_writer_initialized = false; // VideoWriter 초기화 여부 플래그

    if (!is_writer_initialized) { // 첫 프레임에서만 초기화
        video_writer.open(
            "/home/linux/ros2_ws/video/ros2_basic_30_test3.mp4", // 저장 경로
            cv::VideoWriter::fourcc('m', 'p', '4', 'v'),          // mp4v 코덱
            10.0,                                                  // FPS (LIDAR 주기 10Hz와 동일)
            cv::Size(result.cols, result.rows),                    // 프레임 크기
            true);                                                 // 컬러 저장

        if (video_writer.isOpened()) {
            is_writer_initialized = true; // 초기화 성공
            printf("Video Writer Initialized Successfully.\n");
        } else {
            printf("Failed to open Video Writer.\n"); // 초기화 실패
        }
    }

    if (is_writer_initialized) {
        video_writer.write(result); // 현재 프레임을 mp4 파일에 저장
    }

    // ⑪ 처리 결과 터미널 출력
    RCLCPP_INFO(this->get_logger(),
        "Received Image : err:%d, leftvel: %.2f, rightvel: %.2f, time:%.2f ms",
        error, vel.x, vel.y, totalTime);
    // error: 통로 중심 오차
    // leftvel: 왼쪽 바퀴 속도
    // rightvel: 오른쪽 바퀴 속도
    // time: 1프레임 처리 시간 (ms)

    cv::imshow("Lidar 2D map", result); // 처리 결과 이미지 화면 출력
    cv::waitKey(1);                     // 1ms 대기 (OpenCV 창 업데이트)
}
