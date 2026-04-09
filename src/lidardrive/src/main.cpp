#include "lidardrive/lidardrive.hpp"

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv); // 초기화
    auto node = make_shared<LineDetector>(); // 노드 생성
    rclcpp::spin(node); // 실행
    rclcpp::shutdown(); // 끔
    return 0;
}
