#include "lidarsim/sub.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LineDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
