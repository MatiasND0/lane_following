#include <memory>

#include <rclcpp/rclcpp.hpp>

// Declaración de factory implementada en src/pipeline/lane_pipeline_node.cpp
std::shared_ptr<rclcpp::Node> make_lane_pipeline_node();

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(make_lane_pipeline_node());
    rclcpp::shutdown();
    return 0;
}
