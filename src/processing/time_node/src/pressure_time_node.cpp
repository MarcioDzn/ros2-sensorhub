#include "pressure_time_node.hpp"

PressureTimeNode::PressureTimeNode() : Node("pressure_time_node")
{
    sub_ = this->create_subscription<interfaces::msg::Time>(
        "/pressure/time", 10,
        std::bind(&PressureTimeNode::time_callback, this, std::placeholders::_1)
    );
    RCLCPP_INFO(this->get_logger(), "PressureTimeNode iniciado");
}

void PressureTimeNode::time_callback(const interfaces::msg::Time::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(),
                "[Pressure] Total time: %ld us | Names: %lu",
                msg->total_time, msg->names.size());
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PressureTimeNode>());
    rclcpp::shutdown();
    return 0;
}