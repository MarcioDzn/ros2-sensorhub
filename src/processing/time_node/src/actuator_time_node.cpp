#include "actuator_time_node.hpp"

ActuatorTimeNode::ActuatorTimeNode() : Node("actuator_time_node")
{
    sub_ = this->create_subscription<interfaces::msg::Time>(
        "/actuator/time", 10,
        std::bind(&ActuatorTimeNode::time_callback, this, std::placeholders::_1)
    );
    RCLCPP_INFO(this->get_logger(), "ActuatorTimeNode iniciado");
}

void ActuatorTimeNode::time_callback(const interfaces::msg::Time::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(),
                "[Actuator] Total time: %ld us | Names: %lu",
                msg->total_time, msg->names.size());
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ActuatorTimeNode>());
    rclcpp::shutdown();
    return 0;
}