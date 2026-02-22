#include "imu_time_node.hpp"

IMUTimeNode::IMUTimeNode() : Node("imu_time_node")
{
    sub_ = this->create_subscription<interfaces::msg::Time>(
        "/imu/time", 10,
        std::bind(&IMUTimeNode::time_callback, this, std::placeholders::_1)
    );
    RCLCPP_INFO(this->get_logger(), "IMUTimeNode iniciado");
}

void IMUTimeNode::time_callback(const interfaces::msg::Time::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(),
                "[IMU] Total time: %ld us | Names: %lu",
                msg->total_time, msg->names.size());
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMUTimeNode>());
    rclcpp::shutdown();
    return 0;
}