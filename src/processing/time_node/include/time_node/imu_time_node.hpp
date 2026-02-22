#ifndef IMU_TIME_NODE_HPP
#define IMU_TIME_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/time.hpp"

class IMUTimeNode : public rclcpp::Node
{
    public:
        IMUTimeNode();

    private:
        void time_callback(const interfaces::msg::Time::SharedPtr msg);
        rclcpp::Subscription<interfaces::msg::Time>::SharedPtr sub_;
};

#endif //IMU_TIME_NODE_HPP