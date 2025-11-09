#ifndef CENTRAL_NODE_HPP
#define CENTRAL_NODE_HPP

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "interfaces/msg/imu_data.hpp"

#include "managers/imu_manager.hpp"

using IMUData = interfaces::msg::IMUData;

class CentralNode : public rclcpp::Node
{
    public:
        explicit CentralNode();
        virtual ~CentralNode();

    private:
        std::unique_ptr<IMUManager> imu_manager_;
        rclcpp::TimerBase::SharedPtr timer_;
        size_t count_;

        void timer_callback();
};

#endif // CENTRAL_NODE_HPP