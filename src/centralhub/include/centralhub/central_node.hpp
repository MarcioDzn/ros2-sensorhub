#ifndef CENTRAL_NODE_HPP
#define CENTRAL_NODE_HPP

#include <memory>
#include <chrono>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "interfaces/msg/imu_data.hpp"

using IMUData = interfaces::msg::IMUData;

class CentralNode : public rclcpp::Node
{
    public:
        explicit CentralNode();
        virtual ~CentralNode();

    private:
        std::vector<rclcpp::Publisher<IMUData>::SharedPtr> publishers_;
        rclcpp::TimerBase::SharedPtr timer_;

        size_t count_;
        std::vector<size_t> imu_ids_;

        void timer_callback();
        void get_imu_data(int id, std::vector<double>& imu_data);
};

#endif // CENTRAL_NODE_HPP