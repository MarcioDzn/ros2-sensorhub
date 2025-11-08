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

        // parâmetros
        std::vector<long> imu_ids_;
        std::vector<long> imu_addresses_;
        std::vector<long> multiplex_ids_;
        std::vector<std::vector<int>> euler_orders_;
        std::vector<std::string> imu_names_;
        int update_rate_ms_;

        void timer_callback();
        void get_imu_data(int id, std::vector<double>& imu_data);
        std::vector<std::vector<int>> chunk_vector(
            const std::vector<int64_t>& flat, 
            size_t group_size);
};

#endif // CENTRAL_NODE_HPP