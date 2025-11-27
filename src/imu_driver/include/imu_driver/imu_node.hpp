#ifndef IMU_NODE_HPP
#define IMU_NODE_HPP

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "imu_lib.hpp"
#include "interfaces/msg/imu_data.hpp"

using IMUData = interfaces::msg::IMUData;

class IMUNode : public rclcpp::Node
{
    public:
        explicit IMUNode();
        virtual ~IMUNode();

    private:
        void timer_callback();
        std::vector<std::vector<int>> chunk_vector(
            const std::vector<int64_t>& flat, 
            size_t group_size);
        void load_parameters();
        void set_parameters();
		
        std::vector<std::shared_ptr<BNO055IMU>> imus_;
        std::vector<rclcpp::Publisher<IMUData>::SharedPtr> publishers_;
        
        rclcpp::TimerBase::SharedPtr timer_;
        int update_rate_ms_;
        
        // parâmetros
        std::vector<int64_t> imu_ids_;
        std::vector<int64_t> imu_addresses_;
        std::vector<int64_t> multiplex_ids_;
        std::vector<std::vector<int>> euler_orders_;
        std::vector<std::string> imu_names_;
};

#endif // IMU_NODE_HPP
