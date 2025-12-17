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

        struct ImuConfig {
            std::string name;
            int id;
            int address;
            int multiplexer;
            std::array<int, 3> euler_order;
        };

        void timer_callback();
        void load_parameters();
        void set_parameters();
        void parse_imus(std::vector<std::string> param_names);
		
        std::vector<std::shared_ptr<BNO055IMU>> imus_;
        std::vector<rclcpp::Publisher<IMUData>::SharedPtr> publishers_;
        
        rclcpp::TimerBase::SharedPtr timer_;
        int update_rate_ms_;
        std::vector<ImuConfig> imus_config_;
};

#endif // IMU_NODE_HPP
