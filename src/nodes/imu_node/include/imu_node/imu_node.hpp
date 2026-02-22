#ifndef IMU_NODE_HPP
#define IMU_NODE_HPP

#include <memory>
#include <vector>
#include <fstream>

#include "imu_manager.hpp"
#include "rclcpp/rclcpp.hpp"

#include "interfaces/msg/imu_data.hpp"
#include "interfaces/msg/imu_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "interfaces/msg/time.hpp"


struct LoopTiming {
    long start = 0;
    long end = 0;
    long total = 0;
};

struct ImuConfig {
    std::string name;
    int id;
    int address;
    int multiplexer;
    std::array<int, 3> euler_order;
};

using IMUData = interfaces::msg::IMUData;
using IMUState = interfaces::msg::IMUState;
using Time = interfaces::msg::Time;

class IMUNode : public rclcpp::Node
{
    public:
        explicit IMUNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
        virtual ~IMUNode();

    private:
        void state_callback();
		
        rclcpp::Publisher<IMUState>::SharedPtr publisher_;
        rclcpp::Publisher<Time>::SharedPtr time_publisher_;

        std::shared_ptr<ParameterManager> parameter_manager_;
        std::shared_ptr<IMUManager> manager_;

        rclcpp::TimerBase::SharedPtr timer_;

        std::ofstream timing_log_;
};

#endif // IMU_NODE_HPP
