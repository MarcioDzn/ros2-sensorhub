#ifndef CENTRAL_NODE_HPP
#define CENTRAL_NODE_HPP

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include "interfaces/msg/imu_data.hpp"
#include "interfaces/srv/set_motor_config.hpp"

#include "managers/imu_manager.hpp"
#include "managers/motor_manager.hpp"


using IMUData = interfaces::msg::IMUData;
using SetMotorConfig = interfaces::srv::SetMotorConfig;

class CentralNode : public rclcpp::Node
{
    public:
        explicit CentralNode();
        virtual ~CentralNode();

    private:
        std::unique_ptr<IMUManager> imu_manager_;
        std::unique_ptr<MotorManager> motor_manager_;
        rclcpp::Service<SetMotorConfig>::SharedPtr motor_service_;
        
        rclcpp::TimerBase::SharedPtr timer_;
        size_t count_;

        int update_rate_ms_;

        void timer_callback();
        void motor_service_callback(
                const std::shared_ptr<SetMotorConfig::Request> request,
                std::shared_ptr<SetMotorConfig::Response> response);
        void set_goal_position(
                const std::shared_ptr<SetMotorConfig::Request> request,
                std::shared_ptr<SetMotorConfig::Response> response);
        void enable_torque(
                const std::shared_ptr<SetMotorConfig::Request> request,
                std::shared_ptr<SetMotorConfig::Response> response);
        void disable_torque(
                const std::shared_ptr<SetMotorConfig::Request> request,
                std::shared_ptr<SetMotorConfig::Response> response);
        void get_present_position(
                const std::shared_ptr<SetMotorConfig::Request> request,
                std::shared_ptr<SetMotorConfig::Response> response);
};

#endif // CENTRAL_NODE_HPP
