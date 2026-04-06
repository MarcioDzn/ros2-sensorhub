#ifndef CLIENT_HPP
#define CLIENT_HPP

#include "rclcpp/rclcpp.hpp"

#include "interfaces/msg/actuator_state.hpp"

#include "interfaces/msg/pressure_state.hpp"
#include "interfaces/msg/pressure_data.hpp"
#include "interfaces/msg/pressure_unit_sensor.hpp"

#include "interfaces/msg/imu_data.hpp"
#include "interfaces/msg/imu_state.hpp"

#include "interfaces/msg/synced_sensor_data.hpp"

class ClientNode : public rclcpp::Node
{
    public:
        explicit ClientNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        virtual ~ClientNode();
                
        void run(int argc, char **argv);
        void execute_path(); 
        
    private:
        rclcpp::Publisher<interfaces::msg::ActuatorState>::SharedPtr actuator_publisher_;
        rclcpp::Publisher<interfaces::msg::PressureState>::SharedPtr pressure_publisher_;
        rclcpp::Publisher<interfaces::msg::IMUState>::SharedPtr imu_publisher_;
};

#endif // CLIENT_HPP
