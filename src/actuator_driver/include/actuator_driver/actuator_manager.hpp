#ifndef ACTUATOR_MANAGER_HPP
#define ACTUATOR_MANAGER_HPP

#include <cstdint>
#include <string>
#include <vector>
#include <memory>

#include "actuator_comm/controller/actuator_controller.hpp"
#include "actuator_comm/core/actuator_factory.hpp"
#include "rclcpp/rclcpp.hpp"

struct ActuatorParams {
    int update_rate_ms;
    std::string usb_port;
    uint32_t baudrate;
    std::vector<uint8_t> actuator_ids;
};

class ActuatorManager
{
    public:
        explicit ActuatorManager();

        void get_parameters();

        void init_node(std::shared_ptr<rclcpp::Node> node);
        std::shared_ptr<ActuatorController> get_controller() { return controller_; }
        int init_comm(std::shared_ptr<rclcpp::Node> node);
        int execute_command(std::shared_ptr<rclcpp::Node> node, 
            uint8_t id, const std::string& command, const std::vector<int16_t>& params);
        int set_goal_position(std::shared_ptr<rclcpp::Node> node, 
            uint8_t id, uint16_t goal);

    private:
        void declare_parameters(std::shared_ptr<rclcpp::Node> node);
        void set_parameters(std::shared_ptr<rclcpp::Node> node);

        ActuatorParams parameters_;
        std::shared_ptr<ActuatorController> controller_;

};

#endif // ACTUATOR_MANAGER_HPP