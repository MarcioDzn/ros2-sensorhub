#ifndef DYNAMIXEL_CONTROLLER_HPP
#define DYNAMIXEL_CONTROLLER_HPP

#include <string>
#include <memory>

#include "actuator_comm/controller/actuator_controller.hpp"
#include "actuator_comm/link/dynamixel_link.hpp"
#include "actuator_comm/protocol/dynamixel_protocol.hpp"

class DynamixelController : public ActuatorController
{
    public:
        explicit DynamixelController();

        int init(std::string device, int baudrate) override;
        int setTorque(uint8_t id, uint8_t enable_torque) override;
        int setGoalPosition(uint8_t id, uint16_t goal_pos) override;
        int getCurrentPosition(uint8_t id, uint16_t& curr_pos) override;

    private:
        std::shared_ptr<DynamixelLink> link_;
};

#endif // DYNAMIXEL_CONTROLLER_HPP