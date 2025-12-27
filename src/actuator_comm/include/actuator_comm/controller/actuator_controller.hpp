#ifndef ACTUATOR_CONTROLLER_HPP
#define ACTUATOR_CONTROLLER_HPP

#include <span>
#include <cstdint>

class ActuatorController
{
    public:
        virtual int init(std::string device, int baudrate) = 0;
        virtual int setTorque(uint8_t id, uint8_t enable_torque) = 0;
        virtual int setGoalPosition(uint8_t id, uint16_t goal_pos) = 0;
        virtual int getCurrentPosition(uint8_t id, uint16_t& curr_pos) = 0;
};

#endif // ACTUATOR_CONTROLLER_HPP