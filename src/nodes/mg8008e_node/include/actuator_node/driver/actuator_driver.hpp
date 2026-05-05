#ifndef ACTUATOR_DRIVER_HPP
#define ACTUATOR_DRIVER_HPP

#include <string>
#include <cstdint>

class IActuatorDriver
{
    public:
        virtual ~IActuatorDriver() = default;

        virtual int init(std::string device, int baudrate) = 0;
        virtual int set_angle(uint8_t id, int32_t angle, int32_t speed) = 0;
        virtual int get_angle(uint8_t id, double& angle) = 0;
};

#endif // ACTUATOR_DRIVER_HPP