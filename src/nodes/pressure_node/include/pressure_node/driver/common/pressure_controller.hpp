#ifndef PRESSURE_CONTROLLER_HPP
#define PRESSURE_CONTROLLER_HPP

#include <span>
#include <cstdint>
#include <string>
#include <vector>

class PressureController
{
    public:
        virtual ~PressureController() = default;

        virtual int init(std::string device, int baudrate) = 0;
        virtual int getData(std::vector<uint16_t>& data) = 0;
};

#endif // PRESSURE_CONTROLLER_HPP