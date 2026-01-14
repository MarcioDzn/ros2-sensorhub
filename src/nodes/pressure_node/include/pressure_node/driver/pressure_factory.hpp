#ifndef PRESSURE_FACTORY_HPP
#define PRESSURE_FACTORY_HPP

#include <memory>
#include <string>

#include "driver/common/pressure_controller.hpp"

class PressureFactory
{
    public:
        static std::unique_ptr<PressureController> 
        createPressure();
};

#endif // PRESSURE_FACTORY_HPP