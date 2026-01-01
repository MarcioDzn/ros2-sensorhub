#include "pressure_comm/core/pressure_factory.hpp"
#include "base_controller.hpp"

std::unique_ptr<PressureController> 
PressureFactory::createPressure()
{
    auto controller = std::make_unique<BaseController>();
    return controller;
}