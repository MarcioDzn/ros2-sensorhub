#include "driver/pressure_factory.hpp"
#include "driver/base_controller.hpp"

std::unique_ptr<PressureController> 
PressureFactory::createPressure()
{
    auto controller = std::make_unique<BaseController>();
    return controller;
}