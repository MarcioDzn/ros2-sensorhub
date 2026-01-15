#include "driver/dynamixel/dynamixel_controller.hpp"

#include "driver/dynamixel/dynamixel_link.hpp"
#include "driver/dynamixel/dynamixel_protocol.hpp"
#include "common_serial/serial_handler.hpp"

#define TORQUE_ADDR         0x18
#define GOAL_POS_ADDR       0x1E
#define CURRENT_POS_ADDR    0x24

DynamixelController::DynamixelController() {}

int DynamixelController::init(std::string device, int baudrate)
{
    auto transport = std::make_shared<SerialHandler>();
    if (transport->init(device.c_str()) < 0)
        return -1; 
    transport->setDefaultConfig();
    transport->setBaudRate(baudrate);

    auto protocol = std::make_shared<DynamixelProtocol>();

    link_ = std::make_shared<DynamixelLink>(
        transport, protocol);

    return 0;
}

int DynamixelController::setTorque(uint8_t id, uint8_t enable_torque)
{
    if (!link_)
        return -1;

    if (link_->write1Byte(id, TORQUE_ADDR, enable_torque) < 0)
        return -1;
    return 0;
}

int DynamixelController::setGoalPosition(uint8_t id, uint16_t goal_pos)
{
    if (!link_)
        return -1;

    if (link_->write2Byte(id, GOAL_POS_ADDR, goal_pos) < 0)
        return -1;
    return 0;
}

int DynamixelController::getCurrentPosition(uint8_t id, uint16_t& curr_pos)
{
    if (!link_)
        return -1;

    if (link_->read2Byte(id, CURRENT_POS_ADDR, curr_pos) < 0)
        return -1;
    return 0;
}