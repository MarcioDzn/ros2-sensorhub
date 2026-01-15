#include "models/actuator.hpp"

Actuator::Actuator(uint8_t id) {
    id_ = id;
    position_ = 0;
    torque_ = false;
}

int Actuator::set_position(uint16_t position)
{
    if (position < 0 || position > MAX_POSITION)
        return -1;

    position_ = position;
    return 0;
}

void Actuator::set_torque_status(bool status)
{
    torque_ = status;
}

const uint8_t Actuator::get_id() const
{
    return id_;   
}

const uint16_t Actuator::get_position() const
{
    return position_;
}

const uint16_t Actuator::get_position_deg() const
{
    return position_ * (360 / MAX_POSITION);
}

const bool Actuator::get_torque_status() const
{
    return torque_;
}

Actuator::~Actuator() {}