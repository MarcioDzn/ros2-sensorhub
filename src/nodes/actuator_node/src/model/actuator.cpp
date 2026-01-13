#include "model/actuator.hpp"

Actuator::Actuator(uint8_t id) {
    id_ = id;
    position_ = 0;
}

void Actuator::set_position(uint16_t position)
{
    if (position < 0 || position > MAX_POSITION)
        return;

    position_ = position;
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

Actuator::~Actuator() {}