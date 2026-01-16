#include "models/pressure.hpp"

// PressureSensor
PressureSensor::PressureSensor(uint8_t id) : id_(id), value_(0) {}

int PressureSensor::set_value(uint16_t value)
{
    value_ = value;
    return 0;
}

uint8_t PressureSensor::get_id() const
{
    return id_;
}

uint16_t PressureSensor::get_value() const
{
    return value_;
}

// Insole
Insole::Insole(const std::vector<uint8_t>& ids)
{
    sensors_.reserve(ids.size());
    for (const auto& id : ids)
    {
        sensors_.emplace_back(id);
    }
}

void Insole::add_pressure_sensor(uint8_t id)
{
    if (!get_pressure_sensor_by_id(id))
        sensors_.emplace_back(id);
}

void Insole::add_pressure_sensors(const std::vector<uint8_t>& ids)
{
    sensors_.reserve(sensors_.size() + ids.size());

    for (const auto& id : ids)
    {
        add_pressure_sensor(id);
    }
}

PressureSensor* Insole::get_pressure_sensor_by_id(uint8_t id)
{
    for (auto& sensor : sensors_)
    {
        if (sensor.get_id() == id) return &sensor;
    }
    return nullptr;
}

std::vector<PressureSensor>& Insole::get_sensors()
{
    return sensors_;
}

const std::vector<PressureSensor>& Insole::get_sensors() const
{
    return sensors_;
}