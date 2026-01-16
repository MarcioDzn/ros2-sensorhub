#ifndef PRESSURE_HPP
#define PRESSURE_HPP

#include <cstdint>
#include <vector>
#include <memory>

class PressureSensor
{
    public:
        explicit PressureSensor(uint8_t id);
        virtual ~PressureSensor() = default;

        int set_value(uint16_t value);

        uint8_t get_id() const;
        uint16_t get_value() const;

    private:
        uint8_t id_;
        uint16_t value_;
};

class Insole
{
    public:
        explicit Insole(const std::vector<uint8_t>& ids);
        virtual ~Insole() = default;

        void add_pressure_sensor(uint8_t id);
        void add_pressure_sensors(const std::vector<uint8_t>& ids);
        
        PressureSensor* get_pressure_sensor_by_id(uint8_t id);

        std::vector<PressureSensor>& get_sensors();
        const std::vector<PressureSensor>& get_sensors() const;

    private:
        std::vector<PressureSensor> sensors_;
};



#endif // PRESSURE_HPP