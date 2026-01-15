#ifndef ACTUATOR_HPP
#define ACTUATOR_HPP

#include <cstdint>

#define MAX_POSITION 4096

class Actuator 
{
    public:
        explicit Actuator(uint8_t id);
        virtual ~Actuator();

        int set_position(uint16_t position);
        void set_torque_status(bool status);

        const uint8_t get_id() const;
        const uint16_t get_position() const;
        const uint16_t get_position_deg() const;
        const bool get_torque_status() const;

    private:
        uint8_t id_;
        uint16_t position_;
        bool torque_;

};

#endif // ACTUATOR_HPP