#ifndef MG8008E_DRIVER_HPP
#define MG8008E_DRIVER_HPP

#include "driver/actuator_driver.hpp"
#include "common_serial/serial_handler.hpp"

#include <vector>
#include <array>
#include <memory>

class SerialHandler;

class MG8008EDriver : public IActuatorDriver
{
    public:
        typedef enum
        {
            PREAMBLE_POS            = ,
            ID_POS                  = ,
            LENGTH_POS              = ,
            INSTRUCTION_POS         = ,
            ERROR_POS               = ,
            PARAMETER_POS           = ,

            MIN_PAYLOAD_SIZE        = ,
            RXPACKET_MAX_LEN        = ()
        } mg8008e_packet;

        typedef enum
        {
            TORQUE_ADDR             = ,
            GOAL_POS_ADDR           = ,
            CURRENT_POS_ADDR        = 
        } mg8008e_protocol_addresses;

        typedef enum
        {
            READ_INSTR              = ,
            WRITE_INSTR             = 
        } mg8008e_protocol_instructions;

        explicit MG8008EDriver();
        virtual ~MG8008EDriver() = default;

        int init(std::string device, int baudrate) override;
        int set_torque(uint8_t id, uint8_t enable_torque) override;
        int set_goal_position(uint8_t id, uint16_t goal_position) override;
        int get_current_position(uint8_t id, uint16_t& current_position) override;
    
    private:
        struct StatusPacket
        {
            uint8_t id;
            uint8_t error;
            std::array<uint8_t, RXPACKET_MAX_LEN> params;
        };

        std::vector<uint8_t> get_packet(
            uint8_t id, uint8_t instr, const std::vector<uint8_t>& params);
        int read_packet(std::array<uint8_t, RXPACKET_MAX_LEN>& packet);
        int read_status(uint8_t id, StatusPacket& out);

        std::unique_ptr<SerialHandler> transport_;
        
};

#endif // MG8008E_DRIVER_HPP
