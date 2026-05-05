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
            MULTI_LOOP_2            = 0XA4,
            READ_MULTI_LOOP_2       = 0X92,
        } mg8008e_modes;

        typedef enum
        {
            HEADER_POS              = 0,
            COMMAND_POS             = 1,
            ID_POS                  = 2,
            LENGTH_POS              = 3,
            FRAME_TYPE_POS          = 4,
            PAYLOAD_START_POS       = 5
        } mg8008e_packet;


        typedef enum
        {
            MULTI_LOOP_2_TYPE       = 0xEF
        } mg8008e_frame_type;

        typedef enum
        {
            READ_INSTR              = ,
            WRITE_INSTR             = 
        } mg8008e_protocol_instructions;

        explicit MG8008EDriver();
        virtual ~MG8008EDriver() = default;

        int init(std::string device, int baudrate) override;
        int set_torque(uint8_t id, uint8_t enable_torque) override;
        int set_goal_position(uint8_t id, int32_t goal_position, int32_t speed);
        int get_current_position(uint8_t id, uint16_t& current_position) override;
    
    private:
        struct StatusPacket
        {
            uint8_t id;
            uint8_t error;
            std::array<uint8_t, RXPACKET_MAX_LEN> params;
        };

        std::vector<uint8_t> get_packet(
            const uint8_t id, 
            const uint16_t command, 
            const uint16_t frame_type,
            const std::vector<uint8_t>& params);

        int read_packet(std::array<uint8_t, RXPACKET_MAX_LEN>& packet);
        int read_status(uint8_t id, StatusPacket& out);
        std::vector<uint8_t> get4bytes(int32_t value);

        std::unique_ptr<SerialHandler> transport_;
        
};

#endif // MG8008E_DRIVER_HPP
