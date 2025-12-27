#ifndef ACTUATOR_LINK_HPP
#define ACTUATOR_LINK_HPP

#include <span>
#include <cstdint>
#include <vector>
#include <memory>
#include <chrono>
#include <array>

#include "actuator_comm/protocol/actuator_protocol.hpp"
#include "common_serial/serial_handler.hpp"

#define RXPACKET_MAX_LEN    (250)

struct StatusPacket
{
    uint8_t id;
    uint8_t error;
    std::array<uint8_t, RXPACKET_MAX_LEN> params;
};

class ActuatorLink
{
    public:
        explicit ActuatorLink(
            std::shared_ptr<ActuatorProtocol> protocol,
            std::shared_ptr<SerialHandler> transport)
            :   protocol_(std::move(protocol)), 
                transport_(std::move(transport)) {}

        virtual int write1Byte(uint8_t id, uint8_t address, uint8_t data) = 0;
        virtual int write2Byte(uint8_t id, uint8_t address, uint16_t data) = 0;
        virtual int read1Byte(uint8_t id, uint8_t address, uint8_t& read_data) = 0;
        virtual int read2Byte(uint8_t id, uint8_t address, uint16_t& read_data) = 0;
        virtual ~ActuatorLink() = default;

    protected:
        std::shared_ptr<ActuatorProtocol> protocol_;
        std::shared_ptr<SerialHandler> transport_;
};


#endif // ACTUATOR_LINK_HPP