#ifndef ACTUATOR_LINK_HPP
#define ACTUATOR_LINK_HPP

#include <cstdint>
#include <vector>
#include <memory>
#include <chrono>

#include "actuator_comm/protocol/actuator_protocol.hpp"
#include "common_serial/serial_handler.hpp"

struct StatusPacket
{
    uint8_t id;
    uint8_t error;
    std::vector<uint8_t> params;
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
        virtual int readPacket(uint8_t* packet) = 0;
        virtual int readStatus(uint8_t id, StatusPacket& out) = 0;

        std::shared_ptr<ActuatorProtocol> protocol_;
        std::shared_ptr<SerialHandler> transport_;
};


#endif // ACTUATOR_LINK_HPP