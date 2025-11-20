#ifndef PACKET_BUILDER_BASE_HPP
#define PACKET_BUILDER_BASE_HPP

#include <cstdint>

class PacketBuilderBase
{
    public:
        virtual ~PacketBuilderBase() = 0;
        virtual PacketBuilderBase& startPacket() = 0;
        virtual PacketBuilderBase& setHeader(uint8_t header) = 0;
        virtual PacketBuilderBase& setParamLength(uint8_t length) = 0;
        virtual PacketBuilderBase& setChecksum() = 0;
        virtual PacketBuilderBase& clear() = 0;
        virtual const uint8_t* build() = 0;
};

#endif // PACKET_BUILDER_BASE_HPP