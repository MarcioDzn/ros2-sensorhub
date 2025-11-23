#ifndef PACKET_BUILDER_BASE_HPP
#define PACKET_BUILDER_BASE_HPP

#include <cstdint>
#include <vector>

class PacketBuilderBase
{
    public:
        virtual ~PacketBuilderBase() = 0;
        virtual PacketBuilderBase& startPacket() = 0;
        virtual PacketBuilderBase& setHeader(uint8_t header) = 0;
        virtual PacketBuilderBase& setParamLength(uint8_t length) = 0;
        virtual PacketBuilderBase& setChecksum() = 0;
        virtual PacketBuilderBase& clear() = 0;
        virtual PacketBuilderBase& setID(uint8_t id) = 0;
        virtual PacketBuilderBase& setInstruction(uint8_t instruction) = 0;
        virtual PacketBuilderBase& addParameter(uint8_t* param) = 0;
        virtual std::vector<uint8_t>& build() = 0;
};

inline PacketBuilderBase::~PacketBuilderBase() {}

#endif // PACKET_BUILDER_BASE_HPP