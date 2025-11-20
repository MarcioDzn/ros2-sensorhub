#ifndef DYNAMIEL_PACKET_BUILDER_HPP
#define DYNAMIEL_PACKET_BUILDER_HPP

#include "device_comm/packet_builder.hpp"

#include <vector>

class DynamixelPacketBuilder : public PacketBuilderBase
{
    public:
        DynamixelPacketBuilder();
        DynamixelPacketBuilder& startPacket() override;
        DynamixelPacketBuilder& setHeader(uint8_t header) override;
        DynamixelPacketBuilder& setParamLength(uint8_t length) override;
        DynamixelPacketBuilder& setChecksum() override;
        DynamixelPacketBuilder& clear() override;
        const uint8_t* build() override;

        DynamixelPacketBuilder& setID(uint8_t id);
        DynamixelPacketBuilder& setInstruction(uint8_t instruction);
        DynamixelPacketBuilder& setAddress(uint8_t address);
        DynamixelPacketBuilder& addParameter(uint8_t* param);

    private:
        enum class State {
            Start,
            ID,
            Length,
            Instruction,
            Address,
            Params,
            Header,
            Checksum,
            Build
        };

        std::vector<uint8_t> buffer_;
        uint8_t param_length_;
        State state_;


        void checkState(State expected);

};

#endif // DYNAMIEL_PACKET_BUILDER_HPP