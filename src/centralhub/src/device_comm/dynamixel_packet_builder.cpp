#include "device_comm/dynamixel_packet_builder.hpp"
#include <cstdlib>
#include <stdexcept>

#define PARAM_LIMIT_SIZE 16
#define PACKET_OVERHEAD_SIZE 7

#define HEADER_POSITION0 0
#define HEADER_POSITION1 1
#define ID_POSITION 2
#define INFO_SIZE_POSITION 3
#define INSTRUCTION_POSITION 4

#define ADDRESS_POSITION 5


DynamixelPacketBuilder::DynamixelPacketBuilder(){}

DynamixelPacketBuilder& DynamixelPacketBuilder::startPacket()
{
    checkState(State::Start);

    clear();
    buffer_.resize(PACKET_OVERHEAD_SIZE);

    state_ = State::ID;

    return *this;
    
}

DynamixelPacketBuilder& DynamixelPacketBuilder::setID(uint8_t id)
{
    checkState(State::ID);

    buffer_[ID_POSITION] = id;

    state_ = State::Length;
    return *this;
}

DynamixelPacketBuilder& DynamixelPacketBuilder::setParamLength(uint8_t length)
{
    checkState(State::Length);

    if (length > PARAM_LIMIT_SIZE) 
        throw std::runtime_error("Param length exceeds limit");

    param_length_ = length;

    // tamanho total do pacote
    buffer_.resize(PACKET_OVERHEAD_SIZE+param_length_);
    buffer_[INFO_SIZE_POSITION] = param_length_+3;

    state_ = State::Address;
    return *this;
}

DynamixelPacketBuilder& DynamixelPacketBuilder::setAddress(uint8_t address)
{
    checkState(State::Address);

    buffer_[ADDRESS_POSITION] = address;

    state_ = State::Instruction;
    return *this;
}

DynamixelPacketBuilder& DynamixelPacketBuilder::setInstruction(uint8_t instruction)
{
    checkState(State::Instruction);

    buffer_[INSTRUCTION_POSITION] = instruction;

    state_ = State::Params;
    return *this;
}

DynamixelPacketBuilder& DynamixelPacketBuilder::addParameter(uint8_t* param)
{
    checkState(State::Params);

    for (int i = 0; i < param_length_;i++)
    {
        buffer_[ADDRESS_POSITION+1+i] = param[i];
    }

    state_ = State::Header;
    return *this;
}


DynamixelPacketBuilder& DynamixelPacketBuilder::setHeader(uint8_t header)
{
    checkState(State::Header);

    buffer_[HEADER_POSITION0] = header;
    buffer_[HEADER_POSITION1] = header;

    state_ = State::Checksum;
    return *this;
    
}

DynamixelPacketBuilder& DynamixelPacketBuilder::setChecksum()
{
    checkState(State::Checksum);

    uint8_t checksum = 0;
    for (size_t i = 2; i < buffer_.size()-1; i++)
    {
        checksum += buffer_[i];
    }
    buffer_[buffer_.size()-1] = ~checksum;

    state_ = State::Build;
    return *this;
}

std::vector<uint8_t>& DynamixelPacketBuilder::build()
{
    checkState(State::Build);
    state_ = State::Start;
    return buffer_;
}

DynamixelPacketBuilder& DynamixelPacketBuilder::clear()
{
    buffer_.clear();
    return *this;
}

DynamixelPacketBuilder::~DynamixelPacketBuilder()
{
    state_ = State::Start;
    clear();
}

void DynamixelPacketBuilder::checkState(State expected)
{
    if (state_ != expected)
        throw std::runtime_error("Invalid step called in builder");    
}