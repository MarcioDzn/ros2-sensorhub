#include <gtest/gtest.h>
#include "dynamixel/dynamixel_protocol.hpp"

TEST(actuator_driver, dynamixel_packet_base_creation)
{
    uint8_t length = 0x02;
    auto protocol = std::make_shared<DynamixelProtocol>();
    auto packet = protocol->createPacketBase();

    std::vector<uint8_t> packet_base = {0xFF, 0xFF, 0, length, 0};
    ASSERT_EQ(packet_base, packet);
}

TEST(actuator_driver, dynamixel_packet_set_header)
{
    auto protocol = std::make_shared<DynamixelProtocol>();
    auto packet = protocol->createPacketBase();
    packet = protocol->setHeader(packet, 1, 0x03);

    EXPECT_EQ(packet[ID_POS], 1);
    EXPECT_EQ(packet[INSTRUCTION_POS], 0x03);
}

TEST(actuator_driver, dynamixel_packet_set_payload)
{
    uint8_t params[2] = {0x05, 2};

    auto protocol = std::make_shared<DynamixelProtocol>();
    auto packet = protocol->createPacketBase();
    uint8_t prev_length = packet[LENGTH_POS];

    packet = protocol->setPayload(packet, params);

    // length deve aumentar em 2 bytes (payload)
    EXPECT_EQ(packet[LENGTH_POS], prev_length+2);

    ASSERT_GE(packet.size(), PARAMETER_POS + 2);
    EXPECT_EQ(packet[PARAMETER_POS], params[0]);
    EXPECT_EQ(packet[PARAMETER_POS+1], params[1]);
}

TEST(actuator_driver, dynamixel_packet_checksum_is_appended_and_correct)
{
    uint8_t params[2] = {0x05, 0x02};

    auto protocol = std::make_shared<DynamixelProtocol>();

    auto packet = protocol->createPacketBase();
    packet = protocol->setHeader(packet, 1, 0x03);
    packet = protocol->setPayload(packet, params);

    size_t size_before = packet.size();
    packet = protocol->setChecksum(packet);

    // tamanho aumentou
    ASSERT_EQ(packet.size(), size_before + 1);

    // checksum correto
    uint16_t sum = 0;
    for (size_t i = ID_POS; i < packet.size() - 1; i++)
    {
        sum += packet[i];
    }

    uint8_t expected_checksum = ~(sum & 0xFF);

    EXPECT_EQ(packet.back(), expected_checksum);
}

TEST(actuator_driver, dynamixel_payload_empty_does_not_change_packet)
{
    auto protocol = std::make_shared<DynamixelProtocol>();
    auto packet = protocol->createPacketBase();

    uint8_t prev_length = packet[LENGTH_POS];
    size_t prev_size = packet.size();

    packet = protocol->setPayload(packet, {});

    EXPECT_EQ(packet[LENGTH_POS], prev_length);
    EXPECT_EQ(packet.size(), prev_size);
}
