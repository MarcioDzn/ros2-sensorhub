#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "dynamixel/dynamixel_link.hpp"
#include "dynamixel/dynamixel_protocol.hpp"

class MockSerialHandler : public SerialHandler
{
public:
    MOCK_METHOD(ssize_t, writeData,
        (const uint8_t*, size_t), (override));
    MOCK_METHOD(ssize_t, readData,
        (uint8_t*, size_t), (override));
};

class TestableDynamixelLink : public DynamixelLink {
public:
    using DynamixelLink::DynamixelLink;  // herda construtores
    using DynamixelLink::readPacket;     // torna readPacket público
};

TEST(actuator_driver, dynamixel_read_packet_success)
{
    auto protocol = std::make_shared<DynamixelProtocol>();
    auto transport = std::make_shared<MockSerialHandler>();
    TestableDynamixelLink link(transport, protocol);

    std::array<uint8_t, RXPACKET_MAX_LEN> rxbuffer;

    EXPECT_CALL(*transport, readData(::testing::_, ::testing::_))
        .WillOnce(::testing::Invoke(
            [](uint8_t* buffer, size_t size) -> ssize_t {
                std::array<uint8_t, 6> rx = {0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC};
                size_t to_copy = std::min(size, rx.size());
                std::copy(rx.begin(), rx.begin() + to_copy, buffer);
                return to_copy;
            }
        ));

    int result = link.readPacket(rxbuffer);

    EXPECT_EQ(result, 0);
}

TEST(actuator_driver, dynamixel_read_packet_checksum_error)
{
    auto protocol = std::make_shared<DynamixelProtocol>();
    auto transport = std::make_shared<MockSerialHandler>();
    TestableDynamixelLink link(transport, protocol);

    std::array<uint8_t, RXPACKET_MAX_LEN> rxbuffer;

    EXPECT_CALL(*transport, readData(::testing::_, ::testing::_))
        .WillOnce(::testing::Invoke(
            [](uint8_t* buffer, size_t size) -> ssize_t {
                std::array<uint8_t, 6> rx = {0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFD};
                size_t to_copy = std::min(size, rx.size());
                std::copy(rx.begin(), rx.begin() + to_copy, buffer);
                return to_copy;
            }
        ));

    int result = link.readPacket(rxbuffer);

    EXPECT_EQ(result, -1);
}

TEST(actuator_driver, dynamixel_read_packet_content)
{
    // testa a busca de conteúdo considerando que
    // o readData não vai entregar os 6 bytes mínimos
    // de uma vez
    auto protocol = std::make_shared<DynamixelProtocol>();
    auto transport = std::make_shared<MockSerialHandler>();
    TestableDynamixelLink link(transport, protocol);

    std::array<uint8_t, RXPACKET_MAX_LEN> rxbuffer;

    std::array<uint8_t, 8> full_rx = {
        0xFF, 0xFF, 0x01, 0x04, 0x02, 0x03, 0x00, 0xF5
    };
    size_t offset = 0;

    EXPECT_CALL(*transport, readData(::testing::_, ::testing::_))
        .WillRepeatedly(::testing::Invoke(
            [&](uint8_t* buffer, size_t size) -> ssize_t {
                size_t remaining = full_rx.size() - offset;
                if (remaining == 0) return 0;
                
                // pega no máximo 3 bytes
                size_t to_copy = std::min<size_t>({size, remaining, 3});
                std::copy(full_rx.begin() + offset, full_rx.begin() + offset + to_copy, buffer);
                offset += to_copy;

                return to_copy;
            }
        ));

    int result = link.readPacket(rxbuffer);

    std::array<uint8_t, 8> expected_rx = {
        0xFF, 0xFF, 0x01, 0x04, 0x02, 0x03, 0x00, 0xF5
    };
    EXPECT_TRUE(std::equal(rxbuffer.begin(), rxbuffer.begin() + expected_rx.size(), expected_rx.begin()));
    EXPECT_EQ(result, 0);
}

TEST(actuator_driver, dynamixel_read_packet_content_trash)
{
    // testa a busca de conteúdo considerando que
    // o readData vai entregar lixo também
    auto protocol = std::make_shared<DynamixelProtocol>();
    auto transport = std::make_shared<MockSerialHandler>();
    TestableDynamixelLink link(transport, protocol);

    std::array<uint8_t, RXPACKET_MAX_LEN> rxbuffer;

    // dois primeiros bytes
    // e último byte
    // são lixo
    std::array<uint8_t, 11> full_rx = {
        0x23, 0x04, 0xFF, 0xFF, 0x01, 0x04, 
        0x02, 0x03, 0x00, 0xF5, 0x17
    };
    size_t offset = 0;

    EXPECT_CALL(*transport, readData(::testing::_, ::testing::_))
        .WillRepeatedly(::testing::Invoke(
            [&](uint8_t* buffer, size_t size) -> ssize_t {
                size_t remaining = full_rx.size() - offset;
                if (remaining == 0) return 0;
                
                // pega no máximo 3 bytes
                size_t to_copy = std::min<size_t>({size, remaining, 3});
                std::copy(full_rx.begin() + offset, full_rx.begin() + offset + to_copy, buffer);
                offset += to_copy;

                return to_copy;
            }
        ));

    int result = link.readPacket(rxbuffer);

    std::array<uint8_t, 8> expected_rx = {
        0xFF, 0xFF, 0x01, 0x04, 0x02, 0x03, 0x00, 0xF5
    };
    EXPECT_TRUE(std::equal(rxbuffer.begin(), rxbuffer.begin() + expected_rx.size(), expected_rx.begin()));
    EXPECT_EQ(result, 0);
}

TEST(actuator_driver, dynamixel_read_packet_content_timeout)
{
    auto protocol = std::make_shared<DynamixelProtocol>();
    auto transport = std::make_shared<MockSerialHandler>();
    TestableDynamixelLink link(transport, protocol);

    std::array<uint8_t, RXPACKET_MAX_LEN> rxbuffer;

    EXPECT_CALL(*transport, readData(::testing::_, ::testing::_))
        .WillRepeatedly(::testing::Return(0));

    int result = link.readPacket(rxbuffer);

    EXPECT_EQ(result, -2);
}