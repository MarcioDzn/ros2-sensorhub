#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "dynamixel/dynamixel_link.hpp"
#include "dynamixel/dynamixel_protocol.hpp"


class TestableDynamixelLink : public DynamixelLink {
public:
    using DynamixelLink::DynamixelLink; 
    using DynamixelLink::readPacket;     
    using DynamixelLink::readStatus;
    using DynamixelLink::sendPacket;
    using DynamixelLink::sendPacketAndReadStatus;
};

class MockDynamixelLink : public DynamixelLink {
public:
    using DynamixelLink::DynamixelLink; // Herda construtores

    MOCK_METHOD(int, readPacket, 
        ((std::array<uint8_t, RXPACKET_MAX_LEN>&)), (override));
    MOCK_METHOD(int, readStatus, 
        (uint8_t, StatusPacket&), (override));
    MOCK_METHOD(int, sendPacket, 
        (const std::vector<uint8_t>&), (override));

    // métodos são públicos q permitem que o 
    // Google Mock acesse a lógica protegida da pai
    int callRealReadPacket(std::array<uint8_t, RXPACKET_MAX_LEN>& b) {
        return DynamixelLink::readPacket(b);
    }
    int callRealReadStatus(uint8_t id, StatusPacket& s) {
        return DynamixelLink::readStatus(id, s);
    }
    int callRealSendPacket(const std::vector<uint8_t>& p) {
        return DynamixelLink::sendPacket(p);
    }
};

class MockSerialHandler : public SerialHandler
{
public:
    MOCK_METHOD(ssize_t, writeData,
        (const uint8_t*, size_t), (override));
    MOCK_METHOD(ssize_t, readData,
        (uint8_t*, size_t), (override));
};

TEST(actuator_driver, dynamixel_read_packet_success)
{
    auto protocol = std::make_shared<DynamixelProtocol>();
    auto transport = std::make_shared<MockSerialHandler>();

    testing::NiceMock<MockDynamixelLink> link(transport, protocol);

    // usa a lógica real de readPacket
    ON_CALL(link, readPacket(::testing::_))
        .WillByDefault(testing::Invoke(&link, &MockDynamixelLink::callRealReadPacket));

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
    testing::NiceMock<MockDynamixelLink> link(transport, protocol);

    ON_CALL(link, readPacket(::testing::_))
        .WillByDefault(testing::Invoke(&link, &MockDynamixelLink::callRealReadPacket));

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
    testing::NiceMock<MockDynamixelLink> link(transport, protocol);

    ON_CALL(link, readPacket(::testing::_))
        .WillByDefault(testing::Invoke(&link, &MockDynamixelLink::callRealReadPacket));

    std::array<uint8_t, RXPACKET_MAX_LEN> rxbuffer;

    std::array<uint8_t, 8> full_rx = {
        0xFF, 0xFF, 0x01, 0x04, 0x00, 0x03, 0x00, 0xF7
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
        0xFF, 0xFF, 0x01, 0x04, 0x00, 0x03, 0x00, 0xF7
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
    testing::NiceMock<MockDynamixelLink> link(transport, protocol);

    ON_CALL(link, readPacket(::testing::_))
        .WillByDefault(testing::Invoke(&link, &MockDynamixelLink::callRealReadPacket));

    std::array<uint8_t, RXPACKET_MAX_LEN> rxbuffer;

    // dois primeiros bytes
    // e último byte
    // são lixo
    std::array<uint8_t, 11> full_rx = {
        0x23, 0x04, 0xFF, 0xFF, 0x01, 0x04, 
        0x00, 0x03, 0x00, 0xF7, 0x17
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
        0xFF, 0xFF, 0x01, 0x04, 0x00, 0x03, 0x00, 0xF7
    };
    EXPECT_TRUE(std::equal(rxbuffer.begin(), rxbuffer.begin() + expected_rx.size(), expected_rx.begin()));
    EXPECT_EQ(result, 0);
}

TEST(actuator_driver, dynamixel_read_packet_content_timeout)
{
    auto protocol = std::make_shared<DynamixelProtocol>();
    auto transport = std::make_shared<MockSerialHandler>();
    testing::NiceMock<MockDynamixelLink> link(transport, protocol);

    ON_CALL(link, readPacket(::testing::_))
        .WillByDefault(testing::Invoke(&link, &MockDynamixelLink::callRealReadPacket));

    std::array<uint8_t, RXPACKET_MAX_LEN> rxbuffer;

    EXPECT_CALL(*transport, readData(::testing::_, ::testing::_))
        .WillRepeatedly(::testing::Return(0));

    int result = link.readPacket(rxbuffer);

    EXPECT_EQ(result, -2);
}

TEST(actuator_driver, dynamixel_read_status_success)
{
    auto protocol = std::make_shared<DynamixelProtocol>();
    auto transport = std::make_shared<MockSerialHandler>();
    testing::NiceMock<MockDynamixelLink> link(transport, protocol);

    ON_CALL(link, readStatus(::testing::_, ::testing::_))
        .WillByDefault(testing::Invoke(&link, &MockDynamixelLink::callRealReadStatus));

    std::array<uint8_t, RXPACKET_MAX_LEN> rxbuffer;
    EXPECT_CALL(link, readPacket(::testing::_))
        .WillOnce(::testing::Invoke(
            [](std::array<uint8_t, RXPACKET_MAX_LEN>& packet) -> int {
                std::array<uint8_t, 6> rx = {0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC};
                std::copy(rx.begin(), rx.end(), packet.begin());
                return 0;
            }
        ));
    
    StatusPacket status;
    int result = link.readStatus(1, status);

    EXPECT_EQ(result, 0);
}

TEST(actuator_driver, dynamixel_read_status_read_packet_error)
{
    auto protocol = std::make_shared<DynamixelProtocol>();
    auto transport = std::make_shared<MockSerialHandler>();
    testing::NiceMock<MockDynamixelLink> link(transport, protocol);

    ON_CALL(link, readStatus(::testing::_, ::testing::_))
        .WillByDefault(testing::Invoke(&link, &MockDynamixelLink::callRealReadStatus));

    std::array<uint8_t, RXPACKET_MAX_LEN> rxbuffer;
    EXPECT_CALL(link, readPacket(::testing::_))
        .WillOnce(::testing::Return(-1));
    
    StatusPacket status;
    int result = link.readStatus(1, status);

    EXPECT_EQ(result, -1);
}


TEST(actuator_driver, dynamixel_read_status_id_error)
{
    auto protocol = std::make_shared<DynamixelProtocol>();
    auto transport = std::make_shared<MockSerialHandler>();
    testing::NiceMock<MockDynamixelLink> link(transport, protocol);

    ON_CALL(link, readStatus(::testing::_, ::testing::_))
        .WillByDefault(testing::Invoke(&link, &MockDynamixelLink::callRealReadStatus));

    std::array<uint8_t, RXPACKET_MAX_LEN> rxbuffer;
    EXPECT_CALL(link, readPacket(::testing::_))
        .WillOnce(::testing::Invoke(
            [](std::array<uint8_t, RXPACKET_MAX_LEN>& packet) -> int {
                std::array<uint8_t, 6> rx = {0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC};
                std::copy(rx.begin(), rx.end(), packet.begin());
                return 0;
            }
        ));
    
    StatusPacket status;
    int result = link.readStatus(2, status); // id errado de propósito

    EXPECT_EQ(result, -1);
}

TEST(actuator_driver, dynamixel_read_status_content)
{
    auto protocol = std::make_shared<DynamixelProtocol>();
    auto transport = std::make_shared<MockSerialHandler>();
    testing::NiceMock<MockDynamixelLink> link(transport, protocol);

    ON_CALL(link, readStatus(::testing::_, ::testing::_))
        .WillByDefault(testing::Invoke(&link, &MockDynamixelLink::callRealReadStatus));

    std::array<uint8_t, RXPACKET_MAX_LEN> rxbuffer;
    EXPECT_CALL(link, readPacket(::testing::_))
        .WillOnce(::testing::Invoke(
            [](std::array<uint8_t, RXPACKET_MAX_LEN>& packet) -> int {
                std::array<uint8_t, 8> rx = {0xFF, 0xFF, 0x01, 0x04, 0x00, 0x03, 0x00, 0xF7};
                std::copy(rx.begin(), rx.end(), packet.begin());
                return 0;
            }
        ));

    StatusPacket status;
    int result = link.readStatus(1, status);

    EXPECT_EQ(status.error, 0x00);
    EXPECT_EQ(status.id, 0x01);
    EXPECT_EQ(status.params[0], 0x03);
    EXPECT_EQ(status.params[1], 0x00);
}

TEST(actuator_driver, dynamixel_read_status_content_status_not_null_error)
{
    auto protocol = std::make_shared<DynamixelProtocol>();
    auto transport = std::make_shared<MockSerialHandler>();
    testing::NiceMock<MockDynamixelLink> link(transport, protocol);

    ON_CALL(link, readStatus(::testing::_, ::testing::_))
        .WillByDefault(testing::Invoke(&link, &MockDynamixelLink::callRealReadStatus));

    // pacote lido de maneira certa
    // mas com byte de erro não nulo
    std::array<uint8_t, RXPACKET_MAX_LEN> rxbuffer;
    EXPECT_CALL(link, readPacket(::testing::_))
        .WillOnce(::testing::Invoke(
            [](std::array<uint8_t, RXPACKET_MAX_LEN>& packet) -> int {
                std::array<uint8_t, 8> rx = {0xFF, 0xFF, 0x01, 0x04, 0x01, 0x03, 0x00, 0xF6};
                std::copy(rx.begin(), rx.end(), packet.begin());
                return 0;
            }
        ));

    StatusPacket status;
    int result = link.readStatus(1, status);

    EXPECT_EQ(result, -1);
    EXPECT_EQ(status.error, 0x01);
    EXPECT_EQ(status.id, 0x01);
}

TEST(actuator_driver, dynamixel_send_packet_success)
{
    auto protocol = std::make_shared<DynamixelProtocol>();
    auto transport = std::make_shared<MockSerialHandler>();
    TestableDynamixelLink link(transport, protocol);

    EXPECT_CALL(*transport, writeData(::testing::_, ::testing::_))
        .WillOnce(::testing::Invoke(
            [](const uint8_t* buffer, size_t size) -> ssize_t {
                return 0;
            }
        ));

    std::vector<uint8_t> packet = {0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC};
    int result = link.sendPacket(packet);

    EXPECT_EQ(result, 0);
}

TEST(actuator_driver, dynamixel_send_packet_error)
{
    auto protocol = std::make_shared<DynamixelProtocol>();
    auto transport = std::make_shared<MockSerialHandler>();
    TestableDynamixelLink link(transport, protocol);

    EXPECT_CALL(*transport, writeData(::testing::_, ::testing::_))
        .WillOnce(::testing::Invoke(
            [](const uint8_t* buffer, size_t size) -> ssize_t {
                return -1;
            }
        ));

    std::vector<uint8_t> packet = {0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC};
    int result = link.sendPacket(packet);

    EXPECT_EQ(result, -1);
}