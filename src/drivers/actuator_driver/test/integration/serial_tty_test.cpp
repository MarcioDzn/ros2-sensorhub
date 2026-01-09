#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <pty.h>
#include <unistd.h>

#include "dynamixel/dynamixel_link.hpp"
#include "dynamixel/dynamixel_protocol.hpp"


TEST(actuator_driver, integration_serial_write_1_byte_packet_success)
{
    int master_fd, slave_fd;
    char slave_name[100];

    ASSERT_EQ(0, openpty(&master_fd, &slave_fd, slave_name, nullptr, nullptr));

    auto protocol = std::make_shared<DynamixelProtocol>();
    auto transport = std::make_shared<SerialHandler>();

    ASSERT_EQ(transport->init(slave_name), 0);
    transport->setDefaultConfig();
    transport->setBaudRate(2000000);

    auto link = std::make_shared<DynamixelLink>(transport, protocol);

    ASSERT_EQ(link->write1Byte(1, 0x18, 1), 0);

    uint8_t buf[32];
    ssize_t n = read(master_fd, buf, sizeof(buf));

    std::vector<uint8_t> packet = {0xFF, 0xFF, 0x01, 0x04, 0x03, 0x18, 0x01, 0xDE};
    EXPECT_EQ(n, packet.size());
    EXPECT_TRUE(std::equal(buf, buf + n, packet.begin()));

    close(master_fd);
    close(slave_fd);
}

TEST(actuator_driver, integration_serial_write_1_byte_packet_checksum_error)
{
    int master_fd, slave_fd;
    char slave_name[100];

    ASSERT_EQ(0, openpty(&master_fd, &slave_fd, slave_name, nullptr, nullptr));

    auto protocol = std::make_shared<DynamixelProtocol>();
    auto transport = std::make_shared<SerialHandler>();

    ASSERT_EQ(transport->init(slave_name), 0);
    transport->setDefaultConfig();
    transport->setBaudRate(2000000);

    auto link = std::make_shared<DynamixelLink>(transport, protocol);

    ASSERT_EQ(link->write1Byte(1, 0x18, 1), 0);

    uint8_t buf[32];
    ssize_t n = read(master_fd, buf, sizeof(buf));

    std::vector<uint8_t> packet = {0xFF, 0xFF, 0x01, 0x04, 0x03, 0x18, 0x01, 0xD2};
    EXPECT_EQ(n, packet.size());
    EXPECT_FALSE(std::equal(buf, buf + n, packet.begin()));

    close(master_fd);
    close(slave_fd);
}

TEST(actuator_driver, integration_serial_write_2_byte_packet_success)
{
    int master_fd, slave_fd;
    char slave_name[100];

    ASSERT_EQ(0, openpty(&master_fd, &slave_fd, slave_name, nullptr, nullptr));

    auto protocol = std::make_shared<DynamixelProtocol>();
    auto transport = std::make_shared<SerialHandler>();

    ASSERT_EQ(transport->init(slave_name), 0);
    transport->setDefaultConfig();
    transport->setBaudRate(2000000);

    auto link = std::make_shared<DynamixelLink>(transport, protocol);

    ASSERT_EQ(link->write2Byte(1, 0x18, 3000), 0);

    uint8_t buf[32];
    ssize_t n = read(master_fd, buf, sizeof(buf));

    std::vector<uint8_t> packet = {0xFF, 0xFF, 0x01, 0x05, 0x03, 0x18, 0xB8, 0x0B, 0x1B};
    EXPECT_EQ(n, packet.size());
    EXPECT_TRUE(std::equal(buf, buf + n, packet.begin()));

    close(master_fd);
    close(slave_fd);
}

TEST(actuator_driver, integration_serial_write_2_byte_packet_checksum_error)
{
    int master_fd, slave_fd;
    char slave_name[100];

    ASSERT_EQ(0, openpty(&master_fd, &slave_fd, slave_name, nullptr, nullptr));

    auto protocol = std::make_shared<DynamixelProtocol>();
    auto transport = std::make_shared<SerialHandler>();

    ASSERT_EQ(transport->init(slave_name), 0);
    transport->setDefaultConfig();
    transport->setBaudRate(2000000);

    auto link = std::make_shared<DynamixelLink>(transport, protocol);

    ASSERT_EQ(link->write2Byte(1, 0x18, 3000), 0);

    uint8_t buf[32];
    ssize_t n = read(master_fd, buf, sizeof(buf));

    std::vector<uint8_t> packet = {0xFF, 0xFF, 0x01, 0x05, 0x03, 0x18, 0xB8, 0x0B, 0x1C};
    EXPECT_EQ(n, packet.size());
    EXPECT_FALSE(std::equal(buf, buf + n, packet.begin()));

    close(master_fd);
    close(slave_fd);
}