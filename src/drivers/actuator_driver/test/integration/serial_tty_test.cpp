#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <pty.h>
#include <unistd.h>
#include <fcntl.h>

#include "dynamixel/dynamixel_controller.hpp"


TEST(actuator_driver, integration_serial_write_packet_enable_torque)
{
    int master_fd, slave_fd;
    char slave_name[100];

    ASSERT_EQ(0, openpty(&master_fd, &slave_fd, slave_name, nullptr, nullptr));

    auto controller = std::make_shared<DynamixelController>();

    ASSERT_EQ(controller->init(slave_name, 2000000), 0);

    ASSERT_EQ(controller->setTorque(1, 1), 0);

    uint8_t buf[32];
    ssize_t n = read(master_fd, buf, sizeof(buf));

    std::vector<uint8_t> expected_packet = {0xFF, 0xFF, 0x01, 0x04, 0x03, 0x18, 0x01, 0xDE};
    ASSERT_EQ(n, expected_packet.size());
    EXPECT_TRUE(std::equal(buf, buf + n, expected_packet.begin()));

    close(master_fd);
    close(slave_fd);
}

TEST(actuator_driver, integration_serial_write_packet_disable_torque)
{
    int master_fd, slave_fd;
    char slave_name[100];

    ASSERT_EQ(0, openpty(&master_fd, &slave_fd, slave_name, nullptr, nullptr));

    auto controller = std::make_shared<DynamixelController>();

    ASSERT_EQ(controller->init(slave_name, 2000000), 0);

    ASSERT_EQ(controller->setTorque(1, 0), 0);

    uint8_t buf[32];
    ssize_t n = read(master_fd, buf, sizeof(buf));

    std::vector<uint8_t> expected_packet = {0xFF, 0xFF, 0x01, 0x04, 0x03, 0x18, 0x00, 0xDF};
    ASSERT_EQ(n, expected_packet.size());
    EXPECT_TRUE(std::equal(buf, buf + n, expected_packet.begin()));

    close(master_fd);
    close(slave_fd);
}

TEST(actuator_driver, integration_serial_write_packet_set_goal_position)
{
    int master_fd, slave_fd;
    char slave_name[100];

    ASSERT_EQ(0, openpty(&master_fd, &slave_fd, slave_name, nullptr, nullptr));

    auto controller = std::make_shared<DynamixelController>();

    ASSERT_EQ(controller->init(slave_name, 2000000), 0);

    ASSERT_EQ(controller->setGoalPosition(1, 3000), 0);

    uint8_t buf[32];
    ssize_t n = read(master_fd, buf, sizeof(buf));

    std::vector<uint8_t> expected_packet = {0xFF, 0xFF, 0x01, 0x05, 0x03, 0x1E, 0xB8, 0x0B, 0x15};
    ASSERT_EQ(n, expected_packet.size());
    EXPECT_TRUE(std::equal(buf, buf + n, expected_packet.begin()));

    close(master_fd);
    close(slave_fd);
}

TEST(actuator_driver, integration_serial_write_packet_get_current_position_success)
{
    int master_fd, slave_fd;
    char slave_name[100];

    ASSERT_EQ(0, openpty(&master_fd, &slave_fd, slave_name, nullptr, nullptr));

    auto controller = std::make_shared<DynamixelController>();

    ASSERT_EQ(controller->init(slave_name, 2000000), 0);

    // simula o atuador mandando o status
    std::vector<uint8_t> status_packet = {0xFF, 0xFF, 0x01, 0x04, 0x00, 0xB8, 0x0B, 0x37};
    ssize_t written = write(master_fd, status_packet.data(), status_packet.size());
    ASSERT_EQ(written, status_packet.size());

    uint16_t curr_pos;
    ASSERT_EQ(controller->getCurrentPosition(1, curr_pos), 0);

    // verifica se o comando de leitura foi enviado corretamente
    uint8_t buf[32];
    ssize_t n = read(master_fd, buf, sizeof(buf));
    ASSERT_GE(n, 8) << "Deve ler pelo menos 8 bytes";

    std::vector<uint8_t> to_send_packet = {0xFF, 0xFF, 0x01, 0x04, 0x02, 0x24, 0x02, 0xD2};
    EXPECT_TRUE(std::equal(to_send_packet.begin(), to_send_packet.end(), buf));

    // verifica se buscou a posição correta
    uint16_t expected_curr_pos = 3000;
    ASSERT_EQ(curr_pos, expected_curr_pos);

    close(master_fd);
    close(slave_fd);
}

TEST(actuator_driver, integration_serial_write_packet_get_current_position_status_error)
{
    int master_fd, slave_fd;
    char slave_name[100];

    ASSERT_EQ(0, openpty(&master_fd, &slave_fd, slave_name, nullptr, nullptr));

    auto controller = std::make_shared<DynamixelController>();

    ASSERT_EQ(controller->init(slave_name, 2000000), 0);

    // simula o atuador mandando o status
    std::vector<uint8_t> status_packet = {0xFF, 0xFF, 0x01, 0x04, 0x00, 0xB7, 0x0B, 0x38};
    ssize_t written = write(master_fd, status_packet.data(), status_packet.size());
    ASSERT_EQ(written, status_packet.size());

    uint16_t curr_pos;
    ASSERT_EQ(controller->getCurrentPosition(1, curr_pos), 0);

    // verifica se o comando de leitura foi enviado corretamente
    uint8_t buf[32];
    ssize_t n = read(master_fd, buf, sizeof(buf));
    ASSERT_GE(n, 8) << "Deve ler pelo menos 8 bytes";

    std::vector<uint8_t> to_send_packet = {0xFF, 0xFF, 0x01, 0x04, 0x02, 0x24, 0x02, 0xD2};
    EXPECT_TRUE(std::equal(to_send_packet.begin(), to_send_packet.end(), buf));

    // verifica se buscou a posição correta
    uint16_t expected_curr_pos = 3000;
    ASSERT_NE(curr_pos, expected_curr_pos);

    close(master_fd);
    close(slave_fd);
}

TEST(actuator_driver, integration_serial_write_packet_get_current_position_command_id_error)
{
    int master_fd, slave_fd;
    char slave_name[100];

    ASSERT_EQ(0, openpty(&master_fd, &slave_fd, slave_name, nullptr, nullptr));

    auto controller = std::make_shared<DynamixelController>();

    ASSERT_EQ(controller->init(slave_name, 2000000), 0);

    // simula o atuador mandando o status
    std::vector<uint8_t> status_packet = {0xFF, 0xFF, 0x01, 0x04, 0x00, 0xB8, 0x0B, 0x37};
    ssize_t written = write(master_fd, status_packet.data(), status_packet.size());
    ASSERT_EQ(written, status_packet.size());

    uint16_t curr_pos;
    ASSERT_EQ(controller->getCurrentPosition(1, curr_pos), 0);

    // verifica se o comando de leitura foi enviado corretamente
    uint8_t buf[32];
    ssize_t n = read(master_fd, buf, sizeof(buf));
    ASSERT_GE(n, 8) << "Deve ler pelo menos 8 bytes";

    std::vector<uint8_t> to_send_packet = {0xFF, 0xFF, 0x02, 0x04, 0x02, 0x24, 0x02, 0xD1};
    EXPECT_FALSE(std::equal(to_send_packet.begin(), to_send_packet.end(), buf));

    // verifica se buscou a posição correta
    uint16_t expected_curr_pos = 3000;
    ASSERT_EQ(curr_pos, expected_curr_pos);

    close(master_fd);
    close(slave_fd);
}

TEST(actuator_driver, integration_serial_read_timeout)
{
    int master_fd, slave_fd;
    char slave_name[100];

    ASSERT_EQ(0, openpty(&master_fd, &slave_fd, slave_name, nullptr, nullptr));

    auto controller = std::make_shared<DynamixelController>();

    ASSERT_EQ(controller->init(slave_name, 2000000), 0);

    // não envia nenhuma resposta do atuador
    uint16_t curr_pos;
    
    // Deve retornar erro por timeout
    ASSERT_EQ(controller->getCurrentPosition(1, curr_pos), -1) 
        << "Deveria retornar erro por timeout";

    close(master_fd);
    close(slave_fd);
}