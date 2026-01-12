#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <rclcpp/rclcpp.hpp>

#include <pty.h>
#include <unistd.h>
#include <fcntl.h>

#include "actuator_node.hpp"

#include "interfaces/msg/state.hpp"
#include "interfaces/msg/command.hpp"
#include "interfaces/srv/set_torque.hpp"

using namespace std::chrono_literals;

class ActuatorNodeFixture : public ::testing::Test {
    protected:
        int master_fd;
        int slave_fd;
        char slave_name[100];
        std::shared_ptr<ActuatorNode> node;

    void SetUp() override {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }

        ASSERT_EQ(0, openpty(&master_fd, &slave_fd, slave_name, nullptr, nullptr));

        rclcpp::NodeOptions options;
        options.append_parameter_override("base_name", "dxl");
        options.append_parameter_override("update_rate_ms", 15);
        options.append_parameter_override("usb_port", slave_name);
        options.append_parameter_override("baudrate", 2000000);
        options.append_parameter_override("actuator_ids", std::vector<int64_t>{1});

        node = std::make_shared<ActuatorNode>(options);
    }

    void TearDown() {
        node.reset();
        close(master_fd);
        close(slave_fd);
    }

    bool waitFor(std::function<bool()> condition, std::chrono::milliseconds timeout = 100ms) {
        auto start = std::chrono::steady_clock::now();
        while ((std::chrono::steady_clock::now() - start) < timeout) {
            if (condition()) return true;
            rclcpp::spin_some(node);
            std::this_thread::sleep_for(10ms);
        }
        return false;
    }
};

TEST_F(ActuatorNodeFixture, publishes_data_success) {
    bool received = false;

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
        .best_effort()
        .durability_volatile();
    auto sub = node->create_subscription<interfaces::msg::State>(
        "dxl/state",
        qos,
        [&received](const interfaces::msg::State::SharedPtr msg) {
            received = true;
        }
    );

    // mock do status packet
    uint8_t dummy_response[] = {0xFF, 0xFF, 0x01, 0x04, 0x00, 0x00, 0x00, 0xFA};
    write(master_fd, dummy_response, sizeof(dummy_response));

    // necessário esperar pra dar tempo de publicar os dados
    bool success = waitFor([&received]() { return received; }, std::chrono::milliseconds(100));

    ASSERT_TRUE(received);
    rclcpp::shutdown();
}

TEST_F(ActuatorNodeFixture, publishes_data_checksum_error) {
    bool received = false;

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
        .best_effort()
        .durability_volatile();
    auto sub = node->create_subscription<interfaces::msg::State>(
        "dxl/state",
        qos,
        [&received](const interfaces::msg::State::SharedPtr msg) {
            received = true;
        }
    );

    // mock do status packet (com checksum errado)
    uint8_t dummy_response[] = {0xFF, 0xFF, 0x01, 0x04, 0x00, 0x00, 0x00, 0xF3};
    write(master_fd, dummy_response, sizeof(dummy_response));

    // necessário esperar pra dar tempo de publicar os dados
    bool success = waitFor([&received]() { return received; }, std::chrono::milliseconds(100));

    ASSERT_FALSE(received);
    rclcpp::shutdown();
}

TEST_F(ActuatorNodeFixture, publishes_data_content) {
    bool received = false;
    uint8_t id;
    uint16_t curr_pos;

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
        .best_effort()
        .durability_volatile();
    auto sub = node->create_subscription<interfaces::msg::State>(
        "dxl/state",
        qos,
        [&received, &id, &curr_pos](const interfaces::msg::State::SharedPtr msg) {
            received = true;
            id = msg->ids[0];
            curr_pos = msg->positions[0];
        }
    );
    
    // mock do status packet (posicao 512)
    uint8_t dummy_response[] = {0xFF, 0xFF, 0x01, 0x04, 0x00, 0x00, 0x02, 0xF8};
    write(master_fd, dummy_response, sizeof(dummy_response));

    // necessário esperar pra dar tempo de publicar os dados
    bool success = waitFor([&received]() { return received; }, std::chrono::milliseconds(100));

    ASSERT_TRUE(received);
    EXPECT_EQ(id, 1);
    EXPECT_EQ(curr_pos, 512);

    rclcpp::shutdown();
}

TEST_F(ActuatorNodeFixture, subscribes_data_success) {
    uint8_t id;
    uint16_t curr_pos;
    bool data_sent = false;
    uint8_t read_buffer[100];
    ssize_t bytes_read = 0;

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
        .best_effort()
        .durability_volatile();
    auto pub = node->create_publisher<interfaces::msg::Command>(
        "dxl/command",
        qos);

    auto msg = interfaces::msg::Command();
    msg.ids = {1};
    msg.goals = {3000};
    pub->publish(msg);
    
    auto start_time = std::chrono::steady_clock::now();

    while ((std::chrono::steady_clock::now() - start_time) < std::chrono::milliseconds(500)) {
        rclcpp::spin_some(node);
        bytes_read = read(master_fd, read_buffer, sizeof(read_buffer));
        if (bytes_read > 0) {
            data_sent = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    uint8_t expected_packet[] = {0xFF, 0xFF, 0x01, 0x05, 0x03, 0x1E, 0xB8, 0x0B, 0x15};

    ASSERT_EQ(bytes_read, sizeof(expected_packet));
    for (size_t i = 0; i < bytes_read; i++) {
        EXPECT_EQ(read_buffer[i], expected_packet[i]) << "Erro no byte " << i;
    }

    rclcpp::shutdown();
}

TEST_F(ActuatorNodeFixture, service_command_success_set_torque_enable_success) {
    bool data_sent = false;
    uint8_t read_buffer[100];
    ssize_t bytes_read = 0;

    // limpa o buffer
    fcntl(master_fd, F_SETFL, O_NONBLOCK);
    while (read(master_fd, read_buffer, sizeof(read_buffer)) > 0);

    auto client = node->create_client<interfaces::srv::SetTorque>(
        "dxl/set_torque");
    // precisa esperar o service ficar on
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(1)));

    auto request = std::make_shared<interfaces::srv::SetTorque::Request>();
    request->id = 1;
    request->status = true;
    auto result_future = client->async_send_request(request);
    
    // roda até receber a resposta
    auto status = rclcpp::spin_until_future_complete(node, result_future, std::chrono::milliseconds(500));

    ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);
    auto response = result_future.get();
    EXPECT_TRUE(response->success);

    bytes_read = read(master_fd, read_buffer, sizeof(read_buffer));
    uint8_t expected_packet[] = {0xFF, 0xFF, 0x01, 0x04, 0x03, 0x18, 0x01, 0xDE};
    ASSERT_EQ(bytes_read, sizeof(expected_packet));
    for (size_t i = 0; i < bytes_read; i++) {
        EXPECT_EQ(read_buffer[i], expected_packet[i]) << "Erro no byte " << i;
    }

    rclcpp::shutdown();
}