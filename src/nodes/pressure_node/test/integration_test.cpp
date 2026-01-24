#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <rclcpp/rclcpp.hpp>

#include <pty.h>
#include <unistd.h>
#include <fcntl.h>

#include "pressure_node.hpp"

#include "interfaces/msg/pressure_state.hpp"

using namespace std::chrono_literals;

class PressureNodeFixture : public ::testing::Test {
    protected:
        int master_fds[2];
        int slave_fds[2];
        char slave_names[2][100];
        std::shared_ptr<PressureNode> node;

    void SetUp() override {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }

        // sola 1
        ASSERT_EQ(0, openpty(&master_fds[0], &slave_fds[0], slave_names[0], nullptr, nullptr));
        close(slave_fds[0]);

        // sola 2
        ASSERT_EQ(0, openpty(&master_fds[1], &slave_fds[1], slave_names[1], nullptr, nullptr));
        close(slave_fds[1]);

        rclcpp::NodeOptions options;
        options.append_parameter_override("base_name", "pressure");
        options.append_parameter_override("update_rate_ms", 15);
        options.append_parameter_override("usb_ports", std::vector<std::string>{slave_names[0], slave_names[1]});
        options.append_parameter_override("baudrate", 115200);
        options.append_parameter_override("ids", std::vector<int64_t>{1, 2});

        node = std::make_shared<PressureNode>(options);
    }

    void TearDown() {
        node.reset();

        // sola 1
        close(master_fds[0]);
        close(slave_fds[0]);

        // sola 2
        close(master_fds[1]);
        close(slave_fds[1]);
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

TEST_F(PressureNodeFixture, publishes_data_success) {
    bool received = false;

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
        .best_effort()
        .durability_volatile();

    auto sub = node->create_subscription<interfaces::msg::PressureState>(
        "pressure/state",
        qos,
        [&received](const interfaces::msg::PressureState::SharedPtr) {
            received = true;
        }
    );

    // Executor para processar callbacks
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    // Mock do pacote vindo da serial
    std::string dummy_response =
        "1234 1234 1234 1234 1234 1234 1234 1234 "
        "1234 1234 1234 1234 1234 1234 1234 1234";

    dummy_response.push_back('\0');

    write(master_fds[0], dummy_response.c_str(), dummy_response.size());

    auto timeout = std::chrono::milliseconds(500);
    auto start = std::chrono::steady_clock::now();

    while (!received && (std::chrono::steady_clock::now() - start < timeout)) {
        executor.spin_some();
        std::this_thread::sleep_for(10ms);
    }

    ASSERT_TRUE(received);
}

TEST_F(PressureNodeFixture, publishes_data_content_success) {
    bool received = false;
    int8_t received_id = 0;
    interfaces::msg::PressureData received_pressure_data;

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
        .best_effort()
        .durability_volatile();

    auto sub = node->create_subscription<interfaces::msg::PressureState>(
        "pressure/state",
        qos,
        [&received, &received_id, &received_pressure_data](const interfaces::msg::PressureState::SharedPtr msg) {
            received = true;
            received_id = msg->ids[0];
            received_pressure_data = msg->pressures[0];
        }
    );

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    std::string dummy_response =
        "123  1234 1234 1234 1234 1234 1234 1234 "
        "1234 1234 1234 1234 1234 1234 1234 1234";
    dummy_response.push_back('\0');

    write(master_fds[0], dummy_response.c_str(), dummy_response.size());

    auto timeout = std::chrono::milliseconds(500);
    auto start = std::chrono::steady_clock::now();

    while (!received && (std::chrono::steady_clock::now() - start < timeout)) {
        executor.spin_some();
        std::this_thread::sleep_for(10ms);
    }

    ASSERT_TRUE(received);
    EXPECT_EQ(received_id, 1);

    ASSERT_FALSE(received_pressure_data.pressures.empty());
    EXPECT_EQ(received_pressure_data.pressures[0], 123);
}

TEST_F(PressureNodeFixture, publishes_data__multisensor_content_success) {
    bool received = false;
    std::vector<int8_t> received_ids;
    std::vector<interfaces::msg::PressureData> received_pressures_data;

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
        .best_effort()
        .durability_volatile();

    auto sub = node->create_subscription<interfaces::msg::PressureState>(
        "pressure/state",
        qos,
        [&received, &received_ids, &received_pressures_data](const interfaces::msg::PressureState::SharedPtr msg) {
            received = true;
            received_ids = msg->ids;
            received_pressures_data = msg->pressures;
        }
    );

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    std::string dummy_response =
        "123  1234 1234 1234 1234 1234 1234 1234 "
        "1234 1234 1234 1234 1234 1234 1234 1234";
    dummy_response.push_back('\0');

    // mesmos valores pra as duas solas
    write(master_fds[0], dummy_response.c_str(), dummy_response.size());
    write(master_fds[1], dummy_response.c_str(), dummy_response.size());

    auto timeout = std::chrono::milliseconds(500);
    auto start = std::chrono::steady_clock::now();

    while (!received && (std::chrono::steady_clock::now() - start < timeout)) {
        executor.spin_some();
        std::this_thread::sleep_for(10ms);
    }

    ASSERT_TRUE(received);
    ASSERT_EQ(received_ids.size(), 2);
    EXPECT_EQ(received_ids[0], 1);
    EXPECT_EQ(received_ids[1], 2);

    // sola 1
    ASSERT_FALSE(received_pressures_data[0].pressures.empty());
    EXPECT_EQ(received_pressures_data[0].pressures[0], 123);

    // sola 2
    ASSERT_FALSE(received_pressures_data[1].pressures.empty());
    EXPECT_EQ(received_pressures_data[1].pressures[0], 123);
}

TEST_F(PressureNodeFixture, DISABLED_publishes_junk_data) {
    bool received = false;

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
        .best_effort()
        .durability_volatile();

    auto sub = node->create_subscription<interfaces::msg::PressureState>(
        "pressure/state",
        qos,
        [&received](const interfaces::msg::PressureState::SharedPtr msg) {
            received = true;
        }
    );

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    std::string dummy_response =
        "abcd abr  ###  123  1234 1234 1234 1234 1234 1234 1234 "
        "1234 1234 1234 1234 1234 1234 1234 1234";
    dummy_response.push_back('\0');

    write(master_fds[0], dummy_response.c_str(), dummy_response.size());

    auto timeout = std::chrono::milliseconds(500);
    auto start = std::chrono::steady_clock::now();

    while (!received && (std::chrono::steady_clock::now() - start < timeout)) {
        executor.spin_some();
        std::this_thread::sleep_for(10ms);
    }

    ASSERT_FALSE(received) << "No publicou dados que deveriam estar corrompidos";
}