#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <rclcpp/rclcpp.hpp>

#include <pty.h>
#include <unistd.h>
#include <fcntl.h>
#include <chrono>

#include "imu_node.hpp"

#include "interfaces/msg/imu_state.hpp"

using namespace std::chrono_literals;

class IMUFixture : public ::testing::Test {
    protected:
        int master_fd;
        int slave_fd;
        char slave_name[100];
        std::shared_ptr<IMUNode> node;

    void SetUp() override {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }

        ASSERT_EQ(0, openpty(&master_fd, &slave_fd, slave_name, nullptr, nullptr));

        rclcpp::NodeOptions options;
        options.append_parameter_override("base_name", "imu");
        options.append_parameter_override("update_rate_ms", 15);
        options.append_parameter_override("ids", std::vector<int64_t>{1, 2, 3});
        options.append_parameter_override("multiplexer", std::vector<int64_t>{0, 1, 0});
        options.append_parameter_override("addresses", std::vector<int64_t>{40, 41, 40});

        node = std::make_shared<IMUNode>(options);
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