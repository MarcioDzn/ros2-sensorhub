#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "sync_node.hpp"
#include "interfaces/msg/synced_sensor_data.hpp"

using namespace std::chrono_literals;

class SyncNodeFixture : public ::testing::Test
{
protected:
    static void SetUpTestSuite()
    {
        rclcpp::init(0, nullptr);
    }

    static void TearDownTestSuite()
    {
        rclcpp::shutdown();
    }

    void SetUp() override
    {
        node_ = std::make_shared<SyncNode>();

        exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        exec_->add_node(node_);
    }

    void TearDown() override
    {
        exec_->remove_node(node_);
        node_.reset();
    }

    void spin_some()
    {
        exec_->spin_some();
    }

    std::shared_ptr<SyncNode> node_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;
};

TEST_F(SyncNodeFixture, publishes_when_all_three_are_synchronized)
{
    std::promise<void> received_promise;
    auto received_future = received_promise.get_future();

    auto qos = rclcpp::QoS(10).best_effort();

    auto sub = node_->create_subscription<SyncedSensorData>(
        "synced_data",
        qos,
        [&](SyncedSensorData::SharedPtr) {
            received_promise.set_value();
        }
    );

    auto imu_pub = node_->create_publisher<IMUState>("imu/state", qos);
    auto pressure_pub = node_->create_publisher<PressureState>("pressure/state", qos);
    auto actuator_pub = node_->create_publisher<ActuatorState>("dxl/state", qos);

    spin_some();

    rclcpp::Time now = node_->now();

    IMUState imu;
    imu.header.stamp = now;

    PressureState pressure;
    pressure.header.stamp = now;

    ActuatorState actuator;
    actuator.header.stamp = now;

    imu_pub->publish(imu);
    pressure_pub->publish(pressure);
    actuator_pub->publish(actuator);

    auto result = exec_->spin_until_future_complete(
        received_future,
        std::chrono::milliseconds(500)
    );

    EXPECT_EQ(result, rclcpp::FutureReturnCode::SUCCESS);
}