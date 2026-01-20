#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <rclcpp/rclcpp.hpp>

#include <pty.h>
#include <unistd.h>
#include <fcntl.h>
#include <chrono>
#include <map>

#include "imu_node.hpp"
#include "driver/common/imu.hpp"

#include "interfaces/msg/imu_state.hpp"

// variáveis globais para rastreio do estado dos
// pinos do mux
int current_sel_a = 0;
int current_sel_b = 0;

// [SEL_A][SEL_B][ADDRESS][REGISTER]
std::map<int, std::map<int, std::map<int, std::map<uint8_t, uint8_t>>>> mux_registers;

extern "C" {
    int wiringPiSetup(void) { return 0; }
    
    int wiringPiI2CSetup(int devId) { 
        return devId;
    }

    void digitalWrite(int p, int v) {
        if (p == 2) current_sel_a = v; // SEL_A (Pino 2)
        if (p == 0) current_sel_b = v; // SEL_B (Pino 0)
    }

    int wiringPiI2CReadReg8(int fd, int reg) {
        return mux_registers[current_sel_a][current_sel_b][fd][static_cast<uint8_t>(reg)];
    }

    int wiringPiI2CWriteReg8(int fd, int reg, int data) {
        mux_registers[current_sel_a][current_sel_b][fd][static_cast<uint8_t>(reg)] = static_cast<uint8_t>(data);
        return 0;
    }

    void pinMode(int p, int m) { (void)p; (void)m; }
    void delay(unsigned int t) { (void)t; }
}

using namespace std::chrono_literals;

class IMUFixture : public ::testing::Test {
    protected:
        std::shared_ptr<IMUNode> node;
        std::shared_ptr<rclcpp::Node> sub_node;

    void SetUp() override {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }

        mux_registers.clear();

        mux_registers[0][0][40][IMU::BNO055_CHIP_ID_ADDR] = BNO055_ID; 
        mux_registers[0][0][41][IMU::BNO055_CHIP_ID_ADDR] = BNO055_ID;

        sub_node = std::make_shared<rclcpp::Node>("test_subscriber");

        rclcpp::NodeOptions options;
        options.append_parameter_override("base_name", "imu");
        options.append_parameter_override("update_rate_ms", 15);
        options.append_parameter_override("ids", std::vector<int64_t>{1, 2});
        options.append_parameter_override("multiplexer", std::vector<int64_t>{0, 1});
        options.append_parameter_override("addresses", std::vector<int64_t>{40, 41});

        node = std::make_shared<IMUNode>(options);
    }

    void TearDown() {
        node.reset();
        sub_node.reset();
    }
};

TEST_F(IMUFixture, read_euler_success)
{
    // simula dados de Euler no mapa (Roll, Pitch e Yaw = 20 graus)
    // 20.0 * 16 = 320 -> 0x0140
    // roll
    mux_registers[0][0][40][IMU::BNO055_EULER_R_LSB_ADDR] = 0x40;
    mux_registers[0][0][40][IMU::BNO055_EULER_R_MSB_ADDR] = 0x01; 

    // pitch
    mux_registers[0][0][40][IMU::BNO055_EULER_P_LSB_ADDR] = 0x40; 
    mux_registers[0][0][40][IMU::BNO055_EULER_P_MSB_ADDR] = 0x01; 

    // yaw
    mux_registers[0][0][40][IMU::BNO055_EULER_H_LSB_ADDR] = 0x40; 
    mux_registers[0][0][40][IMU::BNO055_EULER_H_MSB_ADDR] = 0x01; 

    using IMUState = interfaces::msg::IMUState;
    IMUState::SharedPtr received_msg = nullptr;

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
        .best_effort()
        .durability_volatile();
    auto sub = sub_node->create_subscription<IMUState>(
        "imu/state", qos, [&](IMUState::SharedPtr msg) {
            received_msg = msg;
        });

    auto start_time = std::chrono::steady_clock::now();
    while (!received_msg && (std::chrono::steady_clock::now() - start_time < 2s)) {
        rclcpp::spin_some(node);
        rclcpp::spin_some(sub_node);
        std::this_thread::sleep_for(10ms);
    }

    ASSERT_NE(received_msg, nullptr) << "Timeout: O nó não publicou nada!";
    ASSERT_FALSE(received_msg->imus.empty());
    
    // 320 / 16.0 = 20.0
    EXPECT_NEAR(received_msg->imus[0].roll, 20.0f, 0.05);
    EXPECT_NEAR(received_msg->imus[0].pitch, 20.0f, 0.05);
    EXPECT_NEAR(received_msg->imus[0].yaw, 20.0f, 0.05);
}

TEST_F(IMUFixture, read_multisensor_euler_success)
{
    // simula dados de Euler no mapa (Roll = 20 graus)
    // 20.0 * 16 = 320 -> 0x0140
    // sensor de id = 1 (roll)
    mux_registers[0][0][40][IMU::BNO055_EULER_R_LSB_ADDR] = 0x40;
    mux_registers[0][0][40][IMU::BNO055_EULER_R_MSB_ADDR] = 0x01; 

    // sensor de id = 2 (roll)
    mux_registers[0][0][41][IMU::BNO055_EULER_R_LSB_ADDR] = 0x40;
    mux_registers[0][0][41][IMU::BNO055_EULER_R_MSB_ADDR] = 0x01; 


    using IMUState = interfaces::msg::IMUState;
    IMUState::SharedPtr received_msg = nullptr;

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
        .best_effort()
        .durability_volatile();
    auto sub = sub_node->create_subscription<IMUState>(
        "imu/state", qos, [&](IMUState::SharedPtr msg) {
            received_msg = msg;
        });

    auto start_time = std::chrono::steady_clock::now();
    while (!received_msg && (std::chrono::steady_clock::now() - start_time < 2s)) {
        rclcpp::spin_some(node);
        rclcpp::spin_some(sub_node);
        std::this_thread::sleep_for(10ms);
    }

    ASSERT_NE(received_msg, nullptr) << "Timeout: O nó não publicou nada!";
    ASSERT_FALSE(received_msg->imus.empty());
    
    // 320 / 16.0 = 20.0
    EXPECT_NEAR(received_msg->imus[0].roll, 20.0f, 0.05);
    EXPECT_NEAR(received_msg->imus[1].roll, 20.0f, 0.05);
}