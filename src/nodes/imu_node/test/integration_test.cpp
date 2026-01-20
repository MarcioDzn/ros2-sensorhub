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

std::map<int, std::map<uint8_t, uint8_t>> bno_registers;

extern "C" {
    int wiringPiSetup(void) { return 0; }
    
    int wiringPiI2CSetup(int devId) { 
        return devId;
    }

    int wiringPiI2CReadReg8(int fd, int reg) {
        // se o fd não existe, não cria nova entrada com []
        if (bno_registers.count(fd) == 0) {
                printf("[MOCK] Erro: Tentativa de ler FD inexistente: %d\n", fd);
                return 0;
            }
        return bno_registers[fd][static_cast<uint8_t>(reg)];
    }

    int wiringPiI2CWriteReg8(int fd, int reg, int data) {
        bno_registers[fd][static_cast<uint8_t>(reg)] = static_cast<uint8_t>(data);
        return 0;
    }

    void pinMode(int p, int m) { (void)p; (void)m; }
    void digitalWrite(int p, int v) { (void)p; (void)v; }
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

        bno_registers.clear();

        bno_registers[40][IMU::BNO055_CHIP_ID_ADDR] = BNO055_ID; 
        bno_registers[41][IMU::BNO055_CHIP_ID_ADDR] = BNO055_ID;

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
    bno_registers[40][IMU::BNO055_EULER_R_LSB_ADDR] = 0x40;
    bno_registers[40][IMU::BNO055_EULER_R_MSB_ADDR] = 0x01; 
    bno_registers[40][IMU::BNO055_EULER_P_LSB_ADDR] = 0x40; 
    bno_registers[40][IMU::BNO055_EULER_P_MSB_ADDR] = 0x01; 
    bno_registers[40][IMU::BNO055_EULER_H_LSB_ADDR] = 0x40; 
    bno_registers[40][IMU::BNO055_EULER_H_MSB_ADDR] = 0x01; 

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