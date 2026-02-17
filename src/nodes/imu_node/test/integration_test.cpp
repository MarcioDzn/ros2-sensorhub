#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <rclcpp/rclcpp.hpp>

#include <pty.h>
#include <unistd.h>
#include <fcntl.h>
#include <chrono>
#include <map>

#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <stdarg.h> // Para va_list e va_arg

#include "imu_node.hpp"
#include "driver/imu.hpp"

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

    int ioctl(int fd, unsigned long request, ...) {
        va_list args;
        va_start(args, request);

        if (request == I2C_RDWR) {
            struct i2c_rdwr_ioctl_data* msgset = va_arg(args, struct i2c_rdwr_ioctl_data*);
            
            if (msgset != nullptr && msgset->nmsgs >= 2) {
                
                uint8_t reg_inicial = msgset->msgs[0].buf[0];

                uint8_t* buffer_destino = msgset->msgs[1].buf;
                int tamanho = msgset->msgs[1].len;

                for (int i = 0; i < tamanho; i++) {
                    
                    buffer_destino[i] = mux_registers[current_sel_a][current_sel_b][fd][reg_inicial + i];
                }
            }
            va_end(args);
            return 0; 
        }

        va_end(args);
        return 0; 
    }
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

        mux_registers[0][0][40][IMU::BNO055_EULER_R_LSB_ADDR] = 0x40;
        mux_registers[0][0][40][IMU::BNO055_EULER_R_MSB_ADDR] = 0x01; 

        mux_registers[0][0][40][IMU::BNO055_CHIP_ID_ADDR] = BNO055_ID; 
        mux_registers[0][0][41][IMU::BNO055_CHIP_ID_ADDR] = BNO055_ID;
        mux_registers[0][1][40][IMU::BNO055_CHIP_ID_ADDR] = BNO055_ID;

        sub_node = std::make_shared<rclcpp::Node>("test_subscriber");

        rclcpp::NodeOptions options;
        options.append_parameter_override("update_rate_ms", 15);
        options.append_parameter_override("ids", std::vector<int64_t>{1, 2, 3});
        options.append_parameter_override("multiplexer", std::vector<int64_t>{0, 1, 0});
        options.append_parameter_override("addresses", std::vector<int64_t>{40, 41, 40});

        node = std::make_shared<IMUNode>(options);
    }

    void TearDown() {
        node.reset();
        sub_node.reset();
    }
};


TEST(IMUInitTest, init_success)
{
if (!rclcpp::ok()) rclcpp::init(0, nullptr);
    
    // limpa e prepara o mock com o ID correto
    mux_registers.clear();
    mux_registers[0][0][40][IMU::BNO055_CHIP_ID_ADDR] = BNO055_ID;

    rclcpp::NodeOptions options;
    options.append_parameter_override("ids", std::vector<int64_t>{1});
    options.append_parameter_override("multiplexer", std::vector<int64_t>{0});
    options.append_parameter_override("addresses", std::vector<int64_t>{40});

    ASSERT_NO_THROW({
        auto test_node = std::make_shared<IMUNode>(options);
    });
    
    rclcpp::shutdown();
}

TEST(IMUInitTest, init_fail_wrong_id)
{
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
    
    mux_registers.clear();
    // coloca um ID qualquer (0xFF) em vez do BNO055_ID (0xA0)
    mux_registers[0][0][40][IMU::BNO055_CHIP_ID_ADDR] = 0xFF;

    rclcpp::NodeOptions options;
    options.append_parameter_override("ids", std::vector<int64_t>{1});
    options.append_parameter_override("multiplexer", std::vector<int64_t>{0});
    options.append_parameter_override("addresses", std::vector<int64_t>{40});

    ASSERT_THROW({
        auto test_node = std::make_shared<IMUNode>(options);
    }, std::runtime_error);
    
    rclcpp::shutdown();
}


TEST_F(IMUFixture, read_quaternions_success)
{
    // LSB 0x40 e MSB 0x40 -> 16448 decimal
    uint8_t mock_val = 0x40;
    
    mux_registers[0][0][40][IMU::BNO055_QUATERNION_DATA_W_LSB_ADDR] = mock_val;
    mux_registers[0][0][40][IMU::BNO055_QUATERNION_DATA_W_MSB_ADDR] = mock_val;

    mux_registers[0][0][40][IMU::BNO055_QUATERNION_DATA_X_LSB_ADDR] = mock_val;
    mux_registers[0][0][40][IMU::BNO055_QUATERNION_DATA_X_MSB_ADDR] = mock_val;

    mux_registers[0][0][40][IMU::BNO055_QUATERNION_DATA_Y_LSB_ADDR] = mock_val;
    mux_registers[0][0][40][IMU::BNO055_QUATERNION_DATA_Y_MSB_ADDR] = mock_val;

    mux_registers[0][0][40][IMU::BNO055_QUATERNION_DATA_Z_LSB_ADDR] = mock_val;
    mux_registers[0][0][40][IMU::BNO055_QUATERNION_DATA_Z_MSB_ADDR] = mock_val;

    using IMUState = interfaces::msg::IMUState;
    IMUState::SharedPtr received_msg = nullptr;

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
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

    ASSERT_NE(received_msg, nullptr);
    ASSERT_FALSE(received_msg->imus.empty());

    const auto & q = received_msg->imus[0];
    
    // com 4 eixos de valores iguais (16448), 
    // a normalização faz: 16448 / sqrt(16448² + 16448² + 16448² + 16448²) = 0.5
    double val = 16448.0 / 16384.0;
    double expected_val = val / std::sqrt(val*val + val*val + val*val + val*val);

    EXPECT_NEAR(q.q_w, expected_val, 0.0001);
    EXPECT_NEAR(q.q_x, expected_val, 0.0001);
    EXPECT_NEAR(q.q_y, expected_val, 0.0001);
    EXPECT_NEAR(q.q_z, expected_val, 0.0001);

    // Validação extra: a norma final deve ser sempre 1.0
    double norm = std::sqrt(q.q_w*q.q_w + q.q_x*q.q_x + q.q_y*q.q_y + q.q_z*q.q_z);
    EXPECT_NEAR(norm, 1.0, 0.0001);
}

TEST_F(IMUFixture, read_multisensor_quaternions_success)
{
    // TODO: colocar mock consistente com um quaternion
    uint8_t mock_val_lsb = 0x40;
    uint8_t mock_val_msb = 0x01;
    
    // 1
    mux_registers[0][0][40][IMU::BNO055_QUATERNION_DATA_W_LSB_ADDR] = mock_val_lsb;
    mux_registers[0][0][40][IMU::BNO055_QUATERNION_DATA_W_MSB_ADDR] = mock_val_msb;

    mux_registers[0][0][40][IMU::BNO055_QUATERNION_DATA_X_LSB_ADDR] = mock_val_lsb;
    mux_registers[0][0][40][IMU::BNO055_QUATERNION_DATA_X_MSB_ADDR] = mock_val_msb;

    mux_registers[0][0][40][IMU::BNO055_QUATERNION_DATA_Y_LSB_ADDR] = mock_val_lsb;
    mux_registers[0][0][40][IMU::BNO055_QUATERNION_DATA_Y_MSB_ADDR] = mock_val_msb;

    mux_registers[0][0][40][IMU::BNO055_QUATERNION_DATA_Z_LSB_ADDR] = mock_val_lsb;
    mux_registers[0][0][40][IMU::BNO055_QUATERNION_DATA_Z_MSB_ADDR] = mock_val_msb;


    // 2
    mux_registers[0][0][41][IMU::BNO055_QUATERNION_DATA_W_LSB_ADDR] = mock_val_lsb;
    mux_registers[0][0][41][IMU::BNO055_QUATERNION_DATA_W_MSB_ADDR] = mock_val_msb;

    mux_registers[0][0][41][IMU::BNO055_QUATERNION_DATA_X_LSB_ADDR] = mock_val_lsb;
    mux_registers[0][0][41][IMU::BNO055_QUATERNION_DATA_X_MSB_ADDR] = mock_val_msb;

    mux_registers[0][0][41][IMU::BNO055_QUATERNION_DATA_Y_LSB_ADDR] = mock_val_lsb;
    mux_registers[0][0][41][IMU::BNO055_QUATERNION_DATA_Y_MSB_ADDR] = mock_val_msb;

    mux_registers[0][0][41][IMU::BNO055_QUATERNION_DATA_Z_LSB_ADDR] = mock_val_lsb;
    mux_registers[0][0][41][IMU::BNO055_QUATERNION_DATA_Z_MSB_ADDR] = mock_val_msb;


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
    
    //ASSERT_EQ(received_msg->imus.size(), 2);

    // Valor bruto mockado
    int16_t raw = (0x01 << 8) | 0x40;   // 0x0140 = 320
    double val = static_cast<double>(raw) / 16384.0;

    // Como W=X=Y=Z
    double expected = val / std::sqrt(4 * val * val);

    // Sensor 0
    EXPECT_NEAR(received_msg->imus[0].q_w, expected, 0.0001);
    EXPECT_NEAR(received_msg->imus[0].q_x, expected, 0.0001);
    EXPECT_NEAR(received_msg->imus[0].q_y, expected, 0.0001);
    EXPECT_NEAR(received_msg->imus[0].q_z, expected, 0.0001);

    // Sensor 1
    EXPECT_NEAR(received_msg->imus[1].q_w, expected, 0.0001);
    EXPECT_NEAR(received_msg->imus[1].q_x, expected, 0.0001);
    EXPECT_NEAR(received_msg->imus[1].q_y, expected, 0.0001);
    EXPECT_NEAR(received_msg->imus[1].q_z, expected, 0.0001);

    // Norma deve ser 1
    auto norm = [](const auto &q) {
        return std::sqrt(q.q_w*q.q_w +
                        q.q_x*q.q_x +
                        q.q_y*q.q_y +
                        q.q_z*q.q_z);
    };

    EXPECT_NEAR(norm(received_msg->imus[0]), 1.0, 0.0001);
    EXPECT_NEAR(norm(received_msg->imus[1]), 1.0, 0.0001);
}

TEST_F(IMUFixture, read_conflict_mux_quaternions_success)
{
    // TODO: colocar mock consistente com um quaternion
    uint8_t mock_val_lsb = 0x40;
    uint8_t mock_val_msb = 0x01;
    
    // 1
    mux_registers[0][0][40][IMU::BNO055_QUATERNION_DATA_W_LSB_ADDR] = mock_val_lsb;
    mux_registers[0][0][40][IMU::BNO055_QUATERNION_DATA_W_MSB_ADDR] = mock_val_msb;

    mux_registers[0][0][40][IMU::BNO055_QUATERNION_DATA_X_LSB_ADDR] = mock_val_lsb;
    mux_registers[0][0][40][IMU::BNO055_QUATERNION_DATA_X_MSB_ADDR] = mock_val_msb;

    mux_registers[0][0][40][IMU::BNO055_QUATERNION_DATA_Y_LSB_ADDR] = mock_val_lsb;
    mux_registers[0][0][40][IMU::BNO055_QUATERNION_DATA_Y_MSB_ADDR] = mock_val_msb;

    mux_registers[0][0][40][IMU::BNO055_QUATERNION_DATA_Z_LSB_ADDR] = mock_val_lsb;
    mux_registers[0][0][40][IMU::BNO055_QUATERNION_DATA_Z_MSB_ADDR] = mock_val_msb;


    // 2
    mux_registers[0][1][40][IMU::BNO055_QUATERNION_DATA_W_LSB_ADDR] = mock_val_lsb;
    mux_registers[0][1][40][IMU::BNO055_QUATERNION_DATA_W_MSB_ADDR] = mock_val_msb;

    mux_registers[0][1][40][IMU::BNO055_QUATERNION_DATA_X_LSB_ADDR] = mock_val_lsb;
    mux_registers[0][1][40][IMU::BNO055_QUATERNION_DATA_X_MSB_ADDR] = mock_val_msb;

    mux_registers[0][1][40][IMU::BNO055_QUATERNION_DATA_Y_LSB_ADDR] = mock_val_lsb;
    mux_registers[0][1][40][IMU::BNO055_QUATERNION_DATA_Y_MSB_ADDR] = mock_val_msb;

    mux_registers[0][1][40][IMU::BNO055_QUATERNION_DATA_Z_LSB_ADDR] = mock_val_lsb;
    mux_registers[0][1][40][IMU::BNO055_QUATERNION_DATA_Z_MSB_ADDR] = mock_val_msb;

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
    

    // Valor bruto mockado
    int16_t raw = (0x01 << 8) | 0x40;   // 0x0140 = 320
    double val = static_cast<double>(raw) / 16384.0;

    // Como W=X=Y=Z
    double expected = val / std::sqrt(4 * val * val);

    // Sensor 0
    EXPECT_NEAR(received_msg->imus[0].q_w, expected, 0.0001);
    EXPECT_NEAR(received_msg->imus[0].q_x, expected, 0.0001);
    EXPECT_NEAR(received_msg->imus[0].q_y, expected, 0.0001);
    EXPECT_NEAR(received_msg->imus[0].q_z, expected, 0.0001);

    // Sensor 1
    EXPECT_NEAR(received_msg->imus[2].q_w, expected, 0.0001);
    EXPECT_NEAR(received_msg->imus[2].q_x, expected, 0.0001);
    EXPECT_NEAR(received_msg->imus[2].q_y, expected, 0.0001);
    EXPECT_NEAR(received_msg->imus[2].q_z, expected, 0.0001);

    // Norma deve ser 1
    auto norm = [](const auto &q) {
        return std::sqrt(q.q_w*q.q_w +
                        q.q_x*q.q_x +
                        q.q_y*q.q_y +
                        q.q_z*q.q_z);
    };

    EXPECT_NEAR(norm(received_msg->imus[0]), 1.0, 0.0001);
    EXPECT_NEAR(norm(received_msg->imus[2]), 1.0, 0.0001);
}
