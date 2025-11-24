#include "central_node.hpp"
#include "managers/imu_manager.hpp"
#include "managers/motor_manager.hpp"

#define BAUDRATE 2000000

using namespace std::chrono_literals;

CentralNode::CentralNode() : Node("central_node"), count_(0)
{
    motor_manager_ = std::make_unique<MotorManager>(this, 1);
    uint8_t error = 0;
    
    if (motor_manager_->initComm("/dev/ttyUSB0", BAUDRATE) < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Erro ao inicializar a comunicacao!");
    }
    
    std::vector<uint8_t> response;
    motor_manager_->enableTorque(&error);
    motor_manager_->getPresentPosition(response, &error);
    motor_manager_->enableLED(&error);
    
    //RCLCPP_INFO(this->get_logger(), "Response: %02X %02X %02X %02X %02X %02X %02X %02X",
    //response[0], response[1], response[2], response[3], response[4], response[5], response[6], response[7]);

    imu_manager_ = std::make_unique<IMUManager>(this);
    imu_manager_->loadParameters();
    imu_manager_->createSensors();
    imu_manager_->initialize();     
    imu_manager_->createPublishers();
    
    // parâmetros do central node
    this->declare_parameter<int>("update_rate_ms", 15);
    update_rate_ms_ = this->get_parameter("update_rate_ms").as_int();

    // executa o callback a cada 
    // <update_rate_ms_> segundos
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(update_rate_ms_), 
        std::bind(&CentralNode::timer_callback, this));
}

// callback que envia os dados dos 3 sensores
void CentralNode::timer_callback()
{
    imu_manager_->publishAll();
}

CentralNode::~CentralNode() = default;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CentralNode>());
    rclcpp::shutdown();
    return 0;
}
