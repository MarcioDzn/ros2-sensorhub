#include "central_node.hpp"
#include "managers/imu_manager.hpp"
#include "managers/motor_manager.hpp"

#define BAUDRATE 2000000

using namespace std::chrono_literals;

CentralNode::CentralNode() : Node("central_node"), count_(0)
{
    // parâmetros do central node
    this->declare_parameter<int>("update_rate_ms", 15);
    this->declare_parameter<int>("goal_position", 1000);
    update_rate_ms_ = this->get_parameter("update_rate_ms").as_int();
    uint16_t goal_position = this->get_parameter("goal_position").as_int();
    
    motor_manager_ = std::make_unique<MotorManager>(this, 1);
    uint8_t error = 0;
    
    if (motor_manager_->initComm("/dev/ttyUSB0", BAUDRATE) < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Erro ao inicializar a comunicacao!");
    }
    
    // service do motor
    motor_service_ = this->create_service<SetMotorConfig>("motor_config", std::bind(
        &CentralNode::motor_service_callback, 
        this,
        std::placeholders::_1,
        std::placeholders::_2
    ));

    imu_manager_ = std::make_unique<IMUManager>(this);
    imu_manager_->loadParameters();
    imu_manager_->createSensors();
    imu_manager_->initialize();     
    imu_manager_->createPublishers();
    
    

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

//ros2 service call /motor_config interfaces/srv/SetMotorConfig "{motor_id: 1, command: 'set_goal_position', params: [1500]}"
//ros2 service call /motor_config interfaces/srv/SetMotorConfig "{motor_id: 1, command: 'enable_torque'}"
//ros2 service call /motor_config interfaces/srv/SetMotorConfig "{motor_id: 1, command: 'get_present_position'}"
void CentralNode::motor_service_callback(
    const std::shared_ptr<SetMotorConfig::Request> request,
    std::shared_ptr<SetMotorConfig::Response> response)
{
    response->success = false;

    motor_manager_->setId(request->motor_id);
    if (request->command == "set_goal_position")
    {
        set_goal_position(request, response);
            
    } else if (request->command == "enable_torque")
    {
        enable_torque(request, response);
        
    } else if (request->command == "disable_torque")
    {
        disable_torque(request, response);
        
    } else if (request->command == "get_present_position")
    {
        get_present_position(request, response);
    }
    
}

void CentralNode::set_goal_position(
    const std::shared_ptr<SetMotorConfig::Request> request,
    std::shared_ptr<SetMotorConfig::Response> response)
{
    uint8_t error = 0;  
    if (request->params.size() < 1)
    {
        RCLCPP_ERROR(this->get_logger(), "Parametros insuficientes");
    } else 
    {
        uint16_t goal_pos = request->params[0];
        motor_manager_->setGoalPosition(goal_pos, &error);
        response->success = (error == 0);
    }
}

void CentralNode::enable_torque(
    const std::shared_ptr<SetMotorConfig::Request> request,
    std::shared_ptr<SetMotorConfig::Response> response)
{
    uint8_t error = 0;
    motor_manager_->enableTorque(&error);
    response->success = (error == 0);
}

void CentralNode::disable_torque(
    const std::shared_ptr<SetMotorConfig::Request> request,
    std::shared_ptr<SetMotorConfig::Response> response)
{
    uint8_t error = 0;
    motor_manager_->disableTorque(&error);
    response->success = (error == 0);
}

void CentralNode::get_present_position(
    const std::shared_ptr<SetMotorConfig::Request> request,
    std::shared_ptr<SetMotorConfig::Response> response)
{
    uint8_t error = 0;
    uint16_t data;
    motor_manager_->getPresentPosition(&data, &error);
    response->success = (error == 0);
    response->result.clear();    
    response->result.push_back(data);
}

CentralNode::~CentralNode() = default;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CentralNode>());
    rclcpp::shutdown();
    return 0;
}
