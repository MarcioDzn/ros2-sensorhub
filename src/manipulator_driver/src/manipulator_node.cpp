#include "manipulator_node.hpp"

#define DEVICE                      "/dev/ttyUSB0"
#define BAUDRATE                    2000000

using namespace std::chrono_literals;

ManipulatorNode::ManipulatorNode() : Node("manipulator_node")
{
    load_parameters();
}

// callback que envia os dados do sensor
void ManipulatorNode::timer_callback()
{
}

void ManipulatorNode::load_parameters()
{
    set_parameters();
}

void ManipulatorNode::set_parameters()
{
}

bool ManipulatorNode::init_serial(const char* device, int baudrate)
{
    serial_handler_ = std::make_unique<SerialHandler>();
    
    if ( serial_handler_->init(device) < 0 )
    {
        RCLCPP_ERROR(this->get_logger(), "Erro ao inicializar a porta serial");
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "Inicializacao realizada");
    
    if ( serial_handler_->setDefaultConfig() < 0 )
    {
        RCLCPP_ERROR(this->get_logger(), "Erro ao aplicar a configuracao padrao da porta serial");
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "Configuracao realizada");
    
    if ( serial_handler_->setBaudRate(baudrate) < 0 )
    {
        RCLCPP_ERROR(this->get_logger(), "Erro ao definir baudrate");
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "Aplicacao de baurate realizada");
    
    return true;
}

void ManipulatorNode::send_packet()
{
    uint8_t packet_torque[] = {0xFF, 0xFF, 0x01, 0x04, 0x03, 0x18, 0x01, 0xDE};
    auto n1 = serial_handler_->writeData(packet_torque, sizeof(packet_torque));
    RCLCPP_INFO(this->get_logger(), "%d", n1);
    
    uint8_t packet_goal[] = {0xFF,0xFF,0x01,0x05,0x03,0x1E,0xDC,0x05,0xF7};
    auto n2 = serial_handler_->writeData(packet_goal, sizeof(packet_goal));
    RCLCPP_INFO(this->get_logger(), "%d", n2);

}

ManipulatorNode::~ManipulatorNode() = default;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<ManipulatorNode>();
    
    const char* device = DEVICE;
    int baudrate = BAUDRATE;
    if ( !node->init_serial(device, baudrate) )
    {
        RCLCPP_FATAL(node->get_logger(), "Erro ao configurar porta serial. Finalizando execucao");
        rclcpp::shutdown();
        return 1;
    }
    RCLCPP_INFO(node->get_logger(), "ManipulatorNode inicializado");
    node->send_packet();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
