#include "pressure_node.hpp"

using namespace std::chrono_literals;

PressureNode::PressureNode() : Node("pressure_node")
{
    // carrega parametros
    load_parameters();
    
    auto qos = rclcpp::QoS(10).reliable();
    // cria um publisher pra cada sensor de pressao
    RCLCPP_INFO(this->get_logger(), 
        "Criando publisher para o sensor de pressao");
    auto publisher = this->create_publisher<InsoleData>("/pressure", qos);
    
    // executa o callback a cada <update_rate_ms_> segundos
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(update_rate_ms_), 
        std::bind(&PressureNode::timer_callback, this));
}

// callback que envia os dados do sensor
void PressureNode::timer_callback()
{
    auto message = InsoleData();
    publisher_->publish(message);

}

void PressureNode::load_parameters()
{
    this->declare_parameter<int>("update_rate_ms", 15);
    set_parameters();
}

void PressureNode::set_parameters()
{
    update_rate_ms_ = this->get_parameter("update_rate_ms").as_int();
}

bool PressureNode::init_serial(const char* device, int baudrate)
{
    serial_handler_ = std::make_unique<SerialHandler>();
    
    if ( serial_handler_->init(device) < 0 )
    {
        RCLCPP_ERROR(this->get_logger(), "Erro ao inicializar a porta serial");
        return false;
    }
    
    if ( serial_handler_->setDefaultConfig() < 0 )
    {
        RCLCPP_ERROR(this->get_logger(), "Erro ao aplicar a configuracao padrao da porta serial");
        return false;
    }
    
    if ( serial_handler_->setBaudRate(baudrate) < 0 )
    {
        RCLCPP_ERROR(this->get_logger(), "Erro ao definir baudrate");
        return false;
    }
    
    return true;
}

PressureNode::~PressureNode() = default;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<PressureNode>();
    
    char* device = "/dev/ttyACM0";
    int baudrate = 115200;
    if ( !node->init_serial(device, baudrate) )
    {
        RCLCPP_FATAL(node->get_logger(), "Erro ao configurar porta serial. Finalizando execucao");
        rclcpp::shutdown();
        return 1;
    }
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
