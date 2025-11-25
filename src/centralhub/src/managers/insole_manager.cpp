#include "managers/insole_manager.hpp"

InsoleManager::InsoleManager(rclcpp::Node* node) : node_(node)
{}

int InsoleManager::initComm(const char* device, int baudrate)
{
    return device_.init(device, baudrate);
}

int InsoleManager::receivePacket(char *buffer, int length)
{
    return device_.readStringData(buffer, length);
}

void InsoleManager::createPublisher()
{
    auto qos = rclcpp::QoS(10).reliable();
    publisher_ = node_->create_publisher<InsoleData>("insole1/pressure", qos);
}

void InsoleManager::publishAll()
{
    InsoleData message;

    char buffer[128];  // buffer maior para pacotes completos
    int n = receivePacket(buffer, sizeof(buffer) - 1);

    if (n <= 0) {
        RCLCPP_ERROR(node_->get_logger(), "FALHA AO LER O PACOTE DE DADOS DE PRESSAO");
        return;
    }

    buffer[n] = '\0'; // termina string

    std::string packet(buffer);
    packet.erase(packet.find_last_not_of(" \r\n") + 1);

    RCLCPP_INFO(node_->get_logger(), "Pacote original: \"%s\" (%zu bytes)", packet.c_str(), packet.size());

    std::istringstream iss(packet);
    std::string word;
    int count = 0;

    RCLCPP_INFO(node_->get_logger(), "Valores tratados: ");

    while (iss >> word && count < 15) {
        try {
            uint16_t value = static_cast<uint16_t>(std::stoi(word));
            message.pressures.push_back(value);
            RCLCPP_INFO(node_->get_logger(), "%u", value);
            count++;
        } catch (...) {
            RCLCPP_WARN(node_->get_logger(), "VALOR INVALIDO NO PACOTE: %s", word.c_str());
        }
    }


    if (message.pressures.size() != 15) {
        RCLCPP_WARN(node_->get_logger(), "ESPERADO 15 VALORES, OBTEVE-SE %zu", message.pressures.size());
        return; 
    }

    publisher_->publish(message);
}


