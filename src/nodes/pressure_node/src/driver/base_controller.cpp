#include "driver/base_controller.hpp"

BaseController::BaseController() {}

int BaseController::init(std::string device, int baudrate)
{
    auto transport = std::make_shared<SerialHandler>();
    if (transport->init(device.c_str()) < 0)
        return -1; 
    transport->setDefaultConfig();
    transport->setBaudRate(baudrate);

    link_ = std::make_shared<PressureLink>(transport);

    return 0;
}

int BaseController::getData(std::vector<uint16_t>& data)
{
    char buffer[BUFFER_SIZE];

    if (!link_->readCString(buffer, BUFFER_SIZE)) {
        return -1; 
    }

    data = parseNumbersFromString(std::string(buffer));

    if (data.empty()) {
        return -2;
    }
    return 0; 
}

std::vector<uint16_t> BaseController::parseNumbersFromString(
    const std::string& input)
{
    std::vector<uint16_t> values;
    std::stringstream ss(input);
    std::string token;
    
    while (ss >> token) {
        try {
            values.push_back(static_cast<uint16_t>(std::stoul(token)));
        } catch (...) {
            continue;
        }
    } 
    return values;
}