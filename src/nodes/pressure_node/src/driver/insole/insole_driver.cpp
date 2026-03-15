#include "driver/insole/insole_driver.hpp"
#include <sstream>
#include <string>

InsoleDriver::InsoleDriver() {}

int InsoleDriver::init(std::string device, int baudrate)
{
    transport_ = std::make_unique<SerialHandler>();
    if (transport_->init(device.c_str()) < 0)
        return -1; 
    transport_->setDefaultConfig();
    transport_->setBaudRate(baudrate);

    return 0;
}

std::vector<uint16_t> InsoleDriver::parse_numbers_from_string(
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

int InsoleDriver::get_data(std::vector<uint16_t>& data)
{
    static std::string rx_buffer;
    char tmp[256];
    ssize_t n;

    // drena tudo que tiver na UART
    while ((n = transport_->readData(tmp, sizeof(tmp))) > 0)
    {
        rx_buffer.append(tmp, n);
    }

    // procura todas as linhas completas e mantem a ultima
    std::string last_line;
    size_t pos;
    while ((pos = rx_buffer.find('\n')) != std::string::npos)
    {
        last_line = rx_buffer.substr(0, pos);
        rx_buffer.erase(0, pos + 1);
    }

    if (last_line.empty())
        return -1; 

    data = parse_numbers_from_string(last_line);

    if (data.empty())
        return -2; 

    return 0; 
}
