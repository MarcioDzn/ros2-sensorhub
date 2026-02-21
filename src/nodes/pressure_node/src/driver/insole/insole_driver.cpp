#include "driver/insole/insole_driver.hpp"

#include <sstream>

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
    char buffer[BUFFER_SIZE];

    if (!read_c_string(buffer, BUFFER_SIZE)) 
        return -1; 

    data = parse_numbers_from_string(std::string(buffer));
    if (data.empty()) 
        return -2;

    return 0; 
}


bool InsoleDriver::read_c_string(char* buffer, size_t max_size)
{
    size_t i = 0;
    char c;
    
    while(i < max_size-1)
    {
        ssize_t n = transport_->readData(&c, 1);
        if (n <= 0) break; 
        buffer[i++] = c;
        if (c == '\n') break; 
    }
    buffer[i] = '\0';
    return (i > 0);
}
