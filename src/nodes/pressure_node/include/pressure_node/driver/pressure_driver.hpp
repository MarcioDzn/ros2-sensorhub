#ifndef PRESSURE_DRIVER_HPP
#define PRESSURE_DRIVER_HPP

#include "common_serial/serial_handler.hpp"

#include <cstdint>
#include <string>
#include <vector>
#include <memory>

#define MAX_BUFFER_COLLECT          6 * 16
#define BUFFER_SIZE                 8 * 16 * 5

class PressureDriver
{
    public:
        explicit PressureDriver();
        virtual ~PressureDriver() = default;

        int init(std::string device, int baudrate);
        int getData(std::vector<uint16_t>& data);

    private:
        std::vector<uint16_t> parseNumbersFromString(const std::string& input);
        bool readCString(char* buffer, size_t max_size);

        std::unique_ptr<SerialHandler> transport_;
};

#endif // PRESSURE_DRIVER_HPP