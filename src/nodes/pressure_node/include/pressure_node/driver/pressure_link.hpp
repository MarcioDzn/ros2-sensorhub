#ifndef PRESSURE_LINK_HPP
#define PRESSURE_LINK_HPP

#include <cstdint>
#include <memory>

#include "common_serial/serial_handler.hpp"

class PressureLink 
{
    public:
        PressureLink(std::shared_ptr<SerialHandler> transport)
            :   transport_(std::move(transport)) {}

        PressureLink() = default;

        bool readCString(char* buffer, size_t max_size);

    private:
        std::shared_ptr<SerialHandler> transport_;

};

#endif // PRESSURE_LINK_HPP