#include "driver/pressure_link.hpp"

// DEBUG
bool PressureLink::readCString(char* buffer, size_t max_size)
{
    size_t i = 0;
    char c;
    
    while(i < max_size-1)
    {
        ssize_t n = transport_->readData(&c, 1);
        if (n <= 0) break; 
        buffer[i++] = c;
        if (c == '\0') break; 
    }
    buffer[i] = '\0';
    return (i > 0);
}