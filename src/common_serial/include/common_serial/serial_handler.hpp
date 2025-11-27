#ifndef SERIAL_HANDLER_HPP 
#define SERIAL_HANDLER_HPP

#include <cstdint>
#include <cstring>
#include <iostream>
#include <vector>
#include <string>

#include <cerrno>
#include <fcntl.h>
#include <unistd.h>

/* 
 * Fonte:
 * https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/
*/

class SerialHandler
{
    public:
        SerialHandler();
        virtual ~SerialHandler();

        int init(const char* device, int baudrate);
        int setBaudRate(int speed);
        int writeData(const std::vector<uint8_t>& data);
        int readData(char *buffer, int length);

    private:
        int setConfigs();
        
        int fd_;
};

#endif // SERIAL_HANDLER_HPP
