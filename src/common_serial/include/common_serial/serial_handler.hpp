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

        int init(const char* device);
        int setBaudRate(int speed);
        int setDefaultConfig();
        
        template<typename T>
        ssize_t readData(T *buffer, size_t size)
        {
            return readGenericData(buffer, size);
        }
        
        ssize_t writeData(const char* buffer, size_t size);
        
    private:
        
        ssize_t readGenericData(void *buffer, size_t size);
        
        int fd_;
};

#endif // SERIAL_HANDLER_HPP
