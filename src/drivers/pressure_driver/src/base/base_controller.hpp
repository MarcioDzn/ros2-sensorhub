#ifndef BASE_CONTROLLER_HPP
#define BASE_CONTROLLER_HPP

#include <cstdint>
#include <string>
#include <memory>
#include <sstream>

#include "pressure_driver/pressure_controller.hpp"
#include "pressure_link.hpp"

#define MAX_BUFFER_COLLECT          6 * 16
#define BUFFER_SIZE                 8 * 16 * 5

class BaseController : public PressureController
{
    public:
        explicit BaseController();

        int init(std::string device, int baudrate) override;
        int getData(std::vector<uint16_t>& data) override;
        
    private:
        std::vector<uint16_t> parseNumbersFromString(const std::string& input);

        std::shared_ptr<PressureLink> link_;
};

#endif // BASE_CONTROLLER_HPP