#ifndef MANIPULATOR_MANAGER_HPP
#define MANIPULATOR_MANAGER_HPP

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <memory>

#include "common_serial/serial_handler.hpp"

class ManipulatorManager
{
	public:
		explicit ManipulatorManager();
		~ManipulatorManager();
	
		void setSerialHandler(std::shared_ptr<SerialHandler> serial);
		uint8_t setTorque(uint8_t id, uint8_t status);
		uint8_t setGoalPosition(uint8_t id, uint16_t goal_position);
		
	private:
		uint8_t* createPacket(uint8_t id, uint8_t instr, uint8_t* parameters, uint8_t parameter_size, uint8_t& out_size);
		
		std::shared_ptr<SerialHandler> serial_handler_;	
};

#endif // MANIPULATOR_MANAGER_HPP
