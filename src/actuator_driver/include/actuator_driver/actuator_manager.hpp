#ifndef ACTUATOR_MANAGER_HPP
#define ACTUATOR_MANAGER_HPP

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <chrono>

#include "common_serial/serial_handler.hpp"

class ActuatorManager
{
	public:
		explicit ActuatorManager();
		~ActuatorManager();
	
		void setSerialHandler(std::shared_ptr<SerialHandler> serial);
		uint8_t setTorque(uint8_t id, uint8_t status);
		uint8_t setGoalPosition(uint8_t id, uint16_t goal_position);
		
	private:
		struct StatusPacket
		{
			uint8_t id;
			uint8_t error;
			std::vector<uint8_t> params;
		};

		uint8_t* createPacket(uint8_t id, uint8_t instr, uint8_t* parameters, uint8_t parameter_size, uint8_t& out_size);
		int readPacket(uint8_t* packet);
		int readStatus(uint8_t id, StatusPacket& out);
		int getPresentPosition(uint8_t id, uint16_t& out);
		
		std::shared_ptr<SerialHandler> serial_handler_;	
};

#endif // ACTUATOR_MANAGER_HPP
