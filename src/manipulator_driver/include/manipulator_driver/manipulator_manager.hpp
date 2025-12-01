#ifndef MANIPULATOR_MANAGER_HPP
#define MANIPULATOR_MANAGER_HPP

#include <cstdint>
#include <cstdlib>
#include <cstring>

class ManipulatorManager
{
	public:
		explicit ManipulatorManager();
		~ManipulatorManager();
		
		uint8_t* createPacket(uint8_t id, uint8_t instr, uint8_t* parameters, uint8_t parameter_size, uint8_t& out_size);
	private:

};

#endif // MANIPULATOR_MANAGER_HPP
