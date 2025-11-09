#ifndef INTERFACE_SENSOR_MANAGER_HPP
#define INTERFACE_SENSOR_MANAGER_HPP

class ISensorManager
{
    public:
        virtual ~ISensorManager() = default;
        virtual void loadParameters() = 0;
        virtual void createSensors() = 0; 
        virtual void createPublishers() = 0; 
        virtual void publishAll() = 0;
};

#endif // INTERFACE_SENSOR_MANAGER_HPP