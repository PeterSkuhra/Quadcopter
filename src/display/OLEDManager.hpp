#ifndef OLED_MANAGER_HPP
#define OLED_MANAGER_HPP

#include <Arduino.h>

#include "IDisplayManager.hpp"

namespace display
{

#define DEFAULT_I2C_ADDRESS     0x3C

class OLEDManager : public IDisplayManager
{
public:

    OLEDManager(const uint8_t = DEFAULT_I2C_ADDRESS);

    void Run() override;


private:

    const uint8_t i2c_address_;

    
};

}

#endif
