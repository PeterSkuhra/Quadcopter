#include <Arduino.h>
#include <EnableInterrupt.h>

#include "control/IController.hpp"
#include "control/FlightController.hpp"

// Pointer of flight controller
control::IController* flight_controller;

/**
 *  Setup quadcopter.
 */
void setup()
{
    // Begin serial communication
    Serial.begin(115200);

    // Create new instance of flight controller
    flight_controller = new control::FlightController();

    // Initializes flight controller
    flight_controller->Init();
}

/**
 *  Main loop of quadcopter.
 */
void loop()
{
    // Main control loop of flight controller
    flight_controller->Control();
}
