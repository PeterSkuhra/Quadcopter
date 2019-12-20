#include <Arduino.h>

#include "Multicopter.hpp"
#include "motor/PWM.hpp"

IExecutable* multicopter;
PWM* pwm;

void setup()
{
    multicopter = Multicopter::GetInstance();
    pwm = PWM::GetInstance(11);
}

void loop()
{
    multicopter->Run();
}
