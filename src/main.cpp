#include <Arduino.h>

#include "Multicopter.hpp"


IExecutable* multicopter;

void setup()
{
    multicopter = Multicopter::GetInstance();
}

void loop()
{
    multicopter->Run();
}
