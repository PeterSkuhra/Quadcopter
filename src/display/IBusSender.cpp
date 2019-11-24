#include "IBusSender.hpp"

#include <Arduino.h>

using namespace display;

IBusSender::IBusSender(HardwareSerial& serial)
{
    ibus_.begin(serial);

    battery_voltage_.number = ibus_.addSensor(IBUSS_INTV);
    rpm_.number = ibus_.addSensor(IBUSS_RPM);
    temperature_.number = ibus_.addSensor(IBUSS_TEMP);
}

void IBusSender::WriteBatteryVoltage(uint8_t voltage)
{
    const uint8_t kVoltageMultipier = 100;

    WriteValue(battery_voltage_, voltage * kVoltageMultipier);
}

void IBusSender::WriteRPM(uint16_t rpm)
{
    WriteValue(rpm_, rpm);
}

void IBusSender::WriteTempetrature(int8_t temperature)
{
    const int16_t kTemperatureBase = 400;

    WriteValue(temperature_, temperature + kTemperatureBase);
}

void IBusSender::WriteValue(Sensor& sensor, int16_t value)
{
    sensor.value = value;
    ibus_.setSensorMeasurement(sensor.number, sensor.value);
}
