#include "FlightController.hpp"

#include "../DevicesWiring.hpp"

#include "command/FlySkyReceiver.hpp"
#include "sensing/imu/IMUReader.hpp"
#include "sensing/voltage/VoltageSensor.hpp"
#include "esc/ESC30A.hpp"

using namespace wiring;

#define MIN_PULSE_US   1000
#define MAX_PULSE_US   1990

#define MAX_PITCH_ANGLE         25  //  degrees [°]
#define MAX_ROLL_ANGLE          25  //  degrees [°]
#define MAX_YAW_AGULAR_RATE     90  //  degrees per second [°/s]


control::FlightController::FlightController() :
    init_(false)
{
    voltage_sensor_ =
        new sensing::voltage::VoltageSensor(BATTERY_VOLTAGE_SENSOR_PIN,
                                            BATTERY_VOLTAGE_SENSOR_RESOLUTION);

    imu_ = new sensing::imu::IMUReader(MPU6050_INT_PIN, false, true, true);

    receiver_ = new command::FlySkyReceiver(wiring::kReceiverPins);

    esc_manager_ = new esc::ESCManager(wiring::kESCPins, *voltage_sensor_);

    motors_speeds_.clear();
    motors_speeds_.resize(MOTOR_COUNT);

    this->InitPID();
}

control::FlightController::~FlightController()
{
    delete voltage_sensor_;
    delete imu_;
    delete receiver_;

    delete filter_.thrust;
    delete filter_.yaw;
    delete filter_.pitch;
    delete filter_.roll;

    delete pid_controller_.thrust;
    delete pid_controller_.yaw;
    delete pid_controller_.pitch;
    delete pid_controller_.roll;

    delete esc_manager_;
}

void control::FlightController::Init()
{
    if (!init_) {

        pinMode(LED_ORANGE_PIN, OUTPUT);
        pinMode(LED_GREEN_PIN, OUTPUT);
        digitalWrite(LED_ORANGE_PIN, HIGH);

        // ESC's
        delay(500);
        if (receiver_->ReadChannel(10) < 1200 && receiver_->ReadChannel(10) > 900) {
            Serial.println(F("\nCalibrating all ESC..."));

            if (esc_manager_->Calibrate()) {
                Serial.println(F("\nCalibration all ESC OK :)"));
            }
            else {
                Serial.println(F("\nCalibration ESC NOK :("));
            }
        }

        while (receiver_->ReadChannel(10) < 1200) {
            delay(10);
        }

        // IMU
        imu_->Begin();

        Serial.println(F("\nCalibrating imu..."));
        if (imu_->Calibrate()) {
            Serial.println(F("\nCalibration imu OK :)"));
        }
        else {
            Serial.println(F("\nCalibration imu NOK :("));
            //while(true);

        }
        delay(100);

        // Receiver
        Serial.println(F("\nCalibrating receiver..."));
        if (receiver_->Calibrate()) {
            Serial.println(F("\nCalibration rec OK :)"));
        }
        else {
            Serial.println(F("\nCalibration rec NOK :("));
            while(true);
        }


        this->InitFilter();

        digitalWrite(LED_ORANGE_PIN, LOW);
        digitalWrite(LED_GREEN_PIN, HIGH);

        init_ = true;
    }
}

void control::FlightController::InitPID()
{
    const float kPGainYaw = 3.0;
    const float kIGainYaw = 0.02;
    const float kDGainYaw = 0.0;
    const float kLimitYaw = 250;

    const float kPGainPitch = 0.0;
    const float kIGainPitch = 0.0;
    const float kDGainPitch = 10.0;
    const float kLimitPitch = 250;

    const float kPGainRoll = 0.0;
    const float kIGainRoll = 0.0;
    const float kDGainRoll = 10.0;
    const float kLimitRoll = 250;

    pid_controller_.yaw = new PID(kPGainYaw,
                                 kIGainYaw,
                                 kDGainYaw,
                                 kLimitYaw);

    pid_controller_.pitch = new PID(kPGainPitch,
                                   kIGainPitch,
                                   kDGainPitch,
                                   kLimitPitch);

    pid_controller_.roll = new PID(kPGainRoll,
                                  kIGainRoll,
                                  kDGainRoll,
                                  kLimitRoll);
}

void control::FlightController::Control()
{
    this->ReadReceiverData();

    this->MapReceiverData();

    this->ReadIMUData();

    this->PIDCalculation();

    this->CalculateMotorsSpeeds();

    this->WriteMotorsSpeeds();
}

void control::FlightController::InitFilter()
{
    receiver_data_.thrust = receiver_->ReadChannel(1);
    receiver_data_.yaw    = receiver_->ReadChannel(2);
    receiver_data_.pitch  = receiver_->ReadChannel(3);
    receiver_data_.roll   = receiver_->ReadChannel(4);

    const uint8_t kFilterWeight = 10;

    filter_.thrust = new ExponentialFilter<float>(kFilterWeight, receiver_data_.thrust);
    filter_.yaw    = new ExponentialFilter<float>(kFilterWeight, receiver_data_.yaw);
    filter_.pitch  = new ExponentialFilter<float>(kFilterWeight, receiver_data_.pitch);
    filter_.roll   = new ExponentialFilter<float>(kFilterWeight, receiver_data_.roll);
}

void control::FlightController::ReadReceiverData()
{
    // receiver_data_.thrust = receiver_->ReadChannel(1);
    // receiver_data_.yaw = receiver_->ReadChannel(2);
    // receiver_data_.pitch = receiver_->ReadChannel(3);
    // receiver_data_.roll = receiver_->ReadChannel(4);

    // Serial.print("1: ");
    // Serial.print(receiver_->ReadChannel(1));
    // Serial.print("  2: ");
    // Serial.print(receiver_->ReadChannel(2));
    // Serial.print("  3: ");
    // Serial.print(receiver_->ReadChannel(3));
    // Serial.print("  4: ");
    // Serial.println(receiver_->ReadChannel(4));

    receiver_data_.thrust = filter_.thrust->Filter(receiver_->ReadChannel(1));
    receiver_data_.yaw    = filter_.yaw->Filter(receiver_->ReadChannel(2));
    receiver_data_.pitch  = filter_.pitch->Filter(receiver_->ReadChannel(3));
    receiver_data_.roll   = filter_.roll->Filter(receiver_->ReadChannel(4));

    // Serial.print("Thr: ");
    // Serial.print(receiver_data_.thrust);
    // Serial.print("  Yaw: ");
    // Serial.print(receiver_data_.yaw);
    // Serial.print("  Pit: ");
    // Serial.print(receiver_data_.pitch);
    // Serial.print("  Rol: ");
    // Serial.println(receiver_data_.roll);

    // Serial.println(receiver_data_.thrust);
    // Serial.print(" ");
    // Serial.println((int32_t)filter_typr_[0]->Filter(receiver_data_.thrust));
}

void control::FlightController::MapReceiverData()
{
    receiver_data_.yaw = map(receiver_data_.yaw,
                             MIN_PULSE_US,
                             MAX_PULSE_US,
                             -MAX_YAW_AGULAR_RATE,
                             MAX_YAW_AGULAR_RATE);

    receiver_data_.pitch = map(receiver_data_.pitch,
                               MIN_PULSE_US,
                               MAX_PULSE_US,
                               -MAX_PITCH_ANGLE,
                               MAX_PITCH_ANGLE);

    receiver_data_.roll   = map(receiver_data_.roll,
                                MIN_PULSE_US,
                                MAX_PULSE_US,
                                -MAX_ROLL_ANGLE,
                                MAX_ROLL_ANGLE);
}

void control::FlightController::ReadIMUData()
{
    imu_->Update();

    imu_data_.yaw = imu_->GetZAngularRate();
    imu_data_.pitch = imu_->GetPitchAngle();
    imu_data_.roll = imu_->GetRollAngle();

    // // TEST without mpu update
    // delay(12);
    // imu_data_.yaw = 0;
    // imu_data_.pitch = 0;
    // imu_data_.roll = 0;

    // Serial.print("YAW: "); Serial.print(imu_data_.yaw);
    // Serial.print("   PITCH: "); Serial.print(imu_data_.pitch);
    // Serial.print("   ROLL: "); Serial.println(imu_data_.roll);
}

void control::FlightController::PIDCalculation()
{
    pid_data_.yaw = pid_controller_.yaw->Update(receiver_data_.yaw, imu_data_.yaw);
    pid_data_.pitch = pid_controller_.pitch->Update(receiver_data_.pitch, imu_data_.pitch);
    pid_data_.roll = pid_controller_.roll->Update(receiver_data_.roll, imu_data_.roll);

    // Serial.print(receiver_data_.roll);
    // Serial.print(" ");
    // Serial.print(imu_data_.roll);
    // Serial.print(" ");
    // Serial.println(pid_data_.roll);

    // Serial.print(receiver_data_.thrust); Serial.print(" ");
    // Serial.print(pid_data_.yaw); Serial.print(" ");
    // Serial.print(pid_data_.pitch); Serial.print(" ");
    // Serial.print(pid_data_.roll); Serial.println(" ");
}

void control::FlightController::CalculateMotorsSpeeds()
{
    // Front right
    motors_speeds_[FRONT_RIGHT] = receiver_data_.thrust + pid_data_.yaw +
                       pid_data_.pitch + pid_data_.roll;
    // Front left
    motors_speeds_[FRONT_LEFT] = receiver_data_.thrust - pid_data_.yaw +
                       pid_data_.pitch - pid_data_.roll;

    // Back right
    motors_speeds_[BACK_RIGHT] = receiver_data_.thrust - pid_data_.yaw -
                       pid_data_.pitch + pid_data_.roll;

    // Back left
    motors_speeds_[BACK_LEFT] = receiver_data_.thrust + pid_data_.yaw -
                       pid_data_.pitch - pid_data_.roll;


    // // Serial.print(F("FR: "));
    // Serial.print("FR: ");
    // Serial.print(motors_speeds_[FRONT_RIGHT]);
    //
    // // Serial.print(F("  FL: "));
    // Serial.print("  FL: ");
    // Serial.print(motors_speeds_[FRONT_LEFT]);
    //
    // // Serial.print(F("  BR: "));
    // Serial.print("  BR: ");
    // Serial.print(motors_speeds_[BACK_RIGHT]);
    //
    // // Serial.print(F("  BL: "));
    // Serial.print("  BL: ");
    // Serial.println(motors_speeds_[BACK_LEFT]);
    // Serial.println();
}

void control::FlightController::WriteMotorsSpeeds()
{
    esc_manager_->SetSpeeds(motors_speeds_);
}
