#include "IMUReader.hpp"

#define LIBCALL_ENABLEINTERRUPT
#include <EnableInterrupt.h>


/**
 *  Interrupt routine.
 */
static volatile bool dmp_data_ready = false;
static int global_fifo_count = 0;               // docasne
static int sinceLastIMUCheck = 0;               // DOCASNE
static void SetDMPDataReady()
{
    dmp_data_ready = true;
}

static uint8_t teapotPacket[14] = {
    '$',
    0x02,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0x00,
    0x00,
    '\r',
    '\n'
};

namespace imu
{

#define COMMUNICATION_FREQUENCY   400000

#define GYRO_SENSITIVITY_SCALE_FACTOR   16.4        // FS Range = 2000°/s
#define ACCEL_SENSITIVITY_SCALE_FACTOR  16384       // FS Range = 2g

}


sensing::imu::IMUReader::IMUReader(const uint8_t interrupt_pin) :
    mpu_(new MPU6050()),
    kInterruptPin_(interrupt_pin),
    dmp_ready_(false),
    is_calibrated_(false),
    packet_size_(0),
    fifo_count_(0)
{
    for (uint8_t i = 0; i < 3; ++i) {
        axes_invert_[i] = false;
    }
}

sensing::imu::IMUReader::IMUReader(const uint8_t interrupt_pin,
                                   bool x_invert,
                                   bool y_invert,
                                   bool z_invert) :
    mpu_(new MPU6050()),
    kInterruptPin_(interrupt_pin),
    dmp_ready_(false),
    is_calibrated_(false),
    packet_size_(0),
    fifo_count_(0)
{
    axes_invert_[0] = x_invert;
    axes_invert_[1] = y_invert;
    axes_invert_[2] = z_invert;
}

sensing::imu::IMUReader::~IMUReader()
{
    disableInterrupt(kInterruptPin_);

    delete mpu_;
}

void sensing::imu::IMUReader::Begin()
{
    Wire.begin();
    Wire.setClock(COMMUNICATION_FREQUENCY);

    mpu_->initialize();

    if (!mpu_->testConnection()) {
        Serial.println(F("MPU6050 connection failed!"));
    }
    delay(500);

    device_status_ = mpu_->dmpInitialize();
    if (device_status_ == 0) {
        dmp_ready_ = true;

        mpu_->CalibrateAccel(6);     // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        mpu_->CalibrateGyro(6);

        mpu_->setDMPEnabled(true);

        pinMode(kInterruptPin_, INPUT);
        enableInterrupt(kInterruptPin_, SetDMPDataReady, RISING);
        mpu_interrupt_status_ = mpu_->getIntStatus();
        packet_size_ = mpu_->dmpGetFIFOPacketSize();
        // mpu_->resetFIFO();
    }
    else {
        dmp_ready_ = false;
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(device_status_);
        Serial.println(F(")"));
    }
}

bool sensing::imu::IMUReader::Calibrate()
{
    // mpu_->CalibrateAccel();
    // mpu_->CalibrateGyro();
    is_calibrated_ = true;

    return is_calibrated_;
}

bool sensing::imu::IMUReader::IsCalibrated() const
{
    return is_calibrated_;
}

void sensing::imu::IMUReader::Update()
{
    if (!dmp_ready_) {
        return;
    }

    // nove, ale cele zle
    // if (mpu_->dmpGetCurrentFIFOPacket(fifo_buffer_)) {
    //     mpu_->dmpGetQuaternion(&quaternion_, fifo_buffer_);
    //     mpu_->dmpGetGravity(&gravity_, &quaternion_);
    //     mpu_->dmpGetYawPitchRoll(ypr_, &quaternion_, &gravity_);
    // }
    // nove, ale cele zle

    // moj pokus - neuspesne=============
    // if (dmp_data_ready) {
    //
    //     fifo_count_ = mpu_->getFIFOCount();
    //
    //     // wait for correct available data length, should be a VERY short wait
    //     while (fifo_count_ < packet_size_) fifo_count_ = mpu_->getFIFOCount();
    //
    //     // read a packet from FIFO
    //     mpu_->getFIFOBytes(fifo_buffer_, packet_size_);
    //
    //     // track FIFO count here in case there is > 1 packet available
    //     // (this lets us immediately read more without waiting for an interrupt)
    //     fifo_count_ -= packet_size_;
    //
    //
    //     mpu_->dmpGetQuaternion(&quaternion_, fifo_buffer_);
    //     mpu_->dmpGetGravity(&gravity_, &quaternion_);
    //     mpu_->dmpGetYawPitchRoll(ypr_, &quaternion_, &gravity_);
    //
    //     dmp_data_ready = false;
    // }
<<<<<<< HEAD
    // */

=======
    // moj pokus - neuspesne=============

// ========================================================================
// netriafa FIFO, ale drzalo stabilitu
    // if (!dmp_data_ready)
    //     return;

// stabilne, FIFO sa neprejavilo
    while (!dmp_data_ready && fifo_count_ < packet_size_) {
        if (dmp_data_ready && fifo_count_ < packet_size_) {
            fifo_count_ = mpu_->getFIFOCount();
        }
    }

    dmp_data_ready = false;
    mpu_interrupt_status_ = mpu_->getIntStatus();

    fifo_count_ = mpu_->getFIFOCount();
    if (fifo_count_ < packet_size_) {

    }
    else if ((mpu_interrupt_status_ & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT))
        || fifo_count_ >= 1024) {
            mpu_->resetFIFO();
    }
    else if (mpu_interrupt_status_ & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
        while (fifo_count_ >= packet_size_) {
            mpu_->getFIFOBytes(fifo_buffer_, packet_size_);
            fifo_count_ -= packet_size_;
        }

        mpu_->dmpGetQuaternion(&quaternion_, fifo_buffer_);
        mpu_->dmpGetGravity(&gravity_, &quaternion_);
        mpu_->dmpGetYawPitchRoll(ypr_, &quaternion_, &gravity_);
    }
    // stabilne, FIFO sa neprejavilo
    // netriafa FIFO, ale drzalo stabilitu
>>>>>>> 49fdd4ec3744fc6ca165ee47661ef0f00ea443ff
    // ========================================================================
}

float sensing::imu::IMUReader::GetXAcceleration() const
{
    float acc_x = mpu_->getAccelerationX();
    return (!axes_invert_[0] ? acc_x : -acc_x);
}

float sensing::imu::IMUReader::GetYAcceleration() const
{
    float acc_y = mpu_->getAccelerationY();
    return (!axes_invert_[1] ? acc_y : -acc_y);
}

float sensing::imu::IMUReader::GetZAcceleration() const
{
    float acc_z = mpu_->getAccelerationZ();
    return (!axes_invert_[2] ? acc_z : -acc_z);
}

float sensing::imu::IMUReader::GetXAngularRate() const
{
    float ang_rate_x = mpu_->getRotationX() / GYRO_SENSITIVITY_SCALE_FACTOR;
    return (!axes_invert_[0] ? ang_rate_x : -ang_rate_x);
}

float sensing::imu::IMUReader::GetYAngularRate() const
{
    float ang_rate_y = mpu_->getRotationY() / GYRO_SENSITIVITY_SCALE_FACTOR;
    return (!axes_invert_[1] ? ang_rate_y : -ang_rate_y);
}

float sensing::imu::IMUReader::GetZAngularRate() const
{
    float ang_rate_z = mpu_->getRotationZ() / GYRO_SENSITIVITY_SCALE_FACTOR;
    return (!axes_invert_[2] ? ang_rate_z : -ang_rate_z);
}

float sensing::imu::IMUReader::GetRollAngularRate() const
{
    return this->GetXAngularRate();
}

float sensing::imu::IMUReader::GetPitchAngularRate() const
{
    return this->GetYAngularRate();
}

float sensing::imu::IMUReader::GetYawAngularRate() const
{
    return this->GetZAngularRate();
}

float sensing::imu::IMUReader::GetXAngle() const
{
    float angle_x = ypr_[2] * 180 / M_PI;
    return (!axes_invert_[0] ? angle_x : -angle_x);
}

float sensing::imu::IMUReader::GetYAngle() const
{
    float angle_y = ypr_[1] * 180 / M_PI;
    return (!axes_invert_[1] ? angle_y : -angle_y);
}

float sensing::imu::IMUReader::GetZAngle() const
{
    float angle_z = ypr_[0] * 180 / M_PI;
    return (!axes_invert_[2] ? angle_z : -angle_z);
}

float sensing::imu::IMUReader::GetRollAngle() const
{
    return this->GetXAngle();
}

float sensing::imu::IMUReader::GetPitchAngle() const
{
    return this->GetYAngle();
}

float sensing::imu::IMUReader::GetYawAngle() const
{
    return this->GetZAngle();
}

float sensing::imu::IMUReader::GetTemperature() const
{
    return mpu_->getTemperature();
}
