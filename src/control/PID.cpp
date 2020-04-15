#include "PID.hpp"


control::PID::PID(float p_gain, float i_gain, float d_gain, float out_limit) :
    output_limit_(out_limit),
    prev_time_(0)
{
    gains_.p = p_gain;
    gains_.i = i_gain;
    gains_.d = d_gain;
}

float control::PID::Update(float setpoint, float process)
{
    float pid_output;
    const float kError = process - setpoint;
    const float kPrevError = prev_process_ - prev_setpoint_;

    uint32_t current_time = millis();
    uint32_t elapsed_time = current_time - prev_time_;

    outputs_.p = kError * gains_.p;
    outputs_.i = (outputs_.i + (kError * elapsed_time)) * gains_.i;
    outputs_.d = ((kError - kPrevError) / elapsed_time) * gains_.d;

    prev_setpoint_ = setpoint;
    prev_process_ = process;

    prev_time_ = current_time;

    pid_output = outputs_.p + outputs_.i + outputs_.d;

    return constrain(pid_output, -output_limit_, output_limit_);
}

void control::PID::SetPIDGains(float p_gain, float i_gain, float d_gain)
{
    gains_.p = p_gain;
    gains_.i = i_gain;
    gains_.d = d_gain;
}

void control::PID::SetOutputLimit(float limit)
{
    output_limit_ = limit;
}

float control::PID::GetOutputLimit() const
{
    return output_limit_;
}
