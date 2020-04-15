#ifndef ICALIBRATABLE_HPP
#define ICALIBRATABLE_HPP


class ICalibratable
{
public:

    virtual ~ICalibratable() = default;

    virtual bool Calibrate() = 0;

    virtual bool IsCalibrated() const = 0;
};

#endif
