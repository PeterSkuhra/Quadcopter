#ifndef VECTOR_HPP
#define VECTOR_HPP

namespace sensing
{
namespace imu
{

template<class T>
class Vector
{
 public:
    T x;
    T y;
    T z;

 public:
    Vector() :
        x(0),
        y(0),
        z(0)
    {}

    Vector(T x, T y, T z) :
        x(x),
        y(y),
        z(z)
    {}

    T GetMagnitude() const
    {
        return sqrt(x * x + y * y + z * z);
    }
};

}
}

#endif
