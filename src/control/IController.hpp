#ifndef ICONTROLLER_HPP
#define ICONTROLLER_HPP

namespace control
{

class IController
{
public:
    virtual ~IController() = default;

    virtual void Init() = 0;

    virtual void Control() = 0;
};

}

#endif
