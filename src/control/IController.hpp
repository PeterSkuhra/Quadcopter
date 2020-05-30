#ifndef ICONTROLLER_HPP
#define ICONTROLLER_HPP

namespace control
{

/******************************************************************************
 *  Controller interface. Defines methods for the universal controller.
 *****************************************************************************/
class IController
{
public:

    /**
     *  Default destructor.
     */
    virtual ~IController() = default;

    /**
     *  Initializes controller.
     */
    virtual void Init() = 0;

    /**
     *  Control loop.
     */
    virtual void Control() = 0;
};

}

#endif
