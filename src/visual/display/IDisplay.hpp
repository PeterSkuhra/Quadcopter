#ifndef IDISPLAY_HPP
#define IDISPLAY_HPP

#include <Arduino.h>

namespace visual
{

namespace display
{

class IDisplay
{
 public:

     virtual ~IDisplay() = default;

     virtual void Begin() = 0;

     virtual void Clear() = 0;

     virtual void Print(const char* text) = 0;

};

}
}

#endif
