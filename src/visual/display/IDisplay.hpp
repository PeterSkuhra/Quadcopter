#ifndef IDISPLAY_HPP
#define IDISPLAY_HPP

#include <Arduino.h>

namespace visual
{

namespace display
{

/******************************************************************************
 *  Interface for simple display to printing text.
 *****************************************************************************/
class IDisplay
{
 public:

     /**
      * Default destructor.
      */
     virtual ~IDisplay() = default;

     /**
      * Initializes display.
      */
     virtual void Begin() = 0;

     /**
      * Clear the whole display.
      */
     virtual void Clear() = 0;

     /**
      * Prints the entered text on the display.
      *
      * @param text     text to printig
      */
     virtual void Print(const char* text) = 0;
};

}
}

#endif
