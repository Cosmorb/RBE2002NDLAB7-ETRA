#pragma once

#include <Arduino.h>

/** \class Rangefinder 
 * \brief Virtual base class to manage rangefinders. 
 * 
 * */
class Rangefinder 
{
protected:

public:
    Rangefinder(void) {}

    /** 
     * \brief Returns true when there is a new reading and false otherwise.
     * The distance will be stored in distance.
     * */
    virtual bool getDistance(float& distance) = 0;
};
