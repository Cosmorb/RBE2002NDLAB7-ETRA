#include <Sharp-IR.h>

/**
 * 500 is arbitrarily long -- check the data sheet for a reasonable value.
 * 
 * Alternatively, only return something when the value changes.
 */
const uint32_t SHARP_IR_INTERVAL = 500; 

bool SharpIR::getDistance(float& distance)
{
    bool newReading = false;
    uint32_t currTime = millis();
    if(currTime - lastReading >= SHARP_IR_INTERVAL)
    {
        lastReading = currTime;

        uint16_t adcResult = analogRead(adcPin);
        Serial.print(adcResult);
        Serial.print('\t');

        /**
         * TODO: Convert to a distance.
         */
        distance = adcResult; 

#ifdef __SHARP_DEBUG__
        Serial.println(distance);
#endif

        newReading = true;
    }

    return newReading;
}
