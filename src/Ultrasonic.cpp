#include <RBE1001Lib.h>
#include <cmath>

class Ultrasonic
{
private:
    Rangefinder ultra;
    float centiConversion = 2.54f;

public:
    float getDistanceCM()
    {
        return ultra.getDistanceCM();
    }
    float getDistanceIN()
    {
        return ultra.getDistanceCM() * (1 / centiConversion);
    }
};