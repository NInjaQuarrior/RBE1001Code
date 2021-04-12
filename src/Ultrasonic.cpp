#include <RBE1001Lib.h>
#include <cmath>

class Ultrasonic
{
private:
    Rangefinder ultra;
    float centiConversion = 0.3937008;

public:
    float getDistanceCM()
    {
        return ultra.getDistanceCM();
    }
    float getDistanceIN()
    {
        return ultra.getDistanceCM() * centiConversion;
    }
};