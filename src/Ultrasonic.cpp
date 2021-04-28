#include <RBE1001Lib.h>
#include <cmath>

class Ultrasonic
{
private:
    Rangefinder ultra;
    float centiConversion = 2.54f;
    float distanceFromLast = 0;

public:
    float getDistanceCM()
    {
        return ultra.getDistanceCM();
    }
    float getDistanceIN()
    {
        return ultra.getDistanceCM() * (1 / centiConversion);
    }

    void attach()
    {
        ultra.attach(SIDE_ULTRASONIC_TRIG, SIDE_ULTRASONIC_ECHO);
    }

    float distanceFromLastCheck()
    {
        int dist = fabs(distanceFromLast - getDistanceIN());
        distanceFromLast = getDistanceIN();
        return dist;
    }
};