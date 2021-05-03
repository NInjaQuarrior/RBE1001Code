#include <RBE1001Lib.h>
#include <cmath>

class Ultrasonic
{
private:
    Rangefinder ultra;

    //conversion from cm to in
    float CENTI_CONV = 2.54f;

public:
    //attaches the ultrasonic, must call in main
    void attach()
    {
        ultra.attach(SIDE_ULTRASONIC_TRIG, SIDE_ULTRASONIC_ECHO);
    }
    //returns the distance in centimeters
    float getDistanceCM()
    {
        return ultra.getDistanceCM();
    }
    //returns the distanc in inches
    float getDistanceIN()
    {
        return ultra.getDistanceCM() * (1 / CENTI_CONV);
    }

    float distanceFromLast = 0;
    //gets the distance from the last time the ultrasonic was checked
    float distanceFromLastCheck()
    {
        int dist = fabs(distanceFromLast - getDistanceIN());
        distanceFromLast = getDistanceIN();
        return dist;
    }
};