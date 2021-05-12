#include <RBE1001Lib.h>
#include <cmath>

class Ultrasonic
{
private:
    Rangefinder ultra;

    //conversion from cm to in
    const float CENTI_CONV = 2.54f;

public:
    /**
     * attaches the ultrasonic, must call in main
     */
    void attach()
    {
        ultra.attach(SIDE_ULTRASONIC_TRIG, SIDE_ULTRASONIC_ECHO);
    }

    /**
     * returns the distance in centimeters
     */
    float getDistanceCM()
    {
        return ultra.getDistanceCM();
    }

    /**
     * returns the distance in inches
     */
    float getDistanceIN()
    {
        return ultra.getDistanceCM() * (1 / CENTI_CONV);
    }
};