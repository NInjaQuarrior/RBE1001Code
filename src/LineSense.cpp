#include <RBE1001Lib.h>

class LineSense
{

private:
    ESP32AnalogRead leftSensor;

    ESP32AnalogRead rightSensor;

    //port for left sensor
    const int leftPort = 36;
    //port for right sensor
    const int rightPort = 39;

public:
    /**
    *  attaches the line sensor. Must call in main class
    */
    void attach()
    {
        leftSensor.attach(leftPort);
        rightSensor.attach(rightPort);
    }

    /**
     * returns the left sensor voltage, less is more light
     */
    float getLeft()
    {
        return leftSensor.readVoltage();
    }

    /**
     * returns the right sensor voltage, less is more light
     */
    float getRight()
    {
        return rightSensor.readVoltage();
    }

    /**
     * returns the differance between the right and left sensors
     */
    float getDifference()
    {
        return getRight() - getLeft();
    }
};