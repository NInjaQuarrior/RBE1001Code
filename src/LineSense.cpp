#include <RBE1001Lib.h>

class LineSense
{

private:
    ESP32AnalogRead leftSensor;
    ESP32AnalogRead rightSensor;
    int leftPort = 36;
    int rightPort = 39;

public:
    void attach()
    {
        leftSensor.attach(leftPort);
        rightSensor.attach(rightPort);
    }

    float getLeft()
    {
        return leftSensor.readVoltage();
    }

    float getRight()
    {
        return rightSensor.readVoltage();
    }
    float getDifference()
    {
        return getRight() - getLeft();
    }
};