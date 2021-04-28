#include <RBE1001Lib.h>

class ArmServo
{
private:
    int servoPin = 33;
    int upPos = 0;
    int downPos = 180;
    int midPos = 70;

    Servo servo;

public:
    void
    attach()
    {
        servo.attach(servoPin);
    }

    void moveDownPosition()
    {
        servo.write(downPos);
    }
    void moveMidPosition()
    {
        servo.write(upPos);
    }
    void moveUpPosition()
    {
        servo.write(downPos);
    }
};