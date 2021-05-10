#include <RBE1001Lib.h>

class ArmServo
{
private:
    Servo servo;

    const int servoPin = 33;

    //arm in up pos
    const int upPos = 180;
    //arm in down pos
    const int downPos = 0;
    //arm parallel to ground
    const int midPos = 65;

public:
    /**
 * attachs the servo, must be called in main class
 */
    void attach()
    {
        servo.attach(servoPin);
    }

    /**
 * moves the arm to down to the ground
 */
    void moveDownPosition()
    {
        servo.write(downPos);
    }

    /**
 * moves the arm to parrallel to the ground
 */
    void moveMidPosition()
    {
        servo.write(midPos);
    }

    /**
 * moves the arm to up to the ground
 */
    void moveUpPosition()
    {
        servo.write(upPos);
    }
};