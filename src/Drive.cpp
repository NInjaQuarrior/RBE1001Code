#include <RBE1001Lib.h>
#include <RemoteConstants.h>
#define PI 3.14159265358979323846

class Drive
{
private:
    LeftMotor left;
    RightMotor right;

    //constant wheel diameter
    float WHEEL_DIAMETER = 2.75f;
    float centiConversion = 2.54f;

    float ultraProp = .07f;
    float ultraDead = .35f;

    float findBagDead = 20;

    float lineBaseSpeed = .2f;
    float lineProp = .08f;

    float lineFollowTurnDead = .5f;

    float lineSenseBlack = 2.0f;

    //code stuffs
    boolean isTurning = false;

    enum ScanState
    {
        INIT,
        SCANNING,
        TURN_TO,
        DRIVE,
        DONE
    };
    ScanState scanState = INIT;

public:
    /**
 * turns a certain amount of degrees
 * @param degrees degrees to turn, negative to turn counter-clockwise
 * @param speed degrees per second to move
 * 
 * */
    boolean
    turn(float degrees, float speed)
    {

        float moveDegrees = 2 * degrees;
        left.startMoveFor(moveDegrees, speed);
        right.moveFor(-moveDegrees, speed);

        return true;
    }

    /**
 * drive straight a certain amount of inches
 * @param inches inches to move, negative to go backwars
 * @param speed degrees per second to move
 * 
 * */
    boolean driveInches(float inches, float speed)
    {
        float moveDegrees = (inches / (2 * PI * (WHEEL_DIAMETER / 2))) * 360;

        left.startMoveFor(moveDegrees, speed);
        right.moveFor(moveDegrees, speed);

        return true;
    }

    /**
 * drive straight a certain amount of inches
 * @param centi centimeters to move, negative to go backwars
 * @param speed degrees per second to move
 * 
 * */
    boolean driveCentimeters(float centi, float speed)
    {
        float moveDegrees = (centi / (2 * PI * ((WHEEL_DIAMETER / centiConversion) / 2))) * 360;

        left.startMoveFor(moveDegrees, speed);
        right.moveFor(moveDegrees, speed);

        return true;
    }

    /**
 * drive based on effort
 * @param effort -1  to 1
 * 
 * */
    void setEffort(float effort)
    {
        left.setEffort(effort);
        right.setEffort(effort);
    }

    /**
 * drive based on effort
 * @param speed in degrees per second
 * 
 * */
    void setSpeed(float speed)
    {
        left.setSpeed(speed);
        right.setSpeed(speed);
    }

    float lastEffort = 0;
    /**
 * @param button
 * 
 * */
    void
    setButton(uint16_t button)
    {
        if (button == remoteLeft)
        {
            turn(-90, 270);
        }
        else if (button == remoteRight)
        {
            turn(90, 270);
        }

        switch (button)
        {
        case remote1:
            left.setEffort(.1);
            right.setEffort(.1);
            lastEffort = .1;
            break;
        case remote2:
            left.setEffort(.2);
            right.setEffort(.2);
            lastEffort = .2;
            break;
        case remote3:
            left.setEffort(.3);
            right.setEffort(.3);
            lastEffort = .3;
            break;
        case remote4:
            left.setEffort(.4);
            right.setEffort(.4);
            lastEffort = .4;
            break;
        case remote5:
            left.setEffort(.5);
            right.setEffort(.5);
            lastEffort = .5;
            break;
        case remote6:
            left.setEffort(.6);
            right.setEffort(.6);
            lastEffort = .6;
            break;
        case remote7:
            left.setEffort(.7);
            right.setEffort(.7);
            lastEffort = .7;
            break;
        case remote8:
            left.setEffort(.8);
            right.setEffort(.8);
            lastEffort = .8;
            break;
        case remote9:
            left.setEffort(.9);
            right.setEffort(.9);
            lastEffort = .9;
            break;
        case DEFAULT:
            break;
        }
        left.setEffort(lastEffort);
        right.setEffort(lastEffort);
    }

    /**
 * Makes a shape
 * @param sides numbers of side for the shape
 * @param sideLength  the length of each side in inches
 * @param speed degrees per second to move while making shape
 * 
 * */
    boolean makeShape(int sides, float sideLength, float speed)
    {
        float turnDegrees = 360 / sides;
        for (int i = 0; i < sides; i++)
        {
            driveInches(sideLength, speed);
            turn(turnDegrees, speed);
        }
        return true;
    }

    /**
 * makes a spiral shape in the shape of the side put in, ie 4 side for square spiral
 * 
 * @param baseLength the length of the first side in inches
 * @param speed, degrees per second to move
 * @param sides number of sides before completing on spiral
 * @param spiralAmount number of complete spirals to make
 * 
 * */
    boolean makeSpiral(float baseLength, float speed, int sides, int spiralAmount)
    {
        float turnDegrees = 360 / sides;
        for (int i = 0; i < (sides * spiralAmount); i++)
        {
            driveInches(baseLength + (i * baseLength), speed);
            turn(turnDegrees, speed);
        }
        return true;
    }

    boolean driveToInches(float inches, float distanceIN)
    {

        float offset = distanceIN - inches;
        float effort = offset * ultraProp;

        if (distanceIN > inches - ultraDead && distanceIN < inches + ultraDead)
        {
            setEffort(0);
            return true;
        }
        setEffort(effort);
        return false;
    }

    void followLine(float error, float leftSense, float rightSense)
    {
        if (leftSense > lineSenseBlack && rightSense > lineSenseBlack /*&& error < lineFollowTurnDead*/)
        {
            isTurning = true;
            if (turn(180, 360))
            {
                isTurning = false;
            }
        }
        else if (isTurning == false)
        {

            left.setEffort(lineBaseSpeed + (error * lineProp));
            right.setEffort(lineBaseSpeed - (error * lineProp));
        }
    }

    int bagStartAngle = 0;
    int bagEndAngle = 0;

    boolean findBag(float distLast, float curDist)
    {
        switch (scanState)
        {
        case INIT:
            bagStartAngle = 0;
            bagEndAngle = 0;

            if (turn(-45, 360))
            {
                scanState = SCANNING;
            }
            break;
        case SCANNING:
            for (int i = 1; i <= 90; i++)
            {
                turn(1, 270);
                if (distLast > findBagDead && bagStartAngle == 0)
                {
                    bagStartAngle = i;
                }
                else if (distLast > findBagDead)
                {
                    bagEndAngle = i;
                }
            }
            scanState = TURN_TO;
            break;
        case TURN_TO:
            if (turn(-(bagEndAngle - bagStartAngle), 180))
            {
                scanState = DRIVE;
            }
            break;
        case DRIVE:
            if (driveToInches(5, curDist))
            {
                scanState = DONE;
            }
            break;

        case DONE:
            bagStartAngle = 0;
            bagEndAngle = 0;

            break;
        }
    }
};
