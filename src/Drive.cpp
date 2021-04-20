#include <RBE1001Lib.h>
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

    float lineBaseSpeed = .2f;
    float lineProp = .08f;

    float lineFollowTurnDead = .5f;

    float lineSenseBlack = 2.0f;

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

    boolean isTurning = false;
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
};
