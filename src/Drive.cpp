#include <RBE1001Lib.h>
#include <RemoteConstants.h>
//#define PI 3.14159265358979323846

class Drive
{
private:
    LeftMotor left;
    RightMotor right;

    //constantss ======================================
    float WHEEL_DIAMETER = 2.75f;

    //centi to inch conversion
    float CENTI_CONV = 2.54f;

    //Kp for the ultra moving
    float ULTRA_PROP = .07f;

    //deadband for moving to a distance with ultra
    float ULTRA_DEAD = .35f;

    //speed while using ultra
    float ULTRA_DRIVE = 90;

    //deadband for finding an object(bag) with thge ultra
    float FIND_BAG_DEAD = 5;

    //base follow line speed
    float LINE_BASE_SPEED = .2f;

    //Kp for following the line
    float LINE_PROP = .1f;

    //voltage value for determining if a sensor is over the line
    float LINE_SENSE_BLACK = 1.6f;

    //angle to turn before looking for object(bag)
    float Turn_SET_UP_ANGLE = 20.0f;

    //turn speed in degrees per second
    float TURN_SPEED = 270.0f;

    //angle to scan while looking for object
    float SCAN_ANGLE = 150.0f;

    //speed to turn while scanning for object in degrees per second
    float SCAN_SPEED = 270.0f;

    //distance to stop away from bag to pick it up
    float DIST_FROM_BAG = 2.5f; //TODO tune

    //speed to drive in degrees per second
    float DRIVE_SPEED = 270.0f;

    //max distance that the ultra will care about while scanning for a bag
    float MAX_DIST = 30;

    //end constants=========================================

    //for scanning for object(bag)
    enum ScanState
    {
        INIT_SCAN,
        SCANNING,
        TURN_TO,
        DRIVE_SCAN,
        DONE_SCAN
    };
    ScanState scanState = INIT_SCAN;

    //for returning to line from picking up bag from free zone
    enum ReturnState
    {
        TURN_RETURN,
        TURN_TWO,
        DRIVE_RETURN,
        DRIVE_TO_LINE,
        DONE_RETURN
    };

    ReturnState returnState = TURN_RETURN;

public:
    /**
    * turns a certain amount of degrees
    * @param degrees degrees to turn, negative to turn counter-clockwise
    * @param speed degrees per second to move
    * 
    */
    boolean turn(float degrees, float speed)
    {

        float moveDegrees = 2 * degrees;
        left.startMoveFor(moveDegrees, speed);
        right.moveFor(-moveDegrees, speed);

        return true;
    }

    /**
     * turn until not called
     * @param direct -1 for left, 1 for right
     * @param speed in degrees for second
     */
    void turnContinuos(int direct, float speed)
    {
        if (direct <= 0)
        {
            left.setSpeed(-speed);
            right.setSpeed(speed);
        }
        else if (direct >= 0)
        {
            left.setSpeed(speed);
            right.setSpeed(-speed);
        }
    }

    /**
    * drive straight a certain amount of inches
    * @param inches inches to move, negative to go backwars
    * @param speed degrees per second to move
    * 
    */
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
    */
    boolean driveCentimeters(float centi, float speed)
    {
        float moveDegrees = (centi / (2 * PI * ((WHEEL_DIAMETER / CENTI_CONV) / 2))) * 360;

        left.startMoveFor(moveDegrees, speed);
        right.moveFor(moveDegrees, speed);

        return true;
    }

    /**
    * drive based on effort
    * @param effort -1  to 1
    */
    void setEffort(float effort)
    {
        left.setEffort(effort);
        right.setEffort(effort);
    }

    /**
    * drive based on effort
    * @param speed in degrees per second  
    */
    void setSpeed(float speed)
    {
        left.setSpeed(speed);
        right.setSpeed(speed);
    }

    float lastEffort = 0;

    /**
    * uses remote to control robot, numbers scale speed by .1, arrows turn 90 degrees
    * @param button
    */
    void teleOp(uint16_t button)
    {
        if (button == remoteLeft)
        {
            turn(-90, TURN_SPEED);
        }
        else if (button == remoteRight)
        {
            turn(90, TURN_SPEED);
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
    */
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
    */
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

    /**
    * drives to a set distance away from a target using the ultra sonic
    * @param inches distance to move to
    * @param distance the ultrasonic getDistanceIN
    */
    boolean driveToInches(float inches, float distanceIN)
    {

        //float offset = distanceIN - inches;
        //float effort = offset * ULTRA_PROP; // for p control

        if (distanceIN > inches - ULTRA_DEAD && distanceIN < inches + ULTRA_DEAD)
        {
            setEffort(0);
            return true;
        }
        setSpeed(ULTRA_DRIVE);
        return false;
    }

    /**
     * follows the black line using p control
     * @param error the currect difference between the two line sensors
     * @param leftSense the current value of left Sensor
     * @param rightSense current value of the right sensor
     */
    void followLine(float error, float leftSense, float rightSense)
    {
        left.setEffort(LINE_BASE_SPEED + (error * LINE_PROP));
        right.setEffort(LINE_BASE_SPEED - (error * LINE_PROP));
    }

    /**
     * drive until find a line
     * @param speed the speed in degrees per second
     * @param leftSense the current value of left Sensor
     * @param rightSense current value of the right sensor
     */
    boolean driveTillLine(float speed, float leftSense, float rightSense)
    {
        if (leftSense > LINE_SENSE_BLACK || rightSense > LINE_SENSE_BLACK /*&& error < lineFollowTurnDead*/)
        {
            return true;
        }
        setSpeed(speed);
        return false;
    }
    /**
     * line follow until find a t intersection
     * @param speed the speed in degrees per second
     * @param leftSense the current value of left Sensor
     * @param rightSense current value of the right sensor
     */
    boolean lineFollowTillLine(float speed, float leftSense, float rightSense, float error)
    {
        if (leftSense > LINE_SENSE_BLACK || rightSense > LINE_SENSE_BLACK /*&& error < lineFollowTurnDead*/)
        {
            return true;
        }
        followLine(error, leftSense, rightSense);
        return false;
    }

    /**
     * turn until find a line
     * 
    * @param direct -1 for left, 1 for right
    * @param leftSense the current value of left Sensor
    * @param rightSense current value of the right sensor
    */
    boolean alignToLine(int direct, float leftSense, float rightSense)
    {
        if (direct < 0)
        {
            if (rightSense > LINE_SENSE_BLACK - 1)
            {
                left.setSpeed(0);
                right.setSpeed(0);
                return true;
            }
        }
        else if (direct >= 0)
        {
            if (leftSense > LINE_SENSE_BLACK - 1)
            {
                left.setSpeed(0);
                right.setSpeed(0);
                return true;
            }
        }

        turnContinuos(direct, 90);
        return false;
    }

    //store bag start angle
    int bagStartAngle = 0;
    //store bag end angle
    int bagEndAngle = 0;
    //store the angle of the center of the bag
    float bagCenter = 0;
    //store previous ultra distance
    float prevDist = 0;
    //counter for finding bag angle
    int counter = 1;

    /**
     * starting on the line, search for the bag, and drive to it prepared to pick it up
     * @param curDist the ultrasonic get distance in inches
     */
    boolean findBag(float curDist)
    {
        switch (scanState)
        {
        case INIT_SCAN:
            bagStartAngle = 0;
            bagEndAngle = 0;

            if (turn(Turn_SET_UP_ANGLE, TURN_SPEED))
            {
                prevDist = curDist;
                scanState = SCANNING;
            }
            break;
        case SCANNING:

            if (counter <= SCAN_ANGLE)
            {
                turn(1, SCAN_SPEED);
                if (prevDist > curDist && bagStartAngle == 0 && (prevDist - curDist) > FIND_BAG_DEAD && curDist < MAX_DIST)
                {

                    prevDist = curDist;
                    bagStartAngle = counter;
                }
                else if (bagStartAngle != 0 && prevDist < curDist && (curDist - prevDist) > FIND_BAG_DEAD)
                {
                    bagEndAngle = counter;
                    scanState = TURN_TO;
                }
                counter++;
            }
            break;
        case TURN_TO:
            bagCenter = bagEndAngle - bagStartAngle;
            if (turn(-(bagCenter / 2 - 3), 270))
            {
                scanState = DRIVE_SCAN;
            }
            break;
        case DRIVE_SCAN:
            if (driveToInches(DIST_FROM_BAG, curDist))
            {
                scanState = DONE_SCAN;
            }
            break;

        case DONE_SCAN:
            bagStartAngle = 0;
            bagEndAngle = 0;
            scanState = INIT_SCAN;
            return true;
            break;
        }
        return false;
    }
    /**
     * returns from the free zone after picking up the bag
     * @param leftSense left light sensor
     * @param rightSense right light sensor
     */
    boolean returnFromFree(float leftSense, float rightSense)
    {
        switch (returnState)
        {
        case TURN_RETURN:
            if (turn((Turn_SET_UP_ANGLE + (bagCenter / 2)), 180))
            {
                returnState = DRIVE_RETURN;
            }
            break;
        case DRIVE_RETURN:
            if (driveTillLine(DRIVE_SPEED, leftSense, rightSense))
            {
                returnState = DRIVE_TO_LINE;
            }
            break;
        case DRIVE_TO_LINE:
            if (driveInches(3, DRIVE_SPEED))
            {
                returnState = TURN_TWO;
            }
            break;
        case TURN_TWO:
            if (alignToLine(1, leftSense, rightSense)) //face the drop area TODO find turn left or right
            {
                returnState = DONE_RETURN;
            }
            // if (turn(90, 270))
            // {
            //     returnState = DONE_RETURN;
            // }
            break;
        case DONE_RETURN:
            bagCenter = 0;
            counter = 1;
            returnState = TURN_RETURN;
            return true;
            break;
        }
        return false;
    }
};
