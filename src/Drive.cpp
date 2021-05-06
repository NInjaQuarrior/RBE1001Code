#include <RBE1001Lib.h>
#include <RemoteConstants.h>
//#define PI 3.14159265358979323846

class Drive
{
private:
    LeftMotor left;
    RightMotor right;

    //constantss ======================================
    const float WHEEL_DIAMETER = 2.75f;

    //centi to inch conversion
    const float CENTI_CONV = 2.54f;

    //Kp for the ultra moving
    const float ULTRA_PROP = .07f;

    //deadband for moving to a distance with ultra
    const float ULTRA_DEAD = .35f;

    //speed while using ultra
    const float ULTRA_DRIVE = 90;

    //deadband for finding an object(bag) with thge ultra
    const float FIND_BAG_DEAD = 5;

    //base follow line speed
    const float LINE_BASE_SPEED = .2f;

    //Kp for following the line
    const float LINE_PROP = .1f;

    //voltage value for determining if a sensor is over the line
    const float LINE_SENSE_BLACK = 1.6f;

    //angle to turn before looking for object(bag)
    const float Turn_SET_UP_ANGLE = 20.0f;

    //turn speed in degrees per second
    const float TURN_SPEED = 270.0f;

    //angle to scan while looking for object
    const float SCAN_ANGLE = 150.0f;

    //speed to turn while scanning for object in degrees per second
    const float SCAN_SPEED = 270.0f;

    //distance to stop away from bag to pick it up
    const float DIST_FROM_BAG = 2.5f; //TODO tune

    //speed to drive in degrees per second
    const float DRIVE_SPEED = 270.0f;

    //max distance that the ultra will care about while scanning for a bag
    const float MAX_DIST = 30;

    //degrees per second to move in teleop
    const float TELEOP_SPEED = 180;

    //end constants+++++++++++++++++++++++++++++++++++++++++++

    //START enums for state machines ====================================
    //for scanning for object(bag)
    const enum ScanState {
        INIT_SCAN,
        SCANNING,
        TURN_TO,
        DRIVE_SCAN,
        DONE_SCAN
    };

    ScanState scanState = INIT_SCAN;

    //for returning to line from picking up bag from free zone
    const enum ReturnStateFree {
        TURN_RETURN,
        TURN_TWO,
        DRIVE_RETURN,
        DRIVE_TO_LINE,
        DONE_RETURN
    };

    ReturnStateFree returnState = TURN_RETURN;

    const enum DropZeroState {
        INIT_DRIVE_ZERO,
        INIT_TURN_ZERO,
        ALIGN_LINE_ZERO,
        DRIVE_TO_SECT_ZERO,
        PREP_DRIVE_ZERO,
        DRIVE_TO_ZONE_ZERO

    };

    DropZeroState dropZeroState = INIT_TURN_ZERO;

    const enum DropOneState {
        INIT_DRIVE_ONE,
        INIT_TURN_ONE,
        ALIGN_LINE_ONE,
        DRIVE_TO_SECT_ONE,
        PREP_MOVE_TURN_ONE,
        PREP_TURN_ONE,
        TURN_ONE,
        DRIVE_TO_ZONE_ONE
    };

    DropOneState dropOneState = INIT_DRIVE_ONE;

    const enum DropTwoState {
        INIT_DRIVE_TWO,
        INIT_TURN_TWO,
        ALIGN_LINE_TWO,
        DRIVE_TO_ZONE_TWO
    };

    DropTwoState dropTwoState = INIT_DRIVE_TWO;

    const enum ReturnDropZeroState {
        INIT_DRIVE_ZERO_R,
        PREP_NEXT_DRIVE_ZERO_R,
        DRIVE_SEC_ZERO_R,
        PREP_MOVE_TURN_ZERO_R,
        PREP_TURN_ZERO_R,
        ALIGN_LINE_ZERO_R

    };

    ReturnDropZeroState returnZeroState = INIT_DRIVE_ZERO_R;

    const enum ReturnDropOneState {
        INIT_DRIVE_ONE_R,
        PREP_MOVE_LEFT_ONE_R,
        PREP_TURN_LEFT_ONE_R,
        TURN_LEFT_ONE_R,
        DRIVE_SEC_ONE_R,
        REP_MOVE_TURN_ONE_R,
        PREP_TURN_ONE_R,
        ALIGN_LINE_ONE_R
    };

    ReturnDropOneState returnOneState = INIT_DRIVE_ONE_R;

    const enum ReturnDropTwoState {
        INIT_DRIVE_TWO_R,
        PREP_MOVE_TURN_TWO_R,
        PREP_TURN_TWO_R,
        TURN_TWO_R
    };

    ReturnDropTwoState returnTwoState = INIT_DRIVE_TWO_R;

    const enum MoveToPrepState {
        INIT_DRIVE_P,
        PREP_RIGHT_TURN_ONE,
        RIGHT_TURN_P,
        DRIVE_TO_SECOND_SECT,
        SEC_DRIVE_P,
        PREP_LEFT_TURN,
        LEFT_TURN_ONE_P,
        DRIVE_TO_THIRD_SECT
    };

    MoveToPrepState movePrepState = INIT_DRIVE_P;

    const enum MoveToStartState {
        INIT_DRIVE_S,
        DRIVE_TO_SECT_1_S,
        SEC_DRIVE_S,
        PREP_RIGHT_TURN_S,
        RIGHT_TURN_S,
        DRIVE_TO_SECT_2_S,
        TRD_DRIVE_S,
        PREP_LEFT_TURN_S,
        LEFT_TURN_S,

    };

    MoveToStartState moveStartState = INIT_DRIVE_S;

    //END enums for state machines +++++++++++++++++++++++++++++++++++++++++++

    //TODO remove usless methods from previous activities
public:
    /**
    * turns a certain amount of degrees
    * @param degrees degrees to turn, negative to turn counter-clockwise
    * @param speed degrees per second to move
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

    //to be deleted along with teleop
    float lastEffort = 0;

    /**
    * uses remote to control robot, numbers scale speed by .1, arrows turn 90 degrees
    * @param button
    */
    void teleOp(uint16_t button) //TODO delete
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
    * uses remote to control robot, numbers scale speed by .1, arrows turn 90 degrees
    * @param button
    */
    void teleOpAuto(uint16_t button)
    {
        if (button == remoteLeft)
        {
            turn(-45, TURN_SPEED);
        }
        else if (button == remoteRight)
        {
            turn(45, TURN_SPEED);
        }

        left.setSpeed(TELEOP_SPEED);
        right.setSpeed(TELEOP_SPEED);
    }

    /**
    * Makes a shape
    * @param sides numbers of side for the shape
    * @param sideLength  the length of each side in inches
    * @param speed degrees per second to move while making shape
    */
    boolean makeShape(int sides, float sideLength, float speed) //TODO delete
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
    boolean makeSpiral(float baseLength, float speed, int sides, int spiralAmount) //TODO delete
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
    boolean driveToInches(float inches, float curDist)
    {
        if (curDist > inches - ULTRA_DEAD && curDist < inches + ULTRA_DEAD)
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
    boolean lineFollowTillLine(float leftSense, float rightSense, float error)
    {
        if (leftSense > LINE_SENSE_BLACK || rightSense > LINE_SENSE_BLACK /*&& error < lineFollowTurnDead*/)
        {
            return true;
        }
        followLine(error, leftSense, rightSense);
        return false;
    }

    /**
     * line follow until ultra reaches distance
     * @param speed the speed in degrees per second
     * @param leftSense the current value of left Sensor
     * @param rightSense current value of the right sensor
     * @param curDist
     * @param targetDist
     */
    boolean lineFollowToTargetDistance(float leftSense, float rightSense, float error, float curDist, float targetDist)
    {
        if (curDist <= targetDist)
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
     * 
     */
    boolean findBag(float curDist)
    {
        switch (scanState)
        {
        //set up for the scane
        case INIT_SCAN:
            bagStartAngle = 0;
            bagEndAngle = 0;

            if (turn(Turn_SET_UP_ANGLE, TURN_SPEED))
            {
                prevDist = curDist;
                scanState = SCANNING;
            }
            break;
        //look for bag
        case SCANNING:
            //for scan_angle look for bag
            if (counter <= SCAN_ANGLE)
            {
                turn(1, SCAN_SPEED); //slowly now

                //if there is a sudden drop in distance greater than a deadband
                if (bagStartAngle == 0 && prevDist > curDist && (prevDist - curDist) > FIND_BAG_DEAD && curDist < MAX_DIST)
                {

                    prevDist = curDist;
                    bagStartAngle = counter; //store angle
                }
                //if there is a sudden rise in distance above a deadband
                else if (bagStartAngle != 0 && prevDist < curDist && (curDist - prevDist) > FIND_BAG_DEAD)
                {
                    bagEndAngle = counter; //store angle
                    scanState = TURN_TO;
                }
                counter++;
            }
            break;
        //turn to center of bag
        case TURN_TO:
            bagCenter = bagEndAngle - bagStartAngle;
            //-3 to be safe with turning
            if (turn(-(bagCenter / 2 - 3), 270))
            {
                scanState = DRIVE_SCAN;
            }
            break;
        //drive to bag
        case DRIVE_SCAN:
            if (driveToInches(DIST_FROM_BAG, curDist))
            {
                scanState = DONE_SCAN;
            }
            break;
        //hope you didnt miss cus everythings being reset
        case DONE_SCAN:
            //reset variables
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
            //attempt to face perpendicular to the line
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
            //center robot on the line
            if (driveInches(3, DRIVE_SPEED))
            {
                returnState = TURN_TWO;
            }
            break;
        case TURN_TWO:
            //reset robot on line
            if (alignToLine(1, leftSense, rightSense))
            {
                returnState = DONE_RETURN;
            }
            break;
        case DONE_RETURN:
            //reset variables
            bagCenter = 0;
            counter = 1;
            returnState = TURN_RETURN;
            return true;
            break;
        }
        return false;
    }
    //store if the robot is at the intersection closest to the three inch platform
    boolean isInPrepPos = false;

    /**
    * starting at the intersection closest to the start zone, go the the designated drop zone
    * @param dropZone int associated with the drop zone. 0 ground, 1 1.5in, 2 3in
    * @param leftSensor left light sensor value
    * @param rightSensor right light sensor value
    * @param error difference between right and left light sensor
    * @param ultraDist current ultrasonic distance
    * 
    */
    boolean driveToDropZone(int dropZone, float leftSensor, float rightSensor, float error, float ultraDist)
    {

        //if in the right position
        if (isInPrepPos == true)
        {
            switch (dropZone)
            {
            //drop zone 1
            case 0:
                switch (dropZeroState)
                {
                case INIT_DRIVE_ZERO:
                    if (driveInches(3, DRIVE_SPEED))
                    {
                        dropZeroState = INIT_TURN_ZERO;
                    }
                    break;
                case INIT_TURN_ZERO:
                    if (turn(-45, TURN_SPEED))
                    {
                        dropZeroState = ALIGN_LINE_ZERO;
                    }
                    break;
                case ALIGN_LINE_ZERO:
                    if (alignToLine(-1, leftSensor, rightSensor))
                    {
                        dropZeroState = DRIVE_TO_ZONE_ZERO;
                    }
                    break;
                case DRIVE_TO_SECT_ZERO:
                    if (lineFollowTillLine(leftSensor, rightSensor, error))
                    {
                        dropZeroState = INIT_DRIVE_ZERO;
                    }
                    break;
                case PREP_DRIVE_ZERO:
                    if (driveInches(3, DRIVE_SPEED))
                    {
                        dropZeroState = INIT_TURN_ZERO;
                    }
                    break;
                case DRIVE_TO_ZONE_ZERO:
                    if (lineFollowTillLine(leftSensor, rightSensor, error))
                    {
                        //reset stuff
                        dropZeroState = INIT_DRIVE_ZERO;
                        isInPrepPos = false;
                        return true;
                    }
                    break;
                }
                break;
            //1.5in dropzone
            case 1:
                switch (dropOneState)
                {
                case INIT_DRIVE_ONE:
                    if (driveInches(3, DRIVE_SPEED))
                    {
                        dropOneState = INIT_DRIVE_ONE;
                    }
                    break;
                case INIT_TURN_ONE:
                    if (turn(-45, TURN_SPEED))
                    {
                        dropOneState = ALIGN_LINE_ONE;
                    }
                    break;
                case ALIGN_LINE_ONE:
                    if (alignToLine(-1, leftSensor, rightSensor))
                    {
                        dropOneState = DRIVE_TO_SECT_ONE;
                    }
                    break;
                case DRIVE_TO_SECT_ONE:
                    if (lineFollowTillLine(leftSensor, rightSensor, error))
                    {
                        dropOneState = PREP_MOVE_TURN_ONE;
                    }
                    break;
                case PREP_MOVE_TURN_ONE:
                    if (driveInches(3, DRIVE_SPEED))
                    {
                        dropOneState = PREP_TURN_ONE;
                    }
                    break;
                case PREP_TURN_ONE:
                    if (turn(45, TURN_SPEED))
                    {
                        dropOneState = TURN_ONE;
                    }
                    break;
                case TURN_ONE:
                    if (alignToLine(1, leftSensor, rightSensor))
                    {
                        dropOneState = DRIVE_TO_ZONE_ONE;
                    }
                    break;
                case DRIVE_TO_ZONE_ONE:
                    if (lineFollowToTargetDistance(leftSensor, rightSensor, error, ultraDist, 3))
                    {
                        //reset stuff
                        dropOneState = INIT_DRIVE_ONE;
                        isInPrepPos = false;
                        return true;
                    }
                    break;
                }

                break;
            // 3in dropzone
            case 2:
                switch (dropTwoState)
                {
                case INIT_DRIVE_TWO:
                    if (driveInches(3, DRIVE_SPEED))
                    {
                        dropTwoState = INIT_TURN_TWO;
                    }
                    break;
                case INIT_TURN_TWO:
                    if (turn(45, TURN_SPEED))
                    {
                        dropTwoState = ALIGN_LINE_TWO;
                    }
                    break;
                case ALIGN_LINE_TWO:
                    if (alignToLine(1, leftSensor, rightSensor))
                    {
                        dropTwoState = DRIVE_TO_ZONE_TWO;
                    }
                    break;
                case DRIVE_TO_ZONE_TWO:
                    if (lineFollowToTargetDistance(leftSensor, rightSensor, error, ultraDist, 3)) //TODO tune 3 and make constant
                    {
                        dropTwoState = INIT_DRIVE_TWO;
                        isInPrepPos = false;
                        return true;
                    }
                    break;
                }
                break;
            //if for some reason drop zone wasnt set put on ground level
            case -1:
                dropZone = 0;
                break;
            }
        }
        //if not in correct position, go there
        else if (moveToPrepDropPos(leftSensor, rightSensor, error))
        {
            isInPrepPos = true;
        }
        return false;
    }

    /**
     * return to intersection closest to start zone
     * @param dropZone int associated with the drop zone. 0 ground, 1 1.5in, 2 3in
    * @param leftSensor left light sensor value
    * @param rightSensor right light sensor value
    * @param error difference between right and left light sensor
     */
    boolean returnFromDropZone(int dropZone, float leftSensor, float rightSensor, float error)
    {

        //if not in correct position go there
        if (isInPrepPos == false)
        {
            switch (dropZone)
            {
                //at ground zone
            case 0:
                switch (returnZeroState)
                {
                case INIT_DRIVE_ZERO_R:
                    if (lineFollowTillLine(leftSensor, rightSensor, error))
                    {
                        returnZeroState = PREP_NEXT_DRIVE_ZERO_R;
                    }
                    break;
                case PREP_NEXT_DRIVE_ZERO_R:
                    if (driveInches(3, DRIVE_SPEED))
                    {
                        returnZeroState = DRIVE_SEC_ZERO_R;
                    }
                    break;
                case DRIVE_SEC_ZERO_R:
                    if (lineFollowTillLine(leftSensor, rightSensor, error))
                    {
                        returnZeroState = PREP_MOVE_TURN_ZERO_R;
                    }
                    break;
                case PREP_MOVE_TURN_ZERO_R:
                    if (driveInches(3, DRIVE_SPEED))
                    {
                        returnZeroState = PREP_TURN_ZERO_R;
                    }
                    break;
                case PREP_TURN_ZERO_R:
                    if (turn(45, TURN_SPEED))
                    {
                        returnZeroState = ALIGN_LINE_ZERO_R;
                    }
                    break;
                case ALIGN_LINE_ZERO_R:
                    if (alignToLine(1, leftSensor, rightSensor))
                    {
                        returnZeroState = INIT_DRIVE_ZERO_R;
                        isInPrepPos = true;
                    }
                    break;
                }
                break;
            //at 1.5in zone
            case 1:
                switch (returnOneState)
                {
                case INIT_DRIVE_ONE_R:
                    if (lineFollowTillLine(leftSensor, rightSensor, error))
                    {
                        returnOneState = PREP_TURN_LEFT_ONE_R;
                    }
                    break;
                case PREP_MOVE_LEFT_ONE_R:
                    if (driveInches(3, DRIVE_SPEED))
                    {
                        returnOneState = PREP_TURN_LEFT_ONE_R;
                    }
                    break;
                case PREP_TURN_LEFT_ONE_R:
                    if (turn(-45, TURN_SPEED))
                    {
                        returnOneState = TURN_LEFT_ONE_R;
                    }
                    break;
                case TURN_LEFT_ONE_R:
                    if (alignToLine(-1, leftSensor, rightSensor))
                    {
                        returnOneState = DRIVE_SEC_ONE_R;
                    }
                    break;
                case DRIVE_SEC_ONE_R:
                    if (lineFollowTillLine(leftSensor, rightSensor, error))
                    {
                        returnOneState = PREP_TURN_LEFT_ONE_R;
                    }
                    break;
                case REP_MOVE_TURN_ONE_R:
                    if (driveInches(3, DRIVE_SPEED))
                    {
                        returnOneState = PREP_TURN_ONE_R;
                    }
                    break;
                case PREP_TURN_ONE_R:
                    if (turn(45, TURN_SPEED))
                    {
                        returnOneState = ALIGN_LINE_ONE_R;
                    }
                    break;
                case ALIGN_LINE_ONE_R:
                    if (alignToLine(1, leftSensor, rightSensor))
                    {
                        returnOneState = INIT_DRIVE_ONE_R;
                        isInPrepPos = true;
                    }
                    break;
                }
                break;
            //at 3in zone
            case 2:
                switch (returnTwoState)
                {
                case INIT_DRIVE_TWO_R:
                    if (lineFollowTillLine(leftSensor, rightSensor, error))
                    {
                        returnTwoState = PREP_MOVE_TURN_TWO_R;
                    }
                    break;
                case PREP_MOVE_TURN_TWO_R:
                    if (driveInches(3, DRIVE_SPEED))
                    {
                        returnTwoState = PREP_TURN_TWO_R;
                    }
                    break;
                case PREP_TURN_TWO_R:
                    if (turn(-45, TURN_SPEED))
                    {
                        returnTwoState = TURN_TWO_R;
                    }
                    break;
                case TURN_TWO_R:
                    if (alignToLine(1, leftSensor, rightSensor))
                    {
                        returnTwoState = INIT_DRIVE_TWO_R;
                        isInPrepPos = true;
                    }
                    break;
                }
                break;
            case -1:
                dropZone = 0;
                break;
            }
        }
        //if in correct position, go to intersection closest to start
        else if (returnFromPrepDropPos(leftSensor, rightSensor, error) == true)
        {
            isInPrepPos = false;
            return true;
        }

        return false;
    }

    /**
     * When at intersection closest to start go to the intersection closest to 3in zone
     * @param leftSensor left light sensor value
    * @param rightSensor right light sensor value
    * @param error difference between right and left light sensor
     */
    boolean moveToPrepDropPos(float leftSense, float rightSense, float error)
    {
        //it moves through various steps to go there
        switch (movePrepState)
        {
        case INIT_DRIVE_P:
            if (driveInches(3, DRIVE_SPEED))
            {
                movePrepState = PREP_RIGHT_TURN_ONE;
            }
            break;
        case PREP_RIGHT_TURN_ONE:
            if (turn(45, TURN_SPEED))
            {
                movePrepState = RIGHT_TURN_P;
            }
            break;
        case RIGHT_TURN_P:
            if (alignToLine(1, leftSense, rightSense))
            {
                movePrepState = DRIVE_TO_SECOND_SECT;
            }
            break;
        case DRIVE_TO_SECOND_SECT:
            if (lineFollowTillLine(leftSense, rightSense, error))
            {
                movePrepState = SEC_DRIVE_P;
            }
            break;
        case SEC_DRIVE_P:
            if (driveInches(3, DRIVE_SPEED))
            {
                movePrepState = PREP_LEFT_TURN;
            }
            break;
        case PREP_LEFT_TURN:
            if (turn(-45, TURN_SPEED))
            {
                movePrepState = LEFT_TURN_ONE_P;
            }
            break;
        case LEFT_TURN_ONE_P:
            if (alignToLine(-1, leftSense, rightSense))
            {
                movePrepState = DRIVE_TO_THIRD_SECT;
            }
            break;

        case DRIVE_TO_THIRD_SECT:
            if (lineFollowTillLine(leftSense, rightSense, error))
            {
                movePrepState = INIT_DRIVE_P;
                return true;
            }
            break;
        }
        return false;
    }

    /**
     * If at the intersection closest to 3in zone go to intersect closest to start
     *  @param leftSensor left light sensor value
    * @param rightSensor right light sensor value
    * @param error difference between right and left light sensor
     * 
     */
    boolean returnFromPrepDropPos(float leftSense, float rightSense, float error)
    {
        //it moves through various steps to go there
        switch (moveStartState)
        {
        case INIT_DRIVE_S:
            if (driveInches(3, DRIVE_SPEED))
            {
                moveStartState = DRIVE_TO_SECT_1_S;
            }
            break;
        case DRIVE_TO_SECT_1_S:
            if (lineFollowTillLine(leftSense, rightSense, error))
            {
                moveStartState = SEC_DRIVE_S;
            }
            break;
        case SEC_DRIVE_S:
            if (driveInches(3, DRIVE_SPEED))
            {
                moveStartState = PREP_RIGHT_TURN_S;
            }
            break;
        case PREP_RIGHT_TURN_S:
            if (turn(45, TURN_SPEED))
            {
                moveStartState = RIGHT_TURN_S;
            }
            break;
        case RIGHT_TURN_S:
            if (alignToLine(1, leftSense, rightSense))
            {
                moveStartState = DRIVE_TO_SECT_2_S;
            }
            break;
        case DRIVE_TO_SECT_2_S:
            if (lineFollowTillLine(leftSense, rightSense, error))
            {
                moveStartState = TRD_DRIVE_S;
            }
            break;
        case TRD_DRIVE_S:
            if (driveInches(3, DRIVE_SPEED))
            {
                moveStartState = PREP_LEFT_TURN_S;
            }
            break;

        case PREP_LEFT_TURN_S:
            if (turn(4 - 5, TURN_SPEED))
            {
                moveStartState = LEFT_TURN_S;
            }
            break;
        case LEFT_TURN_S:
            if (alignToLine(-1, leftSense, rightSense))
            {
                moveStartState = INIT_DRIVE_S;
                return true;
            }
            break;
        }
        return false;
    }
};
