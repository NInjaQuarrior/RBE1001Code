#include <RBE1001Lib.h>
#include <RemoteConstants.h>
//#define PI 3.14159265358979323846

class Drive
{
private:
    LeftMotor left;
    RightMotor right;

    //constantssss ======================================
    const float WHEEL_DIAMETER = 2.75f;

    //centi to inch conversion
    const float CENTI_CONV = 2.54f;

    //Kp for the ultra moving
    const float ULTRA_PROP = .07f;

    //deadband for moving to a distance with ultra
    const float ULTRA_DEAD = .35f;

    //speed while using ultra
    const float ULTRA_DRIVE = 90.0f;

    //deadband for finding an object(bag) with thge ultra
    const float FIND_BAG_DEAD = 5.0f;

    //base follow line speed
    const float LINE_BASE_SPEED = .22f;

    //Kp for following the line
    const float LINE_PROP = .09f;

    //voltage value for determining if a sensor is over the line
    const float LINE_SENSE_BLACK = 1.4f;

    //angle to turn before looking for object(bag)
    const float Turn_SET_UP_ANGLE = 15.0f;

    //angle to scan while looking for object
    const float SCAN_ANGLE = 150.0f;

    //distance to stop away from bag to pick it up
    const float DIST_FROM_BAG = 6.0f;

    //max distance that the ultra will care about while scanning for a bag
    const float MAX_DIST = 25.0f;

    //speed to turn while scanning for object in degrees per second
    const float SCAN_SPEED = 270.0f;

    //degrees per second to move in teleop
    const float TELEOP_SPEED = 180.0f;

    //medium drive speed
    const float DRIVE_SPEED_MED = 180.0f;

    //fast drive speed
    const float DRIVE_SPEED_FAST = 270.0f;

    //slow drive speed
    const float DRIVE_SPEED_SLOW = 70.0f;

    //angle for preparing to align to the line with the light sensors
    const float PREP_ALIGN_ANGLE = 60.0f;

    const float CENTER_ROBOT_DIST = 3.0f;

    //turn speed in degrees per second medium
    const float TURN_SPEED_MED = 180.0f;

    //turn speed in degrees per second
    const float TURN_SPEED_FAST = 270.0f;

    //turn speed in degrees per second
    const float TURN_SPEED_SLOW = 70.0f;

    //turn left
    const int DIR_LEFT = -1;

    //turn right
    const int DIR_RIGHT = 1;

    //constants for plat states
    const int GROUND_PLAT = 0;

    const int MEDIUM_PLAT = 1;

    const int HIGH_PLAT = 2;

    //end constantsssss+++++++++++++++++++++++++++++++++++++++++++

    //START enums for state machines ====================================

    //for scanning for object(bag)
    enum ScanState
    {
        INIT_SCAN,
        SCANNING,
        TURN_TO_SCAN,
        DRIVE_SCAN,
        DONE_SCAN
    };

    ScanState scanState = INIT_SCAN;

    //for returning to line from picking up bag from free zone
    enum ReturnStateFree
    {
        TURN_RETURN_FREE,
        TURN_TWO_FREE,
        DRIVE_RETURN_FREE,
        DRIVE_TO_LINE_FREE,
        DONE_RETURN_FREE
    };

    ReturnStateFree returnState = TURN_RETURN_FREE;

    //for going to the ground platform
    enum DropZeroState
    {
        INIT_DRIVE_ZERO,
        INIT_TURN_ZERO,
        ALIGN_LINE_ZERO,
        DRIVE_TO_SECT_ZERO,
        PREP_DRIVE_ZERO,
        DRIVE_TO_ZONE_ZERO,
        BACK_UP_ZERO

    };

    DropZeroState dropZeroState = INIT_DRIVE_ZERO;

    //for going to the 1.5 inch platform
    enum DropOneState
    {
        INIT_DRIVE_ONE,
        INIT_TURN_ONE,
        ALIGN_LINE_ONE,
        DRIVE_TO_SECT_ONE,
        PREP_MOVE_TURN_ONE,
        PREP_TURN_ONE,
        TURN_ONE,
        FINAL_MOVE_ONE
    };

    DropOneState dropOneState = INIT_DRIVE_ONE;

    //for going to the 3in platform
    enum DropTwoState
    {
        INIT_DRIVE_TWO,
        INIT_TURN_TWO,
        ALIGN_LINE_TWO,
        DRIVE_TO_ZONE_TWO,
        TURN_LEFT_TWO,
        FINAL_ALIGN_TWO
    };

    DropTwoState dropTwoState = INIT_DRIVE_TWO;

    //return to start from ground plat
    enum ReturnDropZeroState
    {
        INIT_DRIVE_ZERO_R,
        PREP_NEXT_DRIVE_ZERO_R,
        PREP_TURN_ZERO_R,
        ALIGN_LINE_ZERO_R,
        FINAL_TO_START_ZERO_R

    };

    ReturnDropZeroState returnZeroState = INIT_DRIVE_ZERO_R;

    //return to start from 1.5in plat
    enum ReturnDropOneState
    {
        INIT_DRIVE_FORWARD_ONE_R,
        ALIGN_TO_ONE_R,
        DRIVE_FINAL_ONE_R
    };

    ReturnDropOneState returnOneState = INIT_DRIVE_FORWARD_ONE_R;

    //return to start from 3in plat
    enum ReturnDropTwoState
    {
        INIT_DRIVE_TWO_R,
        PREP_MOVE_TURN_TWO_R,
        PREP_TURN_TWO_R,
        TURN_TWO_R,
        FINAL_FOR_TWO_R
    };

    ReturnDropTwoState returnTwoState = INIT_DRIVE_TWO_R;

    //moving to the intersection closest to 3in plat
    enum MoveToPrepState
    {
        INIT_DRIVE_P,
        PREP_RIGHT_TURN_ONE_P,
        RIGHT_TURN_P,
        DRIVE_TO_SECOND_SECT_P,
        SEC_DRIVE_P,
        PREP_LEFT_TURN_P,
        LEFT_TURN_ONE_P,
        DRIVE_TO_THIRD_SECT_P
    };

    MoveToPrepState movePrepState = INIT_DRIVE_P;

    //END enums for state machines +++++++++++++++++++++++++++++++++++++++++++

public:
    /**
    * turns a certain amount of degrees
    * @param degrees degrees to turn, negative to turn counter-clockwise
    * @param speed degrees per second to move
    * @return true when complete
    */
    boolean turn(float degrees, float speed)
    {
        //degrees for each wheel for move
        float moveDegrees = 2 * degrees;
        //move
        left.startMoveFor(moveDegrees, speed);
        right.moveFor(-moveDegrees, speed);

        return true;
    }

    /**
     * turn until not called
     * @param direct -1 for left, 1 for right
     * @param speed in degrees for second
     */
    void turnContinuous(int direct, float speed)
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
    * @return true when complete
    */
    boolean driveInches(float inches, float speed)
    {
        //degrees for each wheel to move
        float moveDegrees = (inches / (2 * PI * (WHEEL_DIAMETER / 2))) * 360;
        //move
        left.startMoveFor(moveDegrees, speed);
        right.moveFor(moveDegrees, speed);

        return true;
    }

    /**
    * drive straight a certain amount of inches
    * @param centi centimeters to move, negative to go backwars
    * @param speed degrees per second to move
    * @return true when complete
    */
    boolean driveCentimeters(float centi, float speed)
    {
        //degrees for each wheel to move
        float moveDegrees = (centi / (2 * PI * ((WHEEL_DIAMETER / CENTI_CONV) / 2))) * 360;
        //move
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
    * @param speed in degrees per second, negative goes backwards 
    */
    void setSpeed(float speed)
    {
        left.setSpeed(speed);
        right.setSpeed(speed);
    }

    /**
    * drives to a set distance away from a target using the ultrasonic
    * @param targetDist distance to move to
    * @param curDist the ultrasonic same unit as targetDist
    * @return true when at proper distance
    */
    boolean driveTo(float targetDist, float curDist)
    {
        //if the robot is at the distance within a deadband
        if (curDist > targetDist - ULTRA_DEAD && curDist < targetDist + ULTRA_DEAD)
        {
            //stop
            setEffort(0);
            return true;
        }

        //move the robot to right distance
        setSpeed(ULTRA_DRIVE);

        return false;
    }

    /**
     * follows the black line using p control
     * @param error the currect difference between the two line sensors getDifference()
     * @param leftSense the current value of left Sensor 
     * @param rightSense current value of the right sensor
     */
    void followLine(float error, float leftSense, float rightSense)
    {
        left.setEffort(LINE_BASE_SPEED + (error * LINE_PROP));
        right.setEffort(LINE_BASE_SPEED - (error * LINE_PROP));
    }

    /**
     * drive straight forward until find a line
     * @param speed the speed in degrees per second
     * @param leftSense the current value of left Sensor
     * @param rightSense current value of the right sensor
     * @return true when hits an line
     */
    boolean driveTillLine(float speed, float leftSense, float rightSense)
    {
        //if either light sensors og above the dark value
        if (leftSense > LINE_SENSE_BLACK || rightSense > LINE_SENSE_BLACK /*&& error < lineFollowTurnDead*/)
        {
            return true;
        }
        //keep driving
        setSpeed(speed);
        return false;
    }

    /**
     * line follow until find a t intersection
     * @param speed the speed in degrees per second
     * @param leftSense the current value of left Sensor
     * @param rightSense current value of the right sensor
     * @return true when hits an line
     */
    boolean lineFollowTillLine(float leftSense, float rightSense, float error)
    {
        //if either light sensors og above the dark value
        if (leftSense > LINE_SENSE_BLACK && rightSense > LINE_SENSE_BLACK /*&& error < lineFollowTurnDead*/)
        {
            return true;
        }
        //keep following line
        followLine(error, leftSense, rightSense);
        return false;
    }

    /**
     * line follow until ultra reaches target distance, only goes forward
     * @param leftSense the current value of left Sensor
     * @param rightSense current value of the right sensor
     * @param error difference between left and right sensors
     * @param curDist current ultrasonic distance 
     * @param targetDist target distance, same unit as curDist
     * @return true when at target distance
     */
    boolean lineFollowToTargetDistance(float leftSense, float rightSense, float error, float curDist, float targetDist)
    {
        //if not in target distance
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
    * @return true found a line
    */
    boolean alignToLine(int direct, float leftSense, float rightSense)
    {
        //turn left
        if (direct < 0)
        {
            //right light sensor found line
            if (rightSense > LINE_SENSE_BLACK - 1)
            {
                left.setSpeed(0);
                right.setSpeed(0);
                return true;
            }
        }
        //turn right
        else if (direct >= 0)
        {
            //left line sensor found line
            if (leftSense > LINE_SENSE_BLACK - 1)
            {
                left.setSpeed(0);
                right.setSpeed(0);
                return true;
            }
        }
        //keep turning
        turnContinuous(direct, TURN_SPEED_MED);
        return false;
    }

    //variables for findBag
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
     * @return true when driven to the bag
     */
    boolean findBag(float curDist)
    {
        switch (scanState)
        {
        //set up for the scan
        case INIT_SCAN:
            bagStartAngle = 0;
            bagEndAngle = 0;
            //turn off the line toward free range
            if (turn(Turn_SET_UP_ANGLE, TURN_SPEED_MED))
            {
                //set prevDist
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

                //if there is a sudden drop in distance greater than a deadband and the object found is closer than a max
                if (bagStartAngle == 0 && prevDist > curDist && (prevDist - curDist) > FIND_BAG_DEAD && curDist < MAX_DIST)
                {
                    //new prevDist
                    prevDist = curDist;
                    bagStartAngle = counter; //store angle
                }
                //if there is a sudden rise in distance above a deadband
                else if (bagStartAngle != 0 && prevDist < curDist && (curDist - prevDist) > FIND_BAG_DEAD)
                {
                    //store angle
                    bagEndAngle = counter;
                    //reset counter
                    counter = 1;
                    scanState = TURN_TO_SCAN;
                }
                //keep track of current angle
                counter++;
            }
            break;

        //turn to center of bag
        case TURN_TO_SCAN:
            //calculate distane to turn to bag center
            bagCenter = bagEndAngle - bagStartAngle;

            //turn to bag center -5 to be safe with turning
            if (turn(-((bagCenter / 2) - 5), TURN_SPEED_FAST))
            {
                scanState = DRIVE_SCAN;
            }
            break;

        //drive to bag
        case DRIVE_SCAN:
            if (driveTo(DIST_FROM_BAG, curDist))
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
     * @return true on the line facing the start
     */
    boolean returnFromFree(float leftSense, float rightSense)
    {
        switch (returnState)
        {

        //turn towards the line
        case TURN_RETURN_FREE:
            //attempt to face perpendicular to the line
            if (turn((Turn_SET_UP_ANGLE + (bagCenter / 2)), TURN_SPEED_MED))
            {
                returnState = DRIVE_RETURN_FREE;
            }
            break;

        //go back to the line
        case DRIVE_RETURN_FREE:
            //drive until found the line
            if (driveTillLine(DRIVE_SPEED_MED, leftSense, rightSense))
            {
                returnState = DRIVE_TO_LINE_FREE;
            }
            break;

        //center the robot on the line
        case DRIVE_TO_LINE_FREE:
            //center robot on the line
            if (driveInches(CENTER_ROBOT_DIST, DRIVE_SPEED_MED))
            {
                returnState = TURN_TWO_FREE;
            }
            break;
        //align to the line
        case TURN_TWO_FREE:

            //reset robot on line
            if (alignToLine(DIR_RIGHT, leftSense, rightSense))
            {
                returnState = DONE_RETURN_FREE;
            }
            break;

        case DONE_RETURN_FREE:
            //reset variables
            bagCenter = 0;
            counter = 1;
            returnState = TURN_RETURN_FREE;
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
    * @param leftSense left light sensor value
    * @param rightSense right light sensor value
    * @param error difference between right and left light sensor
    * @param ultraDist current ultrasonic distance
    * @return true if at the plat form facing it
    */
    boolean driveToDropZone(int dropZone, float leftSense, float rightSense, float error, float ultraDist)
    {

        //if has driven to the intersection closest to the 3in plat around construction
        if (isInPrepPos == true)
        {
            switch (dropZone)
            {
            case 0:
                switch (dropZeroState)
                {

                //center robot on intersection
                case INIT_DRIVE_ZERO:
                    if (driveInches(CENTER_ROBOT_DIST, DRIVE_SPEED_MED))
                    {
                        dropZeroState = INIT_TURN_ZERO;
                    }
                    break;

                //turn robot off the line
                case INIT_TURN_ZERO:
                    if (turn(-PREP_ALIGN_ANGLE, TURN_SPEED_MED))
                    {
                        dropZeroState = ALIGN_LINE_ZERO;
                    }
                    break;

                //turn to the line
                case ALIGN_LINE_ZERO:
                    //turn left till line
                    if (alignToLine(DIR_LEFT, leftSense, rightSense))
                    {
                        dropZeroState = DRIVE_TO_SECT_ZERO;
                    }
                    break;

                //follow line until next intersection
                case DRIVE_TO_SECT_ZERO:
                    if (lineFollowTillLine(leftSense, rightSense, error))
                    {
                        dropZeroState = PREP_DRIVE_ZERO;
                    }
                    break;

                    //center robot on intersection
                case PREP_DRIVE_ZERO:
                    if (driveInches(CENTER_ROBOT_DIST, DRIVE_SPEED_MED))
                    {
                        dropZeroState = DRIVE_TO_ZONE_ZERO;
                    }
                    break;

                //follow line until next intersection
                case DRIVE_TO_ZONE_ZERO:
                    if (lineFollowTillLine(leftSense, rightSense, error))
                    {
                        //reset stuff
                        dropZeroState = BACK_UP_ZERO;
                    }
                    break;

                //back up to prepared to drop off
                case BACK_UP_ZERO:
                    if (driveInches(-4, DRIVE_SPEED_MED))
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
                //center robot on intersection
                case INIT_DRIVE_ONE:
                    if (driveInches(CENTER_ROBOT_DIST, DRIVE_SPEED_MED))
                    {
                        dropOneState = INIT_TURN_ONE;
                    }
                    break;

                //turn off the line
                case INIT_TURN_ONE:
                    if (turn(-PREP_ALIGN_ANGLE, TURN_SPEED_MED))
                    {
                        dropOneState = ALIGN_LINE_ONE;
                    }
                    break;

                //turn left until line
                case ALIGN_LINE_ONE:
                    if (alignToLine(DIR_LEFT, leftSense, rightSense))
                    {
                        dropOneState = DRIVE_TO_SECT_ONE;
                    }
                    break;

                //drive to next intersection
                case DRIVE_TO_SECT_ONE:
                    if (lineFollowTillLine(leftSense, rightSense, error))
                    {
                        dropOneState = PREP_MOVE_TURN_ONE;
                    }
                    break;

                //center robot on the intersection
                case PREP_MOVE_TURN_ONE:
                    if (driveInches(CENTER_ROBOT_DIST, DRIVE_SPEED_MED))
                    {
                        dropOneState = PREP_TURN_ONE;
                    }
                    break;

                //turn the robot off the line
                case PREP_TURN_ONE:
                    if (turn(PREP_ALIGN_ANGLE, TURN_SPEED_MED))
                    {
                        dropOneState = TURN_ONE;
                    }
                    break;

                //turn until the line right
                case TURN_ONE:
                    if (alignToLine(DIR_RIGHT, leftSense, rightSense))
                    {
                        dropOneState = FINAL_MOVE_ONE;
                    }
                    break;

                //get to right distance
                case FINAL_MOVE_ONE:
                    if (driveInches(2.5, DRIVE_SPEED_MED))
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
                //center robot on intersection
                case INIT_DRIVE_TWO:
                    if (driveInches(CENTER_ROBOT_DIST, DRIVE_SPEED_MED))
                    {
                        dropTwoState = INIT_TURN_TWO;
                    }
                    break;

                //turn robot off of line
                case INIT_TURN_TWO:
                    if (turn(PREP_ALIGN_ANGLE, DRIVE_SPEED_SLOW))
                    {
                        dropTwoState = ALIGN_LINE_TWO;
                    }
                    break;

                //turn right until align to line
                case ALIGN_LINE_TWO:
                    if (alignToLine(DIR_RIGHT, leftSense, rightSense))
                    {
                        dropTwoState = DRIVE_TO_ZONE_TWO;
                    }
                    break;

                //drive to the platform
                case DRIVE_TO_ZONE_TWO:
                    if (driveTo(4.5, ultraDist))
                    {
                        dropTwoState = TURN_LEFT_TWO;
                    }
                    break;

                //wiggle a little to find where the robot is on the line
                case TURN_LEFT_TWO:
                    if (turn(-20, DRIVE_SPEED_SLOW))
                    {
                        dropTwoState = FINAL_ALIGN_TWO;
                    }
                    break;

                //do a final align to the line
                case FINAL_ALIGN_TWO:

                    //if already on the line the robot is done
                    if (leftSense > .3)
                    {
                        dropTwoState = INIT_DRIVE_TWO;
                        isInPrepPos = false;
                        return true;
                    }
                    //if not turn right until on the line
                    else if (alignToLine(DIR_RIGHT, leftSense, rightSense))
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
                dropZone = GROUND_PLAT;
                break;
            }
        }
        //if not in correct position, go there
        else if (moveToPrepDropPos(leftSense, rightSense, error))
        {
            isInPrepPos = true;
        }
        return false;
    }

    /**
     * return to intersection closest to start zone
     * @param dropZone int associated with the drop zone. 0 ground, 1 1.5in, 2 3in
    * @param leftSense left light sensor value
    * @param rightSense right light sensor value
    * @param error difference between right and left light sensor
    * @return at intersection closest to the start, facing the start
     */
    boolean returnFromDropZone(int dropZone, float leftSense, float rightSense, float error)
    {

        switch (dropZone)
        {
        //at ground zone
        case 0:
            switch (returnZeroState)
            {
            //follow the line to the next intersection
            case INIT_DRIVE_ZERO_R:
                if (lineFollowTillLine(leftSense, rightSense, error))
                {
                    returnZeroState = PREP_NEXT_DRIVE_ZERO_R;
                }
                break;

            //center robot on the intersection
            case PREP_NEXT_DRIVE_ZERO_R:
                if (driveInches(CENTER_ROBOT_DIST, DRIVE_SPEED_MED))
                {
                    returnZeroState = PREP_TURN_ZERO_R;
                }
                break;

            //turn robot off the line
            case PREP_TURN_ZERO_R:
                if (turn(PREP_ALIGN_ANGLE, TURN_SPEED_MED))
                {
                    returnZeroState = ALIGN_LINE_ZERO_R;
                }
                break;

            //turn to the right until line
            case ALIGN_LINE_ZERO_R:
                if (alignToLine(DIR_RIGHT, leftSense, rightSense))
                {
                    returnZeroState = FINAL_TO_START_ZERO_R;
                }
                break;

            //move to intersection closest to the start
            case FINAL_TO_START_ZERO_R:
                if (lineFollowTillLine(leftSense, rightSense, error))
                {
                    returnZeroState = INIT_DRIVE_ZERO_R;
                    return true;
                }
                break;
            }
            break;

        //at 1.5in zone
        case 1:

            switch (returnOneState)
            {
            case INIT_DRIVE_FORWARD_ONE_R:
                if (driveInches(CENTER_ROBOT_DIST, DRIVE_SPEED_MED)) // from DRIVE_SPEED
                {
                    returnOneState = ALIGN_TO_ONE_R;
                }
                break;
            case ALIGN_TO_ONE_R:
                if (alignToLine(DIR_RIGHT, leftSense, rightSense))
                {
                    returnOneState = DRIVE_FINAL_ONE_R;
                }
                break;
            case DRIVE_FINAL_ONE_R:
                if (lineFollowTillLine(leftSense, rightSense, error))
                {
                    returnOneState = INIT_DRIVE_FORWARD_ONE_R;
                    return true;
                }
                break;
            }
            break;

        //at 3in zone
        case 2:
            switch (returnTwoState)
            {

            //drive until next intersection
            case INIT_DRIVE_TWO_R:
                if (lineFollowTillLine(leftSense, rightSense, error))
                {
                    returnTwoState = PREP_MOVE_TURN_TWO_R;
                }
                break;

            //center robot on interection
            case PREP_MOVE_TURN_TWO_R:
                if (driveInches(CENTER_ROBOT_DIST, DRIVE_SPEED_MED))
                {
                    returnTwoState = PREP_TURN_TWO_R;
                }
                break;

            //turn robot left off the line
            case PREP_TURN_TWO_R:
                if (turn(-PREP_ALIGN_ANGLE, TURN_SPEED_MED))
                {
                    returnTwoState = TURN_TWO_R;
                }
                break;

            //turn left until reach line
            case TURN_TWO_R:
                if (alignToLine(DIR_LEFT, leftSense, rightSense))
                {
                    returnTwoState = FINAL_FOR_TWO_R;
                }
                break;

            //follow line until reach intersection closest to start
            case FINAL_FOR_TWO_R:

                if (lineFollowTillLine(leftSense, rightSense, error))
                {
                    returnTwoState = INIT_DRIVE_TWO_R;
                    return true;
                }
                break;
            }
            break;
        case -1:
            dropZone = GROUND_PLAT;
            break;
        }

        return false;
    }

    /**
     * When at intersection closest to start go to the intersection closest to 3in zone
     * @param leftSensor left light sensor value
    * @param rightSensor right light sensor value
    * @param error difference between right and left light sensor
    * @return true when at intersection closest to 3in zone
     */
    boolean moveToPrepDropPos(float leftSense, float rightSense, float error)
    {
        //it moves through various steps to go there
        switch (movePrepState)
        {

        //center robot on intersection
        case INIT_DRIVE_P:
            if (driveInches(CENTER_ROBOT_DIST, DRIVE_SPEED_MED))
            {
                movePrepState = PREP_RIGHT_TURN_ONE_P;
            }
            break;

        //turn right off the line
        case PREP_RIGHT_TURN_ONE_P:
            if (turn(PREP_ALIGN_ANGLE, TURN_SPEED_SLOW))
            {
                movePrepState = RIGHT_TURN_P;
            }
            break;

        //turn right until reach line
        case RIGHT_TURN_P:
            if (alignToLine(DIR_RIGHT, leftSense, rightSense))
            {
                movePrepState = DRIVE_TO_SECOND_SECT_P;
            }
            break;

        //drive to next intersection
        case DRIVE_TO_SECOND_SECT_P:
            if (lineFollowTillLine(leftSense, rightSense, error))
            {
                movePrepState = SEC_DRIVE_P;
            }
            break;

        //center robot on intersection
        case SEC_DRIVE_P:
            if (driveInches(CENTER_ROBOT_DIST, DRIVE_SPEED_MED))
            {
                movePrepState = PREP_LEFT_TURN_P;
            }
            break;

        //turn robot left off of line
        case PREP_LEFT_TURN_P:
            if (turn(-PREP_ALIGN_ANGLE, TURN_SPEED_SLOW))
            {
                movePrepState = LEFT_TURN_ONE_P;
            }
            break;

        //turn left until reach line
        case LEFT_TURN_ONE_P:
            if (alignToLine(-PREP_ALIGN_ANGLE, leftSense, rightSense))
            {
                movePrepState = DRIVE_TO_THIRD_SECT_P;
            }
            break;

        //drive until reach intersection closest to 3in plat
        case DRIVE_TO_THIRD_SECT_P:
            if (lineFollowTillLine(leftSense, rightSense, error))
            {
                movePrepState = INIT_DRIVE_P;
                return true;
            }
            break;
        }
        return false;
    }
};
