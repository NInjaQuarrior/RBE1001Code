#include <RBE1001Lib.h>
#include <IRdecoder.h>
#include <RemoteConstants.h>
#include <Drive.cpp>
#include <Ultrasonic.cpp>
#include <LineSense.cpp>
#include <ArmServo.cpp>
#include <ESP32Servo.h>
#include <ESP32AnalogRead.h>

//START declarations ====================================================================================

//ultrasonic class object
Ultrasonic ultra;
//Drive class object
Drive drive;
//Linesense class object
LineSense lSensor;
//ArmServo class object
ArmServo servo;
//deadband for if bag in pickup zone
const float BAG_PRESENT_DEAD = 12.0f;

//medium drive speed
const float DRIVE_SPEED_MED = 180.0f;

//fast drive speed
const float DRIVE_SPEED_FAST = 270.0f;

//slow drive speed
const float DRIVE_SPEED_SLOW = 70.0f;

//angle for preparing to align to the line with the light sensors
const float PREP_ALIGN_ANGLE = 60.0f;

const float CENTER_ROBOT_DIST = 4.0f;

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

//distance to move the robot into the start zone from the nearest intersection
const float DIST_TO_START = 7.0f;

//pin for ir detector
const uint8_t IR_DETECTOR_PIN = 15;

IRDecoder decoder(15);

//START enums for state machines ===================================

//for picking up a bag from freezon
enum FreeZoneState
{
  INIT,
  INIT_TURN_AROUND_ONE,
  TURN_AROUND_ONE,
  DRIVE_FORWARD_ONE,
  GO_TO_BAG,
  TURN_AROUND,
  REVERSE,
  PICK_UP,
  RETURN_TO_LINE,
  DONE
};

FreeZoneState freeZoneState = INIT;

// for completing the final demo autonomously
enum AutoState
{
  INIT_AUTO,
  WAIT_TO_START,
  DRIVE_TO_PICKUP,
  CHOOSE_BAG,
  PICKUP_BAG,
  DRIVE_TO_FIRST_SECT,
  DRIVE_TO_DROP_WITH_BAG,
  DROP_OFF_BAG,
  RETURN_FROM_DROP,
  TURN_TO_START_PREP,
  TURN_TO_START,
  GET_IN_START_ZONE,
  DONE_AUTO
};

AutoState autoState = INIT_AUTO;

//for picking up the bag
enum PickUpBagState
{
  DRIVE_TO_BAG,
  TURN_AROUND_PICKUP,
  REVERSE_PICKUP,
  RETURN_TO_LINE_PICKUP
};

PickUpBagState pickUpState = DRIVE_TO_BAG;

//for dropping the bag
enum DropBagState
{
  TURN_AROUND_D,
  ALIGN_LINE_DROP,
  BACK_UP_D,
  DRIVE_FOR_D,
  ALIGN_LINE

};

DropBagState dropBagState = TURN_AROUND_D;

//END enums +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//END declarations +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//START utility classes=============================================================

//store how far the bag is when robot start moving to it
float distAwayFromBag = 0;

/**
 * Picks up a bag from the pickup zone
 */
boolean pickUpBag()
{
  switch (pickUpState)
  {
    //drives to the bag
  case DRIVE_TO_BAG:

    //lower arm
    servo.moveDownPosition();

    //save the distance we need to move to the bag
    distAwayFromBag = ultra.getDistanceIN();

    //drive to bag
    if (drive.driveTo(5, ultra.getDistanceIN()))
    {
      pickUpState = TURN_AROUND_PICKUP;
    }
    break;

    //rotate so that the hook is facing the bag
  case TURN_AROUND_PICKUP:

    //turn
    if (drive.turn(183, DRIVE_SPEED_MED))
    {
      pickUpState = REVERSE_PICKUP;
    }
    break;

    //back into bag and lift it up
  case REVERSE_PICKUP:
    //get hook under bag handle
    if (drive.driveInches(-2, DRIVE_SPEED_FAST))
    {
      //move servo up to pick up bag
      servo.moveUpPosition();
      pickUpState = RETURN_TO_LINE_PICKUP;
    }
    break;

  //return to line
  case RETURN_TO_LINE_PICKUP:
    //return to the line and a little past it
    if (drive.driveInches(distAwayFromBag + 4, DRIVE_SPEED_MED))
    {
      //reset distAwayFromBag
      distAwayFromBag = 0;

      pickUpState = DRIVE_TO_BAG;
      //done
      return true;
    }
    break;
  }
  return false;
}

/**
 * Facing the platform got through the steps to drop off the bag
 */
boolean dropOffBag()
{
  switch (dropBagState)
  {
    //turn around so that the hook is on the platform side
  case TURN_AROUND_D:
    //turn 180 plus an addition 15 for slip compensation
    if (drive.turn(180 + 15, DRIVE_SPEED_SLOW))
    {
      dropBagState = BACK_UP_D;
    }
    break;

  //back up and drop off bag
  case BACK_UP_D:

    if (drive.driveInches(-1, DRIVE_SPEED_MED))
    {
      //move the arm to mid position to drop off bag
      servo.moveMidPosition();
      dropBagState = DRIVE_FOR_D;
    }
    break;

    //drive forward to get away from platform
  case DRIVE_FOR_D:

    if (drive.driveInches(4, DRIVE_SPEED_MED))
    {

      dropBagState = TURN_AROUND_D;
      return true;
    }
    break;
  }
  return false;
}

/**
 * picks up a bag from the free zone and returns to the line, starting from the intersection at the pickup zone
 */
boolean pickUpBagFree()
{
  //note when started robot is facing pickup zone
  switch (freeZoneState)
  {

  case INIT:
    //lower servo to prepare for pickup
    servo.moveDownPosition();

    freeZoneState = INIT_TURN_AROUND_ONE;
    break;

  //Initial turn away form line
  case INIT_TURN_AROUND_ONE:
    if (drive.turn(-PREP_ALIGN_ANGLE, DRIVE_SPEED_MED))
    {
      freeZoneState = TURN_AROUND_ONE;
    }
    break;

  //turn around so facing away from pickup zone
  case TURN_AROUND_ONE:

    //align to the line going left
    if (drive.alignToLine(DIR_LEFT, lSensor.getLeft(), lSensor.getRight()))
    {
      freeZoneState = DRIVE_FORWARD_ONE;
    }
    break;

    //drive forward a bit to center of free zone
  case DRIVE_FORWARD_ONE:
    if (drive.driveInches(5, DRIVE_SPEED_FAST))
    {
      freeZoneState = GO_TO_BAG;
    }
    break;

  //scan and find the bag and drives towards it
  case GO_TO_BAG:
    if (drive.findBag(ultra.getDistanceIN()))
    {
      freeZoneState = TURN_AROUND;
    }
    break;

  //turn around prepared to pick up
  case TURN_AROUND:
    //turn 180 + compensation around to pick up bag
    if (drive.turn(180 + 7, 120))
    {
      freeZoneState = REVERSE;
    }
    break;

  //put the arm under the bag handle
  case REVERSE:
    //reverse putting arm in the handle
    if (drive.driveInches(-2, DRIVE_SPEED_MED))
    {
      freeZoneState = PICK_UP;
    }
    break;

  //pick up bag
  case PICK_UP:
    //lift up arm
    servo.moveUpPosition();
    freeZoneState = RETURN_TO_LINE;
    break;

  //return to the line after picking up the bag
  case RETURN_TO_LINE:

    if (drive.returnFromFree(lSensor.getLeft(), lSensor.getRight()))
    {
      freeZoneState = DONE;
    }
    break;

    //reset stuff
  case DONE:
    freeZoneState = INIT;
    return true;
    break;
  }
  return false;
}

//END utility classes=++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//start functions for demo=============================================================================================

//boolean to keep track of where the robot is in the drop area
boolean inFreeZone = false;

//0 for ground, 1 for 1.5inch, 2 for 3 inch. Selector for drop zone
int dropZone = -1;

/**
 *  State machine for running autonomously for the final demo
 */
void autoFinalDemo()
{
  //track remote press
  u16_t keyPress = decoder.getKeyCode();

  switch (autoState)
  {
  case INIT_AUTO:
    //initialize variables to start state
    //drop zone
    dropZone = -1;
    //bag pick up position
    inFreeZone = false;
    //servo in middle position
    servo.moveMidPosition();

    drive.isInPrepPos = false; //TODO
    autoState = WAIT_TO_START;
    break;

  //dont start until given drop zone
  case WAIT_TO_START:
    //choose platform based on remote press
    switch (keyPress)
    {

    case remote1:
      dropZone = GROUND_PLAT;
      break;

    case remote2:
      dropZone = MEDIUM_PLAT;
      break;

    case remote3:
      dropZone = HIGH_PLAT;
      break;
    }
    if (dropZone != -1)
    {
      autoState = DRIVE_TO_PICKUP;
    }

    break;

  //follow the line until reach pick up zone
  case DRIVE_TO_PICKUP:

    //stop if either the robot reaches the intersection or it detects a bag
    if (drive.lineFollowTillLine(lSensor.getLeft(), lSensor.getRight(), lSensor.getDifference()) || ultra.getDistanceIN() < BAG_PRESENT_DEAD)
    {
      autoState = CHOOSE_BAG;
    }
    break;

  //determine where the bag is
  case CHOOSE_BAG:

    //if a bag is not detected
    if (ultra.getDistanceIN() > BAG_PRESENT_DEAD)
    {
      //the bag is in the free zone
      inFreeZone = true;
    }
    autoState = PICKUP_BAG;
    break;

  //pick up the bag
  case PICKUP_BAG:

    //choose how to pick up the bag depending on where the bag is
    if (inFreeZone == true)
    {
      //pick up bag from free zone
      if (pickUpBagFree())
      {
        autoState = DRIVE_TO_FIRST_SECT;
      }
    }
    else
    {
      //pick up bag from the pick up zone
      if (pickUpBag())
      {
        autoState = DRIVE_TO_FIRST_SECT;
      }
    }
    break;

  //NOTE: robot now facing away from pick up zone
  //drive back to intersect near start area
  case DRIVE_TO_FIRST_SECT:

    //follow line until reach the intersection
    if (drive.lineFollowTillLine(lSensor.getLeft(), lSensor.getRight(), lSensor.getDifference()))
    {
      autoState = DRIVE_TO_DROP_WITH_BAG;
    }
    break;

  //drive to the desinated drop zone, always go around the construction zone
  case DRIVE_TO_DROP_WITH_BAG:
    //drive to drop zone
    if (drive.driveToDropZone(dropZone, lSensor.getLeft(), lSensor.getRight(), lSensor.getDifference(), ultra.getDistanceIN()))
    {
      autoState = DROP_OFF_BAG;
    }

    break;

  //drop off the bag
  case DROP_OFF_BAG:
    if (dropOffBag())
    {
      autoState = RETURN_FROM_DROP;
    }
    break;

  //return to intersect near start zone depending on where the bag was dropped off
  case RETURN_FROM_DROP:

    if (drive.returnFromDropZone(dropZone, lSensor.getLeft(), lSensor.getRight(), lSensor.getDifference()))
    {
      autoState = GET_IN_START_ZONE;
    }
    break;

  //drive a little to get fully into start zone
  case GET_IN_START_ZONE:
    if (drive.driveInches(DIST_TO_START, DRIVE_SPEED_MED))
    {
      autoState = DONE_AUTO;
    }
    break;

  //do it again, like BOSS or a very persistant failure
  case DONE_AUTO:
    autoState = INIT_AUTO;
    inFreeZone = false;
    break;
  }
}

//END functions for demo+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//robot running code====================================================

/**
 * Setup before loop
 */
void setup()
{
  // put your setup code here, to run once:

  //attach hardware
  ultra.attach();
  lSensor.attach();
  servo.attach();

  //init remote decoder
  decoder.init();

  //servo in middle position
  servo.moveMidPosition();

  Serial.begin(9600);
}

/**
 * Run the code in a loop
 */
void loop()
{
  //run the demo for final
  autoFinalDemo();
}

// END robot running code+++++++++++++++++++++++++++++++++++++++++++++++++++++++
