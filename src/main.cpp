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

//distance to move the robot into the start zone from the nearest intersection
const float DIST_TO_START = 7.0f;

//pin for ir detector
const uint8_t IR_DETECTOR_PIN = 15;

IRDecoder decoder(15);

//START enums for state machines ===================================

//for picking up a bag from freezon
enum FreeZoneState
{
  INIT_FREE,
  PREP_TURN_AROUND_ONE_FREE,
  TURN_AROUND_ONE_FREE,
  DRIVE_FORWARD_FREE,
  GO_TO_BAG_FREE,
  TURN_AROUND_TWO_FREE,
  REVERSE_FREE,
  PICK_UP_FREE,
  RETURN_TO_LINE_FREE,
  DONE_FREE
};

FreeZoneState freeZoneState = INIT_FREE;

// for completing the final demo autonomously
enum AutoState
{
  INIT_AUTO,
  WAIT_TO_START_AUTO,
  DRIVE_TO_PICKUP_AUTO,
  CHOOSE_BAG_AUTO,
  PICKUP_BAG_AUTO,
  DRIVE_TO_FIRST_SECT_AUTO,
  DRIVE_TO_DROP_WITH_BAG_AUTO,
  DROP_OFF_BAG_AUTO,
  RETURN_FROM_DROP_AUTO,
  GET_IN_START_ZONE_AUTO,
  DONE_AUTO
};

AutoState autoState = INIT_AUTO;

//for picking up the bag
enum PickUpBagState
{
  DRIVE_TO_BAG_ZONE,
  TURN_AROUND_PICKUP_ZONE,
  REVERSE_PICKUP_ZONE,
  RETURN_TO_LINE_PICKUP_ZONE
};

PickUpBagState pickUpState = DRIVE_TO_BAG_ZONE;

//for dropping the bag
enum DropBagState
{
  TURN_AROUND_D,
  BACK_UP_D,
  DRIVE_FOR_D

};

DropBagState dropBagState = TURN_AROUND_D;

//END enums +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//START sensor value variables
float leftSense;
float rightSense;
float error;

//ultrasonic
float curDistIN;
float curDistCM;

//remote
u16_t keyPress;

//END sensor value varables

//END declarations +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//START utility classes=============================================================

void updateValues()
{

  //light sensor
  leftSense = lSensor.getLeft();
  rightSense = lSensor.getRight();
  error = lSensor.getDifference();

  //ultrasonic
  curDistIN = ultra.getDistanceIN();
  curDistCM = ultra.getDistanceCM();

  //remote
  keyPress = decoder.getKeyCode();
}

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
  case DRIVE_TO_BAG_ZONE:

    //lower arm
    servo.moveDownPosition();

    //save the distance we need to move to the bag
    distAwayFromBag = curDistIN;

    //drive to bag
    if (drive.driveTo(5, curDistIN))
    {
      pickUpState = TURN_AROUND_PICKUP_ZONE;
    }
    break;

    //rotate so that the hook is facing the bag
  case TURN_AROUND_PICKUP_ZONE:

    //turn
    if (drive.turn(183, DRIVE_SPEED_MED))
    {
      pickUpState = REVERSE_PICKUP_ZONE;
    }
    break;

    //back into bag and lift it up
  case REVERSE_PICKUP_ZONE:
    //get hook under bag handle
    if (drive.driveInches(-2, DRIVE_SPEED_FAST))
    {
      //move servo up to pick up bag
      servo.moveUpPosition();
      pickUpState = RETURN_TO_LINE_PICKUP_ZONE;
    }
    break;

  //return to line
  case RETURN_TO_LINE_PICKUP_ZONE:
    //return to the line and a little past it
    if (drive.driveInches(distAwayFromBag + 4, DRIVE_SPEED_MED))
    {
      //reset distAwayFromBag
      distAwayFromBag = 0;

      pickUpState = DRIVE_TO_BAG_ZONE;
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

  case INIT_FREE:
    //lower servo to prepare for pickup
    servo.moveDownPosition();

    freeZoneState = PREP_TURN_AROUND_ONE_FREE;
    break;

  //Initial turn away form line
  case PREP_TURN_AROUND_ONE_FREE:
    if (drive.turn(-PREP_ALIGN_ANGLE, DRIVE_SPEED_MED))
    {
      freeZoneState = TURN_AROUND_ONE_FREE;
    }
    break;

  //turn around so facing away from pickup zone
  case TURN_AROUND_ONE_FREE:

    //align to the line going left
    if (drive.alignToLine(DIR_LEFT, leftSense, rightSense))
    {
      freeZoneState = DRIVE_FORWARD_FREE;
    }
    break;

    //drive forward a bit to center of free zone
  case DRIVE_FORWARD_FREE:
    if (drive.driveInches(5, DRIVE_SPEED_FAST))
    {
      freeZoneState = GO_TO_BAG_FREE;
    }
    break;

  //scan and find the bag and drives towards it
  case GO_TO_BAG_FREE:
    if (drive.findBag(curDistIN))
    {
      freeZoneState = TURN_AROUND_TWO_FREE;
    }
    break;

  //turn around prepared to pick up
  case TURN_AROUND_TWO_FREE:
    //turn 180 + compensation around to pick up bag
    if (drive.turn(180 + 7, TURN_SPEED_SLOW))
    {
      freeZoneState = REVERSE_FREE;
    }
    break;

  //put the arm under the bag handle
  case REVERSE_FREE:
    //reverse putting arm in the handle
    if (drive.driveInches(-2, DRIVE_SPEED_MED))
    {
      freeZoneState = PICK_UP_FREE;
    }
    break;

  //pick up bag
  case PICK_UP_FREE:
    //lift up arm
    servo.moveUpPosition();
    freeZoneState = RETURN_TO_LINE_FREE;
    break;

  //return to the line after picking up the bag
  case RETURN_TO_LINE_FREE:

    if (drive.returnFromFree(leftSense, rightSense))
    {
      freeZoneState = DONE_FREE;
    }
    break;

    //reset stuff
  case DONE_FREE:
    freeZoneState = INIT_FREE;
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

    autoState = WAIT_TO_START_AUTO;
    break;

  //dont start until given drop zone
  case WAIT_TO_START_AUTO:
    //choose platform based on remote press
    switch (keyPress)
    {

    case remote1:
      dropZone = 0;
      break;

    case remote2:
      dropZone = 1;
      break;

    case remote3:
      dropZone = 2;
      break;
    }
    if (dropZone != -1)
    {
      autoState = DRIVE_TO_PICKUP_AUTO;
    }

    break;

  //follow the line until reach pick up zone
  case DRIVE_TO_PICKUP_AUTO:

    //stop if either the robot reaches the intersection or it detects a bag
    if (drive.lineFollowTillLine(leftSense, rightSense, error) || curDistIN < BAG_PRESENT_DEAD)
    {
      autoState = CHOOSE_BAG_AUTO;
    }
    break;

  //determine where the bag is
  case CHOOSE_BAG_AUTO:

    //if a bag is not detected
    if (curDistIN > BAG_PRESENT_DEAD)
    {
      //the bag is in the free zone
      inFreeZone = true;
    }
    autoState = PICKUP_BAG_AUTO;
    break;

  //pick up the bag
  case PICKUP_BAG_AUTO:

    //choose how to pick up the bag depending on where the bag is
    if (inFreeZone == true)
    {
      //pick up bag from free zone
      if (pickUpBagFree())
      {
        autoState = DRIVE_TO_FIRST_SECT_AUTO;
      }
    }
    else
    {
      //pick up bag from the pick up zone
      if (pickUpBag())
      {
        autoState = DRIVE_TO_FIRST_SECT_AUTO;
      }
    }
    break;

  //NOTE: robot now facing away from pick up zone
  //drive back to intersect near start area
  case DRIVE_TO_FIRST_SECT_AUTO:

    //follow line until reach the intersection
    if (drive.lineFollowTillLine(leftSense, rightSense, error))
    {
      autoState = DRIVE_TO_DROP_WITH_BAG_AUTO;
    }
    break;

  //drive to the desinated drop zone, always go around the construction zone
  case DRIVE_TO_DROP_WITH_BAG_AUTO:
    //drive to drop zone
    if (drive.driveToDropZone(dropZone, leftSense, rightSense, error, curDistIN))
    {
      autoState = DROP_OFF_BAG_AUTO;
    }

    break;

  //drop off the bag
  case DROP_OFF_BAG_AUTO:
    if (dropOffBag())
    {
      autoState = RETURN_FROM_DROP_AUTO;
    }
    break;

  //return to intersect near start zone depending on where the bag was dropped off
  case RETURN_FROM_DROP_AUTO:

    if (drive.returnFromDropZone(dropZone, leftSense, rightSense, error))
    {
      autoState = GET_IN_START_ZONE_AUTO;
    }
    break;

  //drive a little to get fully into start zone
  case GET_IN_START_ZONE_AUTO:
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
  //update sensor values
  updateValues();
  //run the demo for final
  autoFinalDemo();
}

// END robot running code+++++++++++++++++++++++++++++++++++++++++++++++++++++++
