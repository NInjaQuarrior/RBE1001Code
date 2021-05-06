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
//ArmServo class obkect
ArmServo servo;
//deadband for if bag in pickup zone
const float BAG_PRESENT_DEAD = 10; //TODO tune

const uint8_t IR_DETECTOR_PIN = 15;
IRDecoder decoder(15);

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
  GET_IN_START_ZONE,
  DONE_AUTO
};

AutoState autoState = INIT_AUTO;

enum PickUpBagState
{
  DRIVE_TO_BAG,
  TURN_AROUND_PICKUP,
  REVERSE_PICKUP,
  RETURN_TO_LINE_PICKUP
};

PickUpBagState pickUpState = DRIVE_TO_BAG;

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
    servo.moveDownPosition();
    distAwayFromBag = ultra.getDistanceIN();
    if (drive.driveToInches(2.5, ultra.getDistanceIN()))
    {
      pickUpState = TURN_AROUND_PICKUP;
    }
    break;
    //hook facing bag
  case TURN_AROUND_PICKUP:
    if (drive.turn(180, 180))
    {
      pickUpState = REVERSE_PICKUP;
    }
    break;
    //back into bag and lift it up
  case REVERSE_PICKUP:
    if (drive.driveInches(-2.5, 270))
    {
      servo.moveUpPosition();
      pickUpState = RETURN_TO_LINE_PICKUP;
    }
    break;
  //return to line
  case RETURN_TO_LINE_PICKUP:
    if (drive.driveInches(distAwayFromBag + 4, 180))
    {
      distAwayFromBag = 0;
      pickUpState = DRIVE_TO_BAG;
      return true;
    }
    break;
  }
  return false;
}

/**
 * Lowers the servo and drives away
 */
boolean dropOffBag()
{
  //maybe go to mid for dropping off
  servo.moveDownPosition();
  if (drive.driveInches(-3, 180))
  {
    return true;
  }
  return false;
}

/**
 * picks up a bag from the free zone and returns to the line
 */
boolean pickUpBagFree()
{

  switch (freeZoneState)
  {
  //note when started robot is facing pickup zone
  case INIT:
    servo.moveDownPosition();
    freeZoneState = INIT_TURN_AROUND_ONE;
    break;
  //Initial turn away form line
  case INIT_TURN_AROUND_ONE:
    if (drive.turn(-45, 180)) //TODO test
    {
      freeZoneState = TURN_AROUND_ONE;
    }
    break;
  //turn around aso facing away from pickup zone
  case TURN_AROUND_ONE:
    if (drive.alignToLine(-1, lSensor.getLeft(), lSensor.getRight()))
    {
      freeZoneState = DRIVE_FORWARD_ONE;
    }
    break;
    //drive forward a bit to center of free zon
  case DRIVE_FORWARD_ONE:
    if (drive.driveInches(5, 270)) //TODO test
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
    //turn around to pick up bag
    if (drive.turn(190, 180))
    {
      freeZoneState = REVERSE;
    }
    break;
  //put the arm under the bag handle
  case REVERSE:
    //reverse putting arm in the handle
    if (drive.driveInches(-2.5, 270))
    {
      freeZoneState = PICK_UP;
    }
    break;
  //pick up bag
  case PICK_UP:
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

//toggle boolean for using remote
boolean enableTeleopRemote = false;

//boolean to keep track of where the robot is in the drop area
boolean inFreeZone = false;
//0 for ground, 1 for 1.5inch, 2 for 3 inch. Selector for drop zone
int dropZone = -1;

void autoFinalDemo()
{
  int16_t keyPress = decoder.getKeyCode();

  //if hit play give control to remote
  if (keyPress == remotePlayPause)
  {
    enableTeleopRemote = !enableTeleopRemote;
  }

  //if not in remote mode
  if (enableTeleopRemote == false)
  {
    switch (autoState)
    {
    case INIT_AUTO:
      dropZone = -1;
      inFreeZone = false;
      servo.moveMidPosition();
      break;
    //dont start until given drop zone
    case WAIT_TO_START:
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
      case DEFAULT:
        break;
      }
      if (dropZone != -1)
      {
        autoState = DRIVE_TO_PICKUP;
      }

      break;
    //follow the line until reach pick up zone
    case DRIVE_TO_PICKUP:
      if (drive.lineFollowTillLine(lSensor.getLeft(), lSensor.getRight(), lSensor.getDifference()) || ultra.getDistanceIN() < BAG_PRESENT_DEAD)
      {
        autoState = CHOOSE_BAG;
      }
      break;
    //determine where the bag is
    case CHOOSE_BAG:
      if (ultra.getDistanceIN() > BAG_PRESENT_DEAD)
      {
        inFreeZone = true;
      }
      autoState = PICKUP_BAG;
      break;
    //pick up the bag
    case PICKUP_BAG:
      if (inFreeZone == true)
      {
        if (pickUpBagFree())
        {
          autoState = DRIVE_TO_FIRST_SECT;
        }
      }
      else
      {
        if (pickUpBag())
        {
          autoState = DRIVE_TO_FIRST_SECT;
        }
      }
      break;
    //NOTE: robot now facing away from pick up zone
    //drive back to start area
    case DRIVE_TO_FIRST_SECT:
      if (drive.lineFollowTillLine(lSensor.getLeft(), lSensor.getRight(), lSensor.getDifference()))
      {

        autoState = DRIVE_TO_DROP_WITH_BAG;
      }
      break;
    //drive to the desinated dropzon
    case DRIVE_TO_DROP_WITH_BAG:
      if (drive.driveToDropZone(dropZone, lSensor.getLeft(), lSensor.getRight(), lSensor.getDifference(), ultra.getDistanceIN()))
      {
        autoState = DROP_OFF_BAG;
      }
      break;
    //drop off the back
    case DROP_OFF_BAG:
      if (dropOffBag())
      {
        autoState = RETURN_FROM_DROP;
      }
      break;
    //return to intersect near start zone
    case RETURN_FROM_DROP:
      if (drive.returnFromDropZone(dropZone, lSensor.getLeft(), lSensor.getRight(), lSensor.getDifference()))
      {
        autoState = GET_IN_START_ZONE;
      }
      break;
    //drive a little to get into start zone
    case GET_IN_START_ZONE:
      if (drive.driveInches(7, 270))
      {
        autoState = DONE_AUTO;
      }

      break;
    //do it again, like BOSS or a very persistant failure
    case DONE_AUTO:
      autoState = INIT_AUTO;
      break;
    }
  }
  else
  {
    //remote control
    drive.teleOpAuto(keyPress);
  }
}

//END functions for demo+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//robot running code====================================================
void setup()
{
  // put your setup code here, to run once:
  //attach hardware
  ultra.attach();
  lSensor.attach();
  servo.attach();

  Serial.begin(9600);
  decoder.init();
  servo.moveMidPosition();

  delay(4000);
}

//boolean used to only run a function once
boolean doneThing = false;

void loop()
{
  //do the thing
  autoFinalDemo();
}

// END robot running code+++++++++++++++++++++++++++++++++++++++++++++++++++++++
