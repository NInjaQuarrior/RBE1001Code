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

float BAG_PRESENT_DEAD = 10;

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
  DRIVE_TO_SECOND_SECT,
  DRIVE_TO_DROP_WITH_BAG,
  DROP_OFF_BAG,
  RETURN_FROM_DROP,
  RETURN_TO_START,
  GET_IN_START_ZONE,
  DONE_AUTO
};

AutoState autoState = INIT_AUTO;

enum PickUpBagState
{
  DRIVE_TO_BAG,
  TURN_AROUND_PICKUP,
  RETURN_TO_LINE_PICKUP
};

PickUpBagState pickUpState = DRIVE_TO_BAG;

//END declarations +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//START utility classes=============================================================

float distAwayFromBag = 0;
boolean pickUpBag()
{
  switch (pickUpState)
  {
  case DRIVE_TO_BAG:
    servo.moveDownPosition();
    distAwayFromBag = ultra.getDistanceIN();
    if (drive.driveToInches(2.5, ultra.getDistanceIN()))
    {
      servo.moveUpPosition();
      pickUpState = TURN_AROUND_PICKUP;
    }
    break;
  case TURN_AROUND_PICKUP:
    if (drive.turn(180, 180))
    {
      pickUpState = RETURN_TO_LINE_PICKUP;
    }
    break;
  case RETURN_TO_LINE_PICKUP:
    if (drive.driveInches(distAwayFromBag, 180))
    {
      distAwayFromBag = 0;
      pickUpState = DRIVE_TO_BAG;
      return true;
    }
    break;
  }

  return false;
}

boolean dropOffBag()
{
  //TODO maybe go mid position for platforms, if so need switch taking in the dropzon and setting to mid if plat 1 or 2

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
  case INIT:
    servo.moveDownPosition();
    freeZoneState = INIT_TURN_AROUND_ONE;
    break;
  case INIT_TURN_AROUND_ONE:
    if (drive.turn(-45, 180)) //TODO test
    {
      freeZoneState = TURN_AROUND_ONE;
    }
    break;
  case TURN_AROUND_ONE:
    if (drive.alignToLine(-1, lSensor.getLeft(), lSensor.getRight()))
    {
      freeZoneState = DRIVE_FORWARD_ONE;
    }
    break;
  case DRIVE_FORWARD_ONE:
    if (drive.driveInches(5, 270)) //TODO test
    {
      freeZoneState = GO_TO_BAG;
    }
    break;
  case GO_TO_BAG:
    if (drive.findBag(ultra.getDistanceIN()))
    {
      freeZoneState = TURN_AROUND;
    }
    break;
  case TURN_AROUND:
    //turn around to pick up bag
    if (drive.turn(190, 180))
    {
      freeZoneState = REVERSE;
    }
    break;
  case REVERSE:
    //reverse putting arm in the handle
    if (drive.driveInches(-2.5, 270))
    {
      freeZoneState = PICK_UP;
    }
    break;
  case PICK_UP:
    servo.moveUpPosition();
    freeZoneState = RETURN_TO_LINE;
    break;
  case RETURN_TO_LINE:
    if (drive.returnFromFree(lSensor.getLeft(), lSensor.getRight()))
    {
      freeZoneState = DONE;
    }
    break;
  case DONE:
    freeZoneState = INIT;
    return true;
    break;
  }
  return false;
}

//END utility classes=++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//start functions for demo=============================================================================================

boolean enableTeleopRemote = false;

boolean inFreeZone = false;
//0 for ground, 1 for 1.5inch, 2 for 3 inch
int dropZone = -1;

void autonomous()
{
  int16_t keyPress = decoder.getKeyCode();

  if (keyPress == remotePlayPause)
  {
    enableTeleopRemote = !enableTeleopRemote;
  }

  if (enableTeleopRemote == false)
  {
    switch (autoState)
    {
    case INIT_AUTO:
      dropZone = -1;
      inFreeZone = false;
      servo.moveMidPosition();
      break;
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
    case DRIVE_TO_PICKUP:
      if (drive.lineFollowTillLine(lSensor.getLeft(), lSensor.getRight(), lSensor.getDifference()) || ultra.getDistanceIN() < BAG_PRESENT_DEAD)
      {
        autoState = CHOOSE_BAG;
      }
      break;
    case CHOOSE_BAG:
      if (ultra.getDistanceIN() > BAG_PRESENT_DEAD)
      {
        inFreeZone = true;
      }
      autoState = PICKUP_BAG;
      break;
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
    case DRIVE_TO_FIRST_SECT:
      if (drive.lineFollowTillLine(lSensor.getLeft(), lSensor.getRight(), lSensor.getDifference()))
      {
        drive.driveInches(1, 270);
        autoState = DRIVE_TO_SECOND_SECT;
      }
      break;
    case DRIVE_TO_SECOND_SECT:
      if (drive.lineFollowTillLine(lSensor.getLeft(), lSensor.getRight(), lSensor.getDifference()))
      {
        autoState = DRIVE_TO_DROP_WITH_BAG;
      }
      break;
    case DRIVE_TO_DROP_WITH_BAG:
      if (drive.driveToDropZone(dropZone, lSensor.getLeft(), lSensor.getRight(), lSensor.getDifference(), ultra.getDistanceIN()))
      {
        autoState = DROP_OFF_BAG;
      }
      break;
    case DROP_OFF_BAG:
      if (dropOffBag())
      {
        autoState = RETURN_FROM_DROP;
      }
      break;
    case RETURN_FROM_DROP:
      if (drive.returnFromDropZone(dropZone, lSensor.getLeft(), lSensor.getRight(), lSensor.getDifference()))
      {
        autoState = RETURN_TO_START;
      }
      break;
    case RETURN_TO_START:
      if (drive.lineFollowTillLine(lSensor.getLeft(), lSensor.getRight(), lSensor.getDifference()))
      {
        autoState = GET_IN_START_ZONE;
      }

      break;
    case GET_IN_START_ZONE:
      if (drive.driveInches(7, 270))
      {
        autoState = DONE_AUTO;
      }

      break;
    case DONE_AUTO:
      autoState = INIT_AUTO;
      break;
    }
  }
  else
  {
    drive.teleOpAuto(keyPress);
  }
}

//END functions for demo+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//robot running code====================================================
void setup()
{
  // put your setup code here, to run once:
  ultra.attach();
  lSensor.attach();
  servo.attach();
  Serial.begin(9600);
  decoder.init();
  servo.moveMidPosition();
  delay(4000);
}

//boolean used to only run a funtion once
boolean doneThing = false;
void loop()
{

  //code for controlling with remote =======================
  // Check for a key press on the remote
  //int16_t keyPress = decoder.getKeyCode();

  //drive.teleOp(keyPress); // is commented out to not interfere with bag pick up
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++

  //code for picking up bag from free zone and returning==============
  if (doneThing == false)
  {
    //leave the line, pick up the bag and return to the line
    if (pickUpBagFree())
    {
      doneThing = true;
    }
  }
  else
  {
    //after picking up the bag and returning to the line, follow the line
    drive.followLine(lSensor.getDifference(), lSensor.getLeft(), lSensor.getRight());
  }
  //+++++++++++++++++++++++++++++++++++++++++++++++++
}

// END robot running code+++++++++++++++++++++++++++++++++++++++++++++++++++++++
