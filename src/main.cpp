#include <RBE1001Lib.h>
#include <IRdecoder.h>
#include <RemoteConstants.h>
#include <Drive.cpp>
#include <Ultrasonic.cpp>
#include <LineSense.cpp>
#include <ArmServo.cpp>
#include <ESP32Servo.h>
#include <ESP32AnalogRead.h>

//ultrasonic class object
Ultrasonic ultra;
//Drive class object
Drive drive;
//Linesense class object
LineSense lSensor;
//ArmServo class obkect
ArmServo servo;

const uint8_t IR_DETECTOR_PIN = 15;
IRDecoder decoder(15);

enum FreeZoneState
{
  INIT,
  GO_TO_BAG,
  TURN_AROUND,
  REVERSE,
  PICK_UP,
  RETURN_TO_LINE,
  DONE
};

FreeZoneState freeZoneState = INIT;

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

void autonomous()
{
  //TODO
}

boolean pickUpBagFree()
{

  switch (freeZoneState)
  {
  case INIT:
    servo.moveDownPosition();
    freeZoneState = GO_TO_BAG;
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

//boolean used to only run a funtion once
boolean doneThing = false;
void loop()
{

  //code for controlling with remote =======================
  // Check for a key press on the remote
  int16_t keyPress = decoder.getKeyCode();

  //drive.teleOp(keyPress); // is commented out to not interfere with bag pick up
  //========================================================

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
  //================================================================
}
