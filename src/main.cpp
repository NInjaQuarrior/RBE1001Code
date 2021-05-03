#include <RBE1001Lib.h>
#include <IRdecoder.h>
#include <RemoteConstants.h>
#include <Drive.cpp>
#include <Ultrasonic.cpp>
#include <LineSense.cpp>
#include <ArmServo.cpp>
#include <ESP32Servo.h>
#include <ESP32AnalogRead.h>

Ultrasonic ultra;
Drive drive;
LineSense lSensor;
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
    if (drive.turn(190, 270))
    {
      freeZoneState = REVERSE;
    }
    break;
  case REVERSE:
    if (drive.driveInches(-2, 270))
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

boolean doneThing = false;
void loop()
{
  // Check for a key press on the remote
  int16_t keyPress = decoder.getKeyCode();

  //Serial.printf("%f", ultra.getDistanceIN());
  //Serial.println("");
  if (doneThing == false)
  {
    if (pickUpBagFree())
    {
      doneThing = true;
    }
  }
  else
  {
    drive.followLine(lSensor.getDifference(), lSensor.getLeft(), lSensor.getRight());
  }
  // If a valid key is pressed, print out its value
  // if (keyPress >= 0)
  // {
  //   Serial.println("Key: " + String(keyPress));
  //   drive.teleOp(keyPress);
  // }
}
