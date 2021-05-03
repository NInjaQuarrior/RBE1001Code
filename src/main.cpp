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

boolean doneThing = false;
void loop()
{
  // Check for a key press on the remote
  int16_t keyPress = decoder.getKeyCode();

  // If a valid key is pressed, print out its value
  if (keyPress >= 0)
  {
    Serial.println("Key: " + String(keyPress));
    drive.teleOp(keyPress);
  }
}

void autonomous()
{
  
}

float bigDist = 0;
boolean gotDist = false;

boolean pickUpBagFree()
{
  if (gotDist == false)
  {
    bigDist = ultra.getDistanceIN();
    gotDist = true;
  }
  switch (freeZoneState)
  {
  case INIT:
    servo.moveDownPosition();
    if (drive.turn(45, 270))
    {
      freeZoneState = GO_TO_BAG;
    }
    break;
  case GO_TO_BAG:
    if (drive.findBag(bigDist, ultra.getDistanceIN()))
    {
      freeZoneState = PICK_UP;
    }
    break;
  case PICK_UP:
    servo.moveUpPosition();
    freeZoneState = RETURN_TO_LINE;
    break;
  case RETURN_TO_LINE:
    if (drive.returnFromFree())
    {
      freeZoneState = DONE;
    }
    break;
  case DONE:
    freeZoneState = INIT;
    gotDist = false;
    bigDist = 0;
    return true;
    break;
  }
  return false;
}
