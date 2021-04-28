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
    drive.setButton(keyPress);
  }
}
