#include <RBE1001Lib.h>
#include <Drive.cpp>
#include <Ultrasonic.cpp>
#include <LineSense.cpp>

//Ultrasonic ultra;
Ultrasonic ultra;
Drive drive;
LineSense lSensor;

void setup()
{
  // put your setup code here, to run once:
  ultra.attach();
  lSensor.attach();
  Serial.begin(9600);
  delay(8000);
}

boolean doneThing = false;
void loop()
{

  drive.followLine(lSensor.getDifference(), lSensor.getLeft(), lSensor.getRight());

  //drive.driveToInches(11, ultra.getDistanceIN());
  //  if (doneThing == false && drive.driveToInches(11, ultra.getDistanceIN()))
  // {
  //   doneThing = true;
  // }
}
