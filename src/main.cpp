#include <RBE1001Lib.h>
#include <Drive.cpp>
#include <Ultrasonic.cpp>

Ultrasonic ultra;
Drive drive;

void setup()
{
  // put your setup code here, to run once:
  ultra.attach();
  delay(8000);
}

boolean doneThing = false;
void loop()
{

  if (ultra.getDistanceIN() < 11)
  {
    drive.setEffort(1);
  }
  else
  {
    drive.setEffort(0);
  }
  //driveInches(5, 270);
  // if (doneThing == false && makeSquareSpiral(5, 450))
  // {
  //   doneThing = true;
  // }
}
