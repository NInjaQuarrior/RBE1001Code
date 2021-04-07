#include <RBE1001Lib.h>
#include <cmath>

LeftMotor left;
RightMotor right;

//constant wheel diameter
float WHEEL_DIAMETER = 2.75f;

/**
 * drive straight a certain amount of inches
 * @param inches inches to move, negative to go backwars
 * @param speed degrees per second to move
 * 
 * */
boolean driveInches(float inches, float speed)
{
  float moveDegrees = (inches / (2 * (M_PI) * (WHEEL_DIAMETER / 2))) * 360;

  left.startMoveFor(moveDegrees, speed);
  right.moveFor(moveDegrees, speed);

  return true;
}

/**
 * turns a certain amount of degrees
 * @param degrees degrees to turn, negative to turn counter-clockwise
 * @param speed degrees per second to move
 * 
 * */
boolean turn(float degrees, float speed)
{

  float moveDegrees = 2 * degrees;
  left.startMoveFor(moveDegrees, 180);
  right.moveFor(-moveDegrees, 180);

  return true;
}

/**
 * Makes a shape
 * @param sides numbers of side for the shape
 * @param sideLength  the length of each side
 * @param speed degrees per second to move while making shape
 * 
 * */
boolean makeShape(int sides, float sideLength, float speed)
{
  float turnDegrees = 360 / sides;
  for (int i = 0; i < sides; i++)
  {
    driveInches(sideLength, speed);
    turn(turnDegrees, speed);
  }
  return true;
}

/**
 * makes a spiral shape in the shape of the side put in, ie 4 side for square spiral
 * 
 * @param baseLength the length of the first side
 * @param speed, degrees per second to move
 * @param sides number of sides before completing on spiral
 * @param spiralAmount number of complete spirals to make
 * 
 * */
boolean makeSpiral(float baseLength, float speed, int sides, int spiralAmount)
{
  float turnDegrees = 360 / sides;
  for (int i = 0; i < (sides * spiralAmount); i++)
  {
    driveInches(baseLength + (i * baseLength), speed);
    turn(turnDegrees, speed);
  }
  return true;
}

/**
 * makes a square spiral with 20 individual sides for the GRAND CHALLENGE
 * 
 * @param baseLength, base side length
 * @param speed degrees per second to move
 * */
boolean makeSquareSpiral(float baseLength, float speed)
{

  for (int i = 0; i < 20; i++)
  {
    driveInches(baseLength + (i * baseLength), speed);
    turn(90, speed);
  }
  return true;
}

void setup()
{
  // put your setup code here, to run once:
  delay(8000);
}

boolean doneThing = false;
void loop()
{
  //driveInches(5, 270);
  if (doneThing == false && makeShape(3, 12, 360))
  {
    doneThing = true;
  }
}
