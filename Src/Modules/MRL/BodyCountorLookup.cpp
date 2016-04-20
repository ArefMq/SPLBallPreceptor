/**
 * @file BodyCountorLookup.cpp
 * @author Ali Sharpassand
 * @date Apr 2016
 */

#include "BodyCountorLookup.h"

BodyCountorLookup::BodyCountorLookup() :
ratioX(1.0641),
ratioY(0.83147),
A(2),
B(-2.f)
{
}

BodyCountorLookup::~BodyCountorLookup()
{
}

bool BodyCountorLookup::check(int x, int y, Angle pan, Angle tilt, int width, int height)
{
  double theX = - ratioX * (x - width /2) / width + pan.toFloat();
  double theY = - ratioY * (y - height /2) / height + tilt.toFloat();

  return isUnderLine(theX, theY) || isUnderLine(-theX, theY);
}

bool BodyCountorLookup::isUnderLine(double x, double y)
{
  return y < A * x + B;

}
