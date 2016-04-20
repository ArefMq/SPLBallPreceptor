/**
 * @file BodyCountorLookup.h
 * @author Ali Sharpassand
 * @date Apr 2016
 */

#ifndef SRC_MODULES_PERCEPTION_MRL_BODYCOUNTORLOOKUP_H_
#define SRC_MODULES_PERCEPTION_MRL_BODYCOUNTORLOOKUP_H_

#include "Tools/Math/Angle.h"

class BodyCountorLookup
{
public:
  BodyCountorLookup();
  virtual ~BodyCountorLookup();

  bool check(int x, int y, Angle pan, Angle tilt, int width, int height);

private:
  float A, B;
  double ratioX, ratioY;
  bool isUnderLine(double x, double y);

};

#endif /* SRC_MODULES_PERCEPTION_MRL_BODYCOUNTORLOOKUP_H_ */
