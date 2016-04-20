/**
 * @file BallPerceptor.cpp
 * This file declares a module that provides a white ball percept without using color tables.
 * The ball center / radius calculation algorithm is based on the BallSpecialist in GT2005.
 * @author <a href="mailto:a.moqadam@mrl-spl.ir">Aref Moqadam</a>
 * @author Ali Sharpassand
 * @date Mar. 2016
 */

#include "BallPerceptor.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Matrix.h"
#include "Tools/Streams/InStreams.h"
#include "Representations/Perception/BallSpot.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/BallModel.h"
#include "Tools/Math/Geometry.h"
#include "Representations/Perception/BallSpots.h"

#include <iostream>
#include <algorithm>
#include <cmath>

MAKE_MODULE(BallPerceptor, Perception)

BallPerceptor::BallPerceptor() :
  edgeDetection(theImage, theColorReference),
  houghTrans(edgeDetection),
  minWhitePercentage(0.35),
  minNonGreenPercentage(0.7),
  blackThre(90),
  minBlackPercentage(0.04),
  maxBlackPercentage(0.5),
  blackCheckRatio(0.65)
{
}

void BallPerceptor::update(BallPercept& ballPercept)
{
  DECLARE_DEBUG_DRAWING("module:BallPerceptor:bodyRejections", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:BallPerceptor:Cross", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:BallPerceptor:blacks", "drawingOnImage");

  MODIFY("module:BallPerceptor:minWhitePercentage", minWhitePercentage);
  MODIFY("module:BallPerceptor:minNonGreenPercentage", minNonGreenPercentage);
  MODIFY("module:BallPerceptor:minBlackPercentage", minBlackPercentage);
  MODIFY("module:BallPerceptor:maxBlackPercentage", maxBlackPercentage);
  MODIFY("module:BallPerceptor:blackThre", blackThre);
  MODIFY("module:BallPerceptor:blackblackCheckRatio", blackCheckRatio);

  COMPLEX_DRAWING("module:BallPerceptor:blacks", {
	  for(int x = 0; x < theImage.width; x++)
		  for(int y = 0; y < theImage.height; y++)
			  if(isBlack(x, y))
				  DOT("module:BallPerceptor:blacks", x, y, ColorClasses::red, ColorClasses::red);
  });

  ballPercept.ballWasSeen = false;
  ballPercept.status = BallPercept::notSeen;

  edgeDetection.calculateEdge(theCameraInfo,
      theFilteredJointData.angles[JointData::HeadYaw],
      theFilteredJointData.angles[JointData::HeadPitch]);

  INIT_DEBUG_IMAGE(edgeImage,edgeDetection);
  SEND_DEBUG_IMAGE(edgeImage);

  houghTrans.update();

  for (const auto& v : houghTrans.extractedPoints())
  {
    int cx = v.x * edgeDetection.step();
    int cy = v.y * edgeDetection.step();
    int r = v.z * edgeDetection.step();

    CIRCLE("module:BallPerceptor:Cross", cx, cy, r, 2, Drawings::bs_solid, ColorClasses::black, Drawings::bs_null, ColorClasses::black);

    if (!refineEdges(cx, cy, r))
      continue;

    if (!checkWhitePercentage(cx, cy, r))
      continue;

    if (!checkBlackPercentage(cx, cy, r * blackCheckRatio))
      continue;

    bool isInBody = false;
    for(float i = 0;  i < 2 * M_PI; i += M_PI / 8)
    {
      const int x = r * cos(i) + cx;
      const int y = r * sin(i) + cy;

      int minY = y;
      theBodyContour.clipBottom(x, minY);
      DOT("module:BallPerceptor:bodyRejections", x, y, ColorClasses::red, ColorClasses::red);
      DOT("module:BallPerceptor:bodyRejections", x, minY, ColorClasses::blue, ColorClasses::blue);
      if(y > minY)
      {
        isInBody = true;
        break;
      }
    }

    if(isInBody)
      continue;

    //-- Check if the ball is totally inside the field
    if (cy-r < theFieldBoundary.getBoundaryY(cx))
      continue;

    //-- Check ball radius
    #define BALL_WIDTH_2 50   //-- Half of the ball width // [FIXME] : this should be a config
    #define BALL_WIDTH   100  //-- ball width             // [FIXME] : this should be a config

    Vector3<> projectedLeft,projectedRight;
    bool isProjected = Geometry::calculatePointOnField(Vector2<>(cx-r, cy), BALL_WIDTH_2, theCameraMatrix, theCameraInfo, projectedLeft) &&
                       Geometry::calculatePointOnField(Vector2<>(cx+r, cy), BALL_WIDTH_2, theCameraMatrix, theCameraInfo, projectedRight);

    if (abs(abs(projectedLeft.y - projectedRight.y) - BALL_WIDTH) > 50)
    {
      DRAWTEXT("module:BallPerceptor:Cross", cx, -cy, 10, ColorClasses::yellow, projectedLeft.y - projectedRight.y);
      CIRCLE("module:BallPerceptor:Cross", cx, cy, r, 2, Drawings::bs_solid, ColorClasses::yellow, Drawings::bs_null, ColorClasses::yellow);
      continue;
    }

    CIRCLE("module:BallPerceptor:Cross", cx, cy, r, 2, Drawings::bs_solid, ColorClasses::red, Drawings::bs_null, ColorClasses::red);

    //-- Exporting results
    if (theCameraMatrix.isValid)
    {
      ballPercept.positionInImage = Vector2<>(cx, cy);
      ballPercept.radiusInImage = r;
      Geometry::Circle c;
      if(calculateBallOnField(ballPercept))
      {
        ballPercept.status = BallPercept::seen;
        ballPercept.ballWasSeen = true;
      }
//      if (Geometry::calculateBallInImage(ballPercept.positionInImage, theCameraMatrix, theCameraInfo, ballPercept.radiusInImage, c))
//      {
//        ballPercept.status = BallPercept::seen;
//        ballPercept.ballWasSeen = true;
//        ballPercept.relativePositionOnField = Vector2<>(c.center.y, c.center.x);
//        return;
//      }
    }
  }//-- End of every extracted points
}

//-- This function has to be on the global space
bool comparePoints(const Vector3i& a, const Vector3i& b)
{
  return a.z < b.z;
}

bool BallPerceptor::refineEdges(int& x, int& y, int& r)
{
  // [TODO] : optimize this function. This can be done by binary search or something to reduce the complexity

  //   p7 p0  p1
  //    \ | /
  //     \|/
  // p6 --O-- p2
  //     /|\
  //    / | \
  //   p5 p4 p3

  // [FIXME] : do something about data types
  Vector2i p0 = Vector2i(x,   y-r); //-- Vertical
  Vector2i p1 = Vector2i(x+r, y-r);
  Vector2i p2 = Vector2i(x+r, y);   //-- Horizontal
  Vector2i p3 = Vector2i(x+r, y+r);
  Vector2i p4 = Vector2i(x  , y+r); //-- Vertical
  Vector2i p5 = Vector2i(x-r, y+r);
  Vector2i p6 = Vector2i(x-r, y);   //-- Horizontal
  Vector2i p7 = Vector2i(x-r, y-r);

  std::vector<Vector3i> points; // x, y, distance from center

  for (; isNotGreenChecked(p0.x, p0.y); --p0.y); points.push_back(Vector3i(p0.x, p0.y, y-p0.y));
  for (; isNotGreenChecked(p2.x, p2.y); ++p2.x); points.push_back(Vector3i(p2.x, p2.y, p2.x-x));
  for (; isNotGreenChecked(p4.x, p4.y); ++p4.y); points.push_back(Vector3i(p4.x, p4.y, p4.y-y));
  for (; isNotGreenChecked(p6.x, p6.y); --p6.x); points.push_back(Vector3i(p6.x, p6.y, x-p6.x));

  for (; isNotGreenChecked(p1.x, p1.y); --p1.y, ++p1.x); points.push_back(Vector3i(p1.x, p1.y, y-p1.y));
  for (; isNotGreenChecked(p3.x, p3.y); ++p3.y, ++p3.x); points.push_back(Vector3i(p3.x, p3.y, p3.y-y));
  for (; isNotGreenChecked(p5.x, p5.y); ++p5.y, --p5.x); points.push_back(Vector3i(p5.x, p5.y, p5.y-y));
  for (; isNotGreenChecked(p7.x, p7.y); --p7.y, --p7.x); points.push_back(Vector3i(p7.x, p7.y, y-p7.y));

//  std::sort(points.begin(), points.end(), comparePoints);
//  //-- Remove 3 most outliers always
//  for (unsigned i=0; i<3; ++i)
//    points.pop_back();

  int maxX = points.at(0).x,
      minX = points.at(0).x,
      maxY = points.at(0).y,
      minY = points.at(0).y;

  for (const auto& p : points)
  {
    if (maxX < p.x)
      maxX = p.x;
    if (minX > p.x)
      minX = p.x;

    if (maxY < p.y)
      maxY = p.y;
    if (minY > p.y)
      minY = p.y;
  }

  x = (maxX+minX)/2;
  y = (maxY+minY)/2;
  r = (maxX-minX)/2;

//  p0 = Vector2i(points.back().x, points.back().y); points.pop_back();
//  p1 = Vector2i(points.back().x, points.back().y); points.pop_back();
//  p2 = Vector2i(points.back().x, points.back().y); points.pop_back();
//
//  const Vector3f m = RHT::fitACircle(p0, p1, p2);
//  x = m.x;
//  y = m.y;
//  r = m.z;

  return true; // [FIXME] : The rest is refining the fitted circles, which turned out to not be useful enough.

  std::vector<Vector3f> circles;
  const unsigned pointSize = points.size();
  for (unsigned i=0; i<pointSize; ++i)
    for (unsigned j=i+1; j<pointSize; ++j)
      for (unsigned k=j+1; k<pointSize; ++k)
        circles.push_back(RHT::fitACircle(
            Vector2i(points[i].x, points[i].y),
            Vector2i(points[j].x, points[j].y),
            Vector2i(points[k].x, points[k].y)));

  Vector3f mean;
  for (auto c : circles)
    mean += c;
  mean /= circles.size();

  x = mean.x;
  y = mean.y;
  r = mean.z;

  Vector3f var;
  for (auto c : circles)
  {
    const Vector3f distance = c - mean;
    var[0] += distance[0] * distance[0];
    var[1] += distance[1] * distance[1];
    var[2] += distance[2] * distance[2];
  }
  var /= (circles.size() - 1);

  Vector3f const invalidity = Vector3f(sqrt(var[0]), sqrt(var[1]), sqrt(var[2])) / mean.z;

  if(invalidity[0] < 2 || invalidity[1] < 2 || invalidity[2] < 2)
    return false;

  return true;
}

bool BallPerceptor::isNotGreenChecked(int x, int y)
{
  if (x > -1 && x < theImage.width && y > -1 && y < theImage.height)
    return !theColorReference.isGreen(theImage[y]+x);
  return false;
}

bool BallPerceptor::checkWhitePercentage(int cx, int cy, int r)
{
  int whitePixel=0, nonGreenPixels = 0, totalSearchedPixel=0;
  for (int sx=0; sx<r; ++sx)
  {
    const int& sY = sqrt(r*r - sx*sx); //-- calculating maximum sy in the mentioned sx
    for (int sy=0; sy<sY; ++sy)
    {
      totalSearchedPixel += searchedColor(cx+sx, cy+sy, whitePixel, nonGreenPixels);
      totalSearchedPixel += searchedColor(cx-sx, cy+sy, whitePixel, nonGreenPixels);

      totalSearchedPixel += searchedColor(cx+sx, cy-sy, whitePixel, nonGreenPixels);
      totalSearchedPixel += searchedColor(cx-sx, cy-sy, whitePixel, nonGreenPixels);
    }
  }

  if (totalSearchedPixel == 0)
    return false;

  //-- Reject balls with less than 85% white pixels
  if ((float)whitePixel/(float)totalSearchedPixel < minWhitePercentage ||
      (float)nonGreenPixels/(float)totalSearchedPixel < minNonGreenPercentage)
    return false;
  return true;
}

bool BallPerceptor::checkBlackPercentage(int cx, int cy, int r)
{
  int blackPixels = 0, totalSearchedPixel=0;
  for (int sx=0; sx<r; ++sx)
  {
    const int& sY = sqrt(r*r - sx*sx); //-- calculating maximum sy in the mentioned sx
    for (int sy=0; sy<sY; ++sy)
    {
      totalSearchedPixel += searchedColorForBlack(cx+sx, cy+sy, blackPixels);
      totalSearchedPixel += searchedColorForBlack(cx-sx, cy+sy, blackPixels);

      totalSearchedPixel += searchedColorForBlack(cx+sx, cy-sy, blackPixels);
      totalSearchedPixel += searchedColorForBlack(cx-sx, cy-sy, blackPixels);
    }
  }

  if (totalSearchedPixel == 0)
    return false;

  if ((float)blackPixels/(float)totalSearchedPixel < minBlackPercentage ||
	  (float)blackPixels/(float)totalSearchedPixel > maxBlackPercentage)
    return false;
  return true;
}

int BallPerceptor::searchedColor(int x, int y, int& color, int& nonGreen)
{
  if (x > -1 && x < theImage.width && y > -1 && y < theImage.height)
  {
    nonGreen += theColorReference.isGreen(theImage[y]+x)?0:1;
    color += theColorReference.isWhite(theImage[y]+x);
    return 1;
  }
  return 0;
}

int BallPerceptor::searchedColorForBlack(int x, int y, int& black)
{
  if (x > -1 && x < theImage.width && y > -1 && y < theImage.height)
  {
    black += isBlack(x, y);
    return 1;
  }
  return 0;
}

bool BallPerceptor::isBlack(int const x, int const y) const
{
	return theImage[y][x].y < blackThre && !theColorReference.isGreen(theImage[y]+x) && !theColorReference.isBlue(theImage[y]+x);
}

bool BallPerceptor::calculateBallOnField(BallPercept& ballPercept)
{
  const Vector2<> correctedCenter = theImageCoordinateSystem.toCorrected(ballPercept.positionInImage);
  Vector3<> cameraToBall(theCameraInfo.focalLength, theCameraInfo.opticalCenter.x - correctedCenter.x, theCameraInfo.opticalCenter.y - correctedCenter.y);
  cameraToBall.normalize(theFieldDimensions.ballRadius * theCameraInfo.focalLength / ballPercept.radiusInImage);
  Vector3<> rotatedCameraToBall = theCameraMatrix.rotation * cameraToBall;
  const Vector3<> sizeBasedCenterOnField = theCameraMatrix.translation + rotatedCameraToBall;
  const Vector3<> bearingBasedCenterOnField =  theCameraMatrix.translation - rotatedCameraToBall * ((theCameraMatrix.translation.z - theFieldDimensions.ballRadius) / rotatedCameraToBall.z);

  CIRCLE("module:BallPerceptor:field", sizeBasedCenterOnField.x, sizeBasedCenterOnField.y, theFieldDimensions.ballRadius, 1, Drawings::ps_solid, ColorRGBA(0, 0, 0xff), Drawings::bs_null, ColorRGBA());
  CIRCLE("module:BallPerceptor:field", bearingBasedCenterOnField.x, bearingBasedCenterOnField.y, theFieldDimensions.ballRadius, 1, Drawings::ps_solid, ColorRGBA(0xff, 0, 0), Drawings::bs_null, ColorRGBA());

  if (rotatedCameraToBall.z < 0)
  {
    ballPercept.relativePositionOnField.x = bearingBasedCenterOnField.x;
    ballPercept.relativePositionOnField.y = bearingBasedCenterOnField.y;
  }
  else
  {
    ballPercept.relativePositionOnField.x = sizeBasedCenterOnField.x;
    ballPercept.relativePositionOnField.y = sizeBasedCenterOnField.y;
  }
  return true;
}
