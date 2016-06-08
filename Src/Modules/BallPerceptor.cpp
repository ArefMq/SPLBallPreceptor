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
#include "Tools/Math/Geometry.h"

#include <iostream>
#include <fstream> //-- For sake of taking snap shots
#include <ctime> //-- For sake of taking snap shots
#include <sstream> //-- For sake of taking snap shots

// [FIXME] : move these
#define minWhitePercentage (0.35)
#define minNonGreenPercentage (0.7)
#define minBlackPercentage (0.04)
#define maxBlackPercentage (0.7)


MAKE_MODULE(BallPerceptor, Perception)

BallPerceptor::BallPerceptor() :
  edgeImage(theImage),
  houghTransform(edgeImage)
{
}

void BallPerceptor::update(BallPercept& ballPercept)
{
  static bool takeASnapShotFlag = false;
  DEBUG_RESPONSE("module:BallPerceptor:takeSnapShot", takeASnapShotFlag = true; );

  DECLARE_DEBUG_DRAWING("module:BallPerceptor:edgePoints", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:BallPerceptor:houghPoints", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:BallPerceptor:selectedHoughs", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:BallPerceptor:selectedPoints", "drawingOnImage");

  DECLARE_DEBUG_DRAWING("module:BallPerceptor:searchLine", "drawingOnImage");

  ballPercept.ballWasSeen = false;
  ballPercept.status = BallPercept::notSeen;

  edgeImage.isCameraUpper = theCameraInfo.camera == CameraInfo::upper;
  edgeImage.originY = theImageCoordinateSystem.origin.y;
  edgeImage.update();
  houghTransform.update();

  for (const auto& p : edgeImage.edgePoints())
    DOT("module:BallPerceptor:edgePoints", p.x, p.y, ColorClasses::red, ColorClasses::red);

  INIT_DEBUG_IMAGE(edgeImage,edgeImage);
  SEND_DEBUG_IMAGE(edgeImage);

  //-- Checking the hough results:
  for (const auto& c : houghTransform.extractedCircles())
  {
    float x = c.x * edgeImage.avStep;
    float y = c.y * edgeImage.avStep;
    float r = c.z * edgeImage.avStep;

    //-- Size Filter
    if (r > 60 || r < 2.5)
      continue;

    //-- White Percentage
    if (!checkWhitePercentage(x, y, r))
      continue;

    if (!refineEdges(x, y, r))
      continue;

    //-- Again Checking size!
    if (r > 60 || r < 2.5)
      continue;

    // [NOTE] : the process of checking radius and white percentage although it's heavy but it is done twice,
    //          please note that the second one (this one down below) is done to check whether this object
    //          has enough white pixel in it. But the first one is done to ignore refining edges for waste
    //          object. So, however this is a heavy process, but it's reduces the time cost overly.
    if (!checkWhitePercentage(x, y, r))
      continue;

    //-- White / Black Percentage
    if (!checkBlackPercentage(x, y, r))
      continue;

    if (!checkBelowFieldBoundary(x, y, r))
     continue;

    //-- Actual Size check
    if (!checkProjectedRadius(x, y, r))
      continue;

    if (!checkOutOfBody(x, y, r))
      continue;

    CIRCLE("module:BallPerceptor:selectedHoughs", x,y,r,1, Drawings::bs_solid, ColorClasses::red, Drawings::bs_null, ColorClasses::red);

    //-- Exporting results
    if (theCameraMatrix.isValid)
    {
      ballPercept.positionInImage = Vector2<>(x, y);
      ballPercept.radiusInImage = r;
      Geometry::Circle c;
      if(calculateBallOnField(ballPercept))
      {
        ballPercept.status = BallPercept::seen;
        ballPercept.ballWasSeen = true;
        if (takeASnapShotFlag)
        {
          takeASnapShot(x,y,r);
          takeASnapShotFlag = false;
        }

        return;
      }
    }
  }
}

bool BallPerceptor::checkOutOfBody(int cx, int cy, int r)
{
  for(float i = 0;  i < 2 * M_PI; i += M_PI / 8)
  {
    const int x = r * cos(i) + cx;
    const int y = r * sin(i) + cy;

    int minY = y;
    theBodyContour.clipBottom(x, minY);
    if(y > minY)
      return false;
  }
  return true;
}

bool BallPerceptor::checkBelowFieldBoundary(int x, int y, int r)
{
  //-- Check if the ball is totally inside the field
  return (y-r > theFieldBoundary.getBoundaryY(x));
}

bool BallPerceptor::checkProjectedRadius(int cx, int cy, int r)
{
  //-- Check ball radius
#define BALL_WIDTH_2 50   //-- Half of the ball width // [FIXME] : this should be a config
#define BALL_WIDTH   100  //-- ball width             // [FIXME] : this should be a config

  Vector3<> projectedLeft,projectedRight;
  bool isProjected = Geometry::calculatePointOnField(Vector2<>(cx-r, cy), BALL_WIDTH_2, theCameraMatrix, theCameraInfo, projectedLeft) &&
      Geometry::calculatePointOnField(Vector2<>(cx+r, cy), BALL_WIDTH_2, theCameraMatrix, theCameraInfo, projectedRight);

  return (abs(abs(projectedLeft.y - projectedRight.y) - BALL_WIDTH) < 50);
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

#define SEARCH_STEP(limit, startRadius, countingFormula, condtion, exportFunction, debug) \
  { \
    const int _limit = limit; \
    int step = startRadius; \
    for (int p=0; p<_limit && step > 0; ) \
    { \
      const int c = countingFormula; \
      if (condtion) \
      { \
        step /= 2; \
        continue; \
      } \
      p += step; \
      exportFunction; \
      debug; \
    } \
  }

bool BallPerceptor::refineEdges(float& X, float& Y, float& R)
{
  Vector2i topLeft;
  Vector2i bottomRight;

  SEARCH_STEP(theImage.width - X, R, X+(p+step), theColorReference.isGreen(theImage[Y]+c),      bottomRight.x = c, LINE("module:BallPerceptor:searchLine", X, Y, c, Y, 1, Drawings::bs_solid, ColorClasses::green););
  SEARCH_STEP(                 X, R, X-(p+step), theColorReference.isGreen(theImage[Y]+c),      topLeft.x = c,     LINE("module:BallPerceptor:searchLine", X, Y, c, Y, 1, Drawings::bs_solid, ColorClasses::green););
  SEARCH_STEP(theImage.height- Y, R, Y+(p+step), theColorReference.isGreen(theImage[c]+(int)X), bottomRight.y = c, LINE("module:BallPerceptor:searchLine", X, Y, X, c, 1, Drawings::bs_solid, ColorClasses::green););
  SEARCH_STEP(                 Y, R, Y-(p+step), theColorReference.isGreen(theImage[c]+(int)X), topLeft.y = c,     LINE("module:BallPerceptor:searchLine", X, Y, X, c, 1, Drawings::bs_solid, ColorClasses::green););

  X = (topLeft.x + bottomRight.x) / 2;
  Y = (topLeft.y + bottomRight.y) / 2;
  R = ((bottomRight.x - topLeft.x) / 2 + (bottomRight.y - topLeft.y) / 2) / 2;

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

int BallPerceptor::searchedColorForBlack(int x, int y, int& black)
{
  if (x > -1 && x < theImage.width && y > -1 && y < theImage.height)
  {
    black += isBlack(theImage[y]+x, theColorReference);
    return 1;
  }
  return 0;
}

bool BallPerceptor::isBlack(const Image::Pixel* p, const ColorReference& r)
{
	return r.isOrange(p) && !r.isGreen(p) && !r.isBlue(p);
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

void BallPerceptor::takeASnapShot(int cx, int cy, int r)
{
  static int nameCFO = 0;
  std::stringstream fileNameImage;
  std::stringstream fileNameMeta;
  fileNameImage << "/home/mrlspl/logs/SnapShot_" << time(0) << "_" << nameCFO << ".image";
  fileNameMeta  << "/home/mrlspl/logs/SnapShot_" << time(0) << "_" << nameCFO << ".meta";

  std::ofstream outFileImage(fileNameImage.str(), std::ios::out | std::ios::binary | std::ios::trunc);
  std::ofstream outFileMeta(fileNameMeta.str(),   std::ios::out | std::ios::trunc);
  if (!outFileImage)
  {
    std::cerr << "Can not create output file image...\n";
    return;
  }
  if (!outFileMeta)
  {
    std::cerr << "Can not create output file meta-data...\n";
    return;
  }

#define BUFFER_WIDTH 30
#define BUFFER_SIZE (BUFFER_WIDTH*BUFFER_WIDTH*3)

  unsigned char dataBuffer[BUFFER_SIZE];
  for (unsigned j=0; j<BUFFER_WIDTH; ++j)
    for (unsigned i=0; i<BUFFER_WIDTH; ++i)
    {
      const int x = (2*r*i/BUFFER_WIDTH + cx - r);
      const int y = (2*r*j/BUFFER_WIDTH + cy - r);

      if (x > -1 && x < theImage.width && y > -1 && y < theImage.height)
      {
        dataBuffer[j*BUFFER_WIDTH*3+i*3+0] = theImage[y][x].y;
        dataBuffer[j*BUFFER_WIDTH*3+i*3+1] = theImage[y][x].cb;
        dataBuffer[j*BUFFER_WIDTH*3+i*3+2] = theImage[y][x].cr;
      }
      else
      {
        dataBuffer[j*BUFFER_WIDTH*3+i*3+0] = 0;
        dataBuffer[j*BUFFER_WIDTH*3+i*3+1] = 0;
        dataBuffer[j*BUFFER_WIDTH*3+i*3+2] = 0;
      }
    }

  outFileImage.write((const char*)dataBuffer, BUFFER_SIZE);
  outFileImage.close();

  outFileMeta << BUFFER_SIZE << " # Buffer Size\n"
              << BUFFER_WIDTH << " # Buffer Width\n"
              << BUFFER_WIDTH << " # Buffer Height\n"
              << cx << " # Ball X\n"
              << cy << " # Ball Y\n"
              << r << " # Ball Radius\n"
              << theImageCoordinateSystem.origin.y << " # Horizon Y \n"
              << theImage.width << " # Frame Width\n"
              << theImage.height << " # Frame Height\n";
  outFileMeta.close();

  if (nameCFO++ > 9876)
    nameCFO = 0;
}

