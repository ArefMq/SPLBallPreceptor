/**
 * @file BallPerceptor.h
 * This file declares a module that provides the white ball percept.
 * @author <a href="mailto:a.moqadam@mrl-spl.ir">Aref Moqadam</a>
 * @author Ali Sharpassand
 * @date Mar. 2016
 */

#pragma once
#define USE_RHT //-- Whether to use RHT or normal HT

#include <vector>

#include "Tools/Module/Module.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/ImageCoordinateSystem.h"
#include "Representations/Perception/BallPercept.h"
#include "Representations/Perception/ColorReference.h"
#include "Representations/Perception/FieldBoundary.h"
#include "Representations/Perception/BodyContour.h"
#include "Tools/Debugging/DebugImages.h"
#include "Representations/Infrastructure/JointData.h"

#include "MRL/HoughTrans.h"
#include "MRL/EdgeDetection.h"
#include "MRL/RHT.h"


class Image;

MODULE(BallPerceptor)
  REQUIRES(FieldDimensions)
  REQUIRES(Image)
  REQUIRES(CameraMatrix)
  REQUIRES(ImageCoordinateSystem)
  REQUIRES(CameraInfo)
  REQUIRES(ColorReference)
  REQUIRES(FieldBoundary)
  REQUIRES(BodyContour)
  REQUIRES(FilteredJointData)
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(BallPercept)
END_MODULE

class BallPerceptor: public BallPerceptorBase
{
private:
  DECLARE_DEBUG_IMAGE(edgeImage);

  void update(BallPercept& ballPercept);
public:
  BallPerceptor();

private:
  float minWhitePercentage, minNonGreenPercentage, minBlackPercentage, maxBlackPercentage, blackCheckRatio;
  int blackThre;
  EdgeDetection edgeDetection;

#ifndef USE_RHT
  HoughTrans houghTrans;
#else
  RHT houghTrans;
#endif

  //-- @return: whether the pixel is inside the image or not
  //-- @param: color: whether the pixel is white or not
  bool checkWhitePercentage(int cx, int cy, int r);
  int searchedColor(int x, int y, int& color, int& nonGreen);
  bool refineEdges(int& x, int& y, int& r);
  bool isNotGreenChecked(int x, int y);
  bool calculateBallOnField(BallPercept& ballPercept);
  bool checkBlackPercentage(int cx, int cy, int r);
  int searchedColorForBlack(int x, int y, int& black);
  bool isBlack(int const x, int const y) const;
};
