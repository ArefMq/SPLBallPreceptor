/**
 * @file BallPerceptor.h
 * This file declares a module that provides the white ball percept.
 * @author <a href="mailto:a.moqadam@mrl-spl.ir">Aref Moqadam</a>
 * @author Ali Sharpassand
 * @date Mar. 2016
 */

#pragma once

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

#include "MRL/EdgeImage.h"
#include "MRL/FRHT.h"

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
public:
  BallPerceptor();

private:
  void update(BallPercept& ballPercept);
  bool checkWhitePercentage(int cx, int cy, int r);
  int searchedColor(int x, int y, int& color, int& nonGreen);
  bool refineEdges(float& x, float& y, float& r);
  bool isNotGreenChecked(int x, int y);
  bool checkBlackPercentage(int cx, int cy, int r);
  int searchedColorForBlack(int x, int y, int& black);
  bool isBlack(const Image::Pixel* p, const ColorReference& r);
  bool checkBelowFieldBoundary(int x, int y, int r);
  bool checkProjectedRadius(int x, int y, int r);
  bool calculateBallOnField(BallPercept& ballPercept);
  bool checkOutOfBody(int x, int y, int r);
  void takeASnapShot(int x, int y, int r);

  EdgeImage edgeImage; DECLARE_DEBUG_IMAGE(edgeImage);
  FRHT houghTransform;
};
