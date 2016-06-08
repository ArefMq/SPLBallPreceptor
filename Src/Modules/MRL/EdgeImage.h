/**
 * @file EdgeImage.h
 * A general purpose module for edge detection
 * @author <a href="mailto:a.moqadam@mrl-spl.ir">Aref Moqadam</a>
 * @date Apr 2016
 */

#pragma once

#include <vector>
#include "Tools/Math/Vector.h"
#include "Representations/Infrastructure/Image.h"

// [FIXME] : make these parameters
#define expStep  0.0625
#define expCStep 1

class EdgeImage : public Image
{
public:
  EdgeImage(const Image& image);
  ~EdgeImage();

  void update();
  const std::vector<Vector2i>& edgePoints() const { return _edgePoints; }
  void refine(const Vector2i& point);
  static inline int edgeingStep(int y) { return y*expStep+expCStep; }


  bool isCameraUpper; // [FIXME] : move this somewhere else
  int originY; // [FIXME] : move this somewhere else
  int avStep; // [FIXME] : this not quite good... :S

private:
  const Image& _image;
  std::vector<std::vector<Vector2i> > _scanGraphLookup;
  std::vector<Vector2i> _edgePoints; // [TODO] : make this raw array

  static Pixel black;
  static Pixel red;

  const Pixel& calculateEdge(const Vector2i& topLeft, const Vector2i& middle, const Vector2i& bottomRight);
  void createLookup();
};
