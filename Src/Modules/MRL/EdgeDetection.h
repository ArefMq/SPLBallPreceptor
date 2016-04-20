/**
 * @file EdgeDetection.h
 * A general purpose module for edge detection
 * @author <a href="mailto:a.moqadam@mrl-spl.ir">Aref Moqadam</a>
 * @date Apr 2016
 */

#pragma once

#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/ColorReference.h"

#include "BodyCountorLookup.h"

#include <vector>
#include <iostream>

class EdgeDetection : public Image
{
public:
  typedef unsigned char Pixel;
  typedef std::vector<std::vector<int> > Filter;

private:
  class SobelFilterVertical : public Filter
  {
  public:
    SobelFilterVertical() {
      std::vector<int> row1;
      row1.push_back(-1); row1.push_back(0); row1.push_back(1);
      push_back(row1);

      std::vector<int> row2;
      row2.push_back(-2); row2.push_back(0); row2.push_back(2);
      push_back(row2);

      std::vector<int> row3;
      row3.push_back(-1); row3.push_back(0); row3.push_back(1);
      push_back(row3);
    }
  };

  class SobelFilterHorizontal : public Filter
  {
  public:
    SobelFilterHorizontal() {
      std::vector<int> row1;
      row1.push_back(-1); row1.push_back(-2); row1.push_back(-1);
      push_back(row1);

      std::vector<int> row2;
      row2.push_back(0); row2.push_back(0); row2.push_back(0);
      push_back(row2);

      std::vector<int> row3;
      row3.push_back(+1); row3.push_back(+2); row3.push_back(+1);
      push_back(row3);
    }
  };


  public:
  EdgeDetection(const Image& image, const ColorReference& colorReference);
  ~EdgeDetection();

  void calculateEdge(const CameraInfo& info, const Angle& pan, const Angle& tilt);
  unsigned step() const { return _step; }

  private:
  inline static Image::Pixel grayChannel(int val) { if (val > 255 || val < 0) std::cerr << "Invalid Value in gray conversion :: " << __FILE__ << " :: " << __LINE__ << std::endl; Image::Pixel p; p.cr = p.cb = 127; p.y = val; return p; }

  const Image& _image;
  const Filter* _filter1;
  const Filter* _filter2;
  BodyCountorLookup _bodyContour;

  int _edgeThreshold;
  unsigned _step;

  // [FIXME] : this is a hack fix it later
  const ColorReference& _colorReference;
};

