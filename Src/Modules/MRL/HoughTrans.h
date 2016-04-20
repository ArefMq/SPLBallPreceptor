/**
 * @file HoughTrans.h
 * A general purpose hough transform module
 * @author <a href="mailto:a.moqadam@mrl-spl.ir">Aref Moqadam</a>
 * @date Apr 2016
 */


#pragma once

#include "Representations/Infrastructure/Image.h"
#include "Tools/Math/Vector.h"
#include <vector>

class HoughTrans
{
  class HoughSpace
  {
  public:
    typedef unsigned long HoughPixel;

    HoughSpace(unsigned width, unsigned height, unsigned depth);
    ~HoughSpace();
    void clean();
    void resize(unsigned width, unsigned height, unsigned depth);

    inline unsigned size() const { return _size; }
    inline unsigned width() const { return _width; }
    inline unsigned height() const { return _height; }
    inline unsigned depth() const { return _depth; }

    inline const HoughPixel& operator () (unsigned i, unsigned j, unsigned k) const;
    inline HoughPixel& operator () (unsigned i, unsigned j, unsigned k);

  private:
    unsigned _width, _height, _depth;
    unsigned _size;
    HoughPixel* _space;
  };

public:
  HoughTrans(const Image& image);
  ~HoughTrans();

  void update();
  const std::vector<Vector4i>& extractedPoints() const { return _extPoints; }

private:
  const Image& _image;
  unsigned _houghDepth;
  unsigned _depthOffset;
  unsigned _depthRatio;
  HoughSpace _houghSpace;
  std::vector<Vector4i> _extPoints;

  void calculateHough();
  void extractPoints();
  inline void increase(unsigned x, unsigned y, unsigned z);
};

