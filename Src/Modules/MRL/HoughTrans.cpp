/**
 * @file HoughTrans.cpp
 * A general purpose hough transform module
 * @author <a href="mailto:a.moqadam@mrl-spl.ir">Aref Moqadam</a>
 * @date Apr 2016
 */

#include "HoughTrans.h"
#include <iostream>
#include <cmath>

#include "Tools/RingBuffer.h"
#include "Tools/Debugging/DebugDrawings.h"

HoughTrans::HoughTrans(const Image& image) :
  _image(image),
  _houghDepth(30), //-- Number of depth layer
  _depthOffset(8), //-- Depth starting point
  _depthRatio(2),  //-- Distance between each layer
  _houghSpace(HoughSpace(0, 0, 0))
{
}

HoughTrans::~HoughTrans()
{
}

void HoughTrans::update()
{

  if (_houghSpace.width() < _image.width || _houghSpace.height() < _image.height)
    _houghSpace.resize(_image.width, _image.height, _houghDepth);

  _houghSpace.clean();
  calculateHough();
  extractPoints();
}

void HoughTrans::extractPoints()
{
  _extPoints.clear();

  double max = 1.85;
  unsigned px=0, py=0, pr=0;
  for (unsigned cx=0; cx<_houghSpace.width(); ++cx)
    for (unsigned cy=0; cy<_houghSpace.height(); ++cy)
      for (unsigned R=0; R<_houghDepth; ++R)
      {
        const double r = R*_depthRatio + _depthOffset;
        const double s = (double)_houghSpace(cx, cy, R)/r;

        if (s > max)
        {
          // max = s;
          px = cx; py = cy; pr = r;
          _extPoints.push_back(Vector4i(px, py, pr, s));
        }
      }

}

void HoughTrans::calculateHough()
{
  //-- Calculate Hough Space;
  for (unsigned cx=0; cx<_image.width; ++cx)
    for (unsigned cy=0; cy<_image.height; ++cy)
      if (_image[cy][cx].y > 127)
        for (unsigned R=0; R<_houghDepth; ++R)
        {
          const int r = R*_depthRatio + _depthOffset;
          //-- Drawing a circle with radius of `r'
          for (int x=0; x<r; ++x)
          {
            const int y = sqrt(r*r - x*x);
            //-- Each line below, draw one quarter
            increase(cx + x, cy + y, R);
            increase(cx - x, cy + y, R);

            //-- The bottom part of the ball is not important,
            //-- since  it does not  have  a clear edge due to
            //-- ground reflex on it:
            //-- increase(cx + x, cy - y, R);
            //-- increase(cx - x, cy - y, R);
          }
        }
}

inline void HoughTrans::increase(unsigned x, unsigned y, unsigned z)
{
  if (x >= 0 && x < _houghSpace.width() && y >= 0 && y < _houghSpace.height() && z >= 0 && z < _houghSpace.depth())
    _houghSpace(x, y, z)++;
}

HoughTrans::HoughSpace::HoughSpace(unsigned width, unsigned height, unsigned depth) :
		    _width(width),
		    _height(height),
		    _depth(depth)
{
  resize(width, height, depth);
  clean();
}

inline const HoughTrans::HoughSpace::HoughPixel& HoughTrans::HoughSpace::operator () (unsigned i, unsigned j, unsigned k) const
{
  if (!(i < _width && j < _height && k < _depth))
    throw ("out of range matrix usage...\n");
  return _space[j*_width*_depth + i*_depth + k];
}

inline HoughTrans::HoughSpace::HoughPixel& HoughTrans::HoughSpace::operator () (unsigned i, unsigned j, unsigned k)
{
  if (!(i < _width && j < _height && k < _depth))
    throw ("out of range matrix usage...\n");
  return _space[j*_width*_depth + i*_depth + k];
}

void HoughTrans::HoughSpace::resize(unsigned width, unsigned height, unsigned depth)
{
  if (_space)
    delete _space;

  _width = width;
  _height = height;
  _depth = depth;

  _size = _width*_height*_depth;
  _space = new HoughPixel[_size];
}

void HoughTrans::HoughSpace::clean()
{
  for (unsigned i=0; i<_size; ++i)
    _space[i] = 0;
}

HoughTrans::HoughSpace::~HoughSpace()
{
  delete _space;
}
