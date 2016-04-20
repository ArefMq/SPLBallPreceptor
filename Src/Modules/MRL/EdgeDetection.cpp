/**
 * @file EdgeDetection.h
 * A general purpose module for edge detection
 * @author <a href="mailto:a.moqadam@mrl-spl.ir">Aref Moqadam</a>
 * @date Apr 2016
 */

#include "EdgeDetection.h"
#include "Tools/Debugging/DebugDrawings.h"

EdgeDetection::EdgeDetection(const Image& image, const ColorReference& colorReference) :
  _image(image),
  _filter1(new SobelFilterVertical()), //-- The filter has to be square
  _filter2(new SobelFilterHorizontal()), //-- The filter has to be square
  _edgeThreshold(450),
  _step(4),
  _colorReference(colorReference)
{
  // [FIXME] : read the above parameters from a config file
}

EdgeDetection::~EdgeDetection()
{
}

void EdgeDetection::calculateEdge(const CameraInfo& info, const Angle& pan, const Angle& tilt)
{
  if (width != _image.width/_step || height != _image.height/_step)
    setResolution(_image.width/_step, _image.height/_step);

  const unsigned startY = 0;
  const unsigned startX = 0;
  const unsigned size   = _filter1->size();
  const unsigned endY   = height - size;
  const unsigned endX   = width  - size;

  for (unsigned x=startX; x<endX; ++x)
  {
    bool isAboveFieldBoundary = info.camera == CameraInfo::upper;

    for (unsigned y=startY; y<endY; ++y)
    {
      if (_bodyContour.check(x*4, y*4, pan, tilt, _image.width, _image.height))
      {
        DOT("hello", x*4, y*4, ColorClasses::red, ColorClasses::red);
        (*this)[y][x] = grayChannel(0);
        continue;
      }

      if (isAboveFieldBoundary &&
         (isAboveFieldBoundary = !(_colorReference.isGreen(_image[y*_step]+(x*_step)) &&
                                   _colorReference.isGreen(_image[y*_step+1]+(x*_step)) )))
      {
        (*this)[y][x] = grayChannel(0);
        continue;
      }


      int r1=0, r2=0, r3=0;
      for (unsigned i=0; i<size; ++i)
        for (unsigned j=0; j<size; ++j)
        {
          //-- Please note that the image parameter order is reversed.
          r1 += ((int)(_image[(y+j)*_step][(x+i)*_step]).y ) * (*_filter1)[i][j]; //-- >>> See below, comment 1
          r2 += ((int)(_image[(y+j)*_step][(x+i)*_step]).cb) * (*_filter1)[i][j]; //-- >>> See below, comment 1
          r3 += ((int)(_image[(y+j)*_step][(x+i)*_step]).cr) * (*_filter1)[i][j]; //-- >>> See below, comment 1
        }
      for (unsigned i=0; i<size; ++i)
        for (unsigned j=0; j<size; ++j)
        {
          //-- Please note that the image parameter order is reversed.
          r1 += ((int)(_image[(y+j)*_step][(x+i)*_step]).y ) * (*_filter2)[i][j]; //-- >>> See below, comment 1
          r2 += ((int)(_image[(y+j)*_step][(x+i)*_step]).cb) * (*_filter2)[i][j]; //-- >>> See below, comment 1
          r3 += ((int)(_image[(y+j)*_step][(x+i)*_step]).cr) * (*_filter2)[i][j]; //-- >>> See below, comment 1
        }

      //-- Getting absolute value for r, since it is the difference and could be negative
      //-- Then subtract by Sobel gain in order to normalize the values.
      //-- The '4' value could be a configurable value, but since it almost a fixed value,
      //-- I put in it this way;
      r1 = (r1<0?-r1:r1)/8; // 8 = 4+4 => because of two filter used
      r2 = (r2<0?-r2:r2)/8;
      r3 = (r3<0?-r3:r3)/8;
      const int r = (r1*r1+r2*r2+r3*r3);
      (*this)[y][x] = grayChannel(r>_edgeThreshold?255:0);
    }
  }
}

//-- COMMENT 1:
//   This syntax could be written as: _image[y*_step+j][(x*_step+i]
//	 but in this case the edges would be calculated by their actual
//	 neighbors,  but in the way I put above, they are calculated by
//	 the having a sight of step;
