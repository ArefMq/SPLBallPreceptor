/**
 * @file EdgeImage.cpp
 * A general purpose module for edge detection
 * @author <a href="mailto:a.moqadam@mrl-spl.ir">Aref Moqadam</a>
 * @date Apr 2016
 */

#include "EdgeImage.h"
#include <iostream>
#include <cmath>
#include "Tools/Debugging/DebugDrawings.h"

#define pl //std::cout << __FILE__ << " :: " << __LINE__ << "\n";

// [FIXME] : make this a class scope function
//-- Filtered Pixel!
#define __fixel(__x, __y, __action) \
 if (__x < this->width && __x > -1 && __y < this->height && __y > -1) { Image::Pixel& pxl = (*this)[__y][__x]; const Image::Pixel& pxlOrg = _image[__y][__x];  __action; }
#define fixel(__x, __y, __action) __fixel(__x, __y, __action)
//#define fixel(__x, __y, __action) __fixel(__x/avStep, (originY+__y)/avStep, __action)
#define SC_X(row, col) (_scanGraphLookup.at(row).at(col).x/avStep)
#define SC_Y(row, col) ((_scanGraphLookup.at(row).at(col).y+originY)/avStep)

Image::Pixel EdgeImage::black;
Image::Pixel EdgeImage::red;


EdgeImage::EdgeImage(const Image& image) :
  isCameraUpper(false),
  originY(0),
  _image(image),
  avStep(1)
{
  black.cr = black.cb = 127; black.y = 0;
  red.cr = 250; red.cb = 0; red.y = 127;
}

EdgeImage::~EdgeImage()
{
}

void EdgeImage::createLookup()
{
  std::cout << "creating edge lookup table...\n";
  setResolution(_image.width/avStep, _image.height/avStep);

  for (int y=0; y< height*2; y+=edgeingStep(y))
  {
    std::vector<Vector2i> scanRow;
    for (int x=0; x< width; x+=edgeingStep(y))
      scanRow.push_back(Vector2i(x, y+10));
    _scanGraphLookup.push_back(scanRow);
  }

  static int i=0;
  if (i++ > 10)
    std::cerr << "[It looks this message is keep popping out!]\n[it might be because the difference of the upper and lower camera resolution.]\n\n";
}

void EdgeImage::refine(const Vector2i& point)
{
  // [FIXME] : I'm not sure about the step, revise the way of step calculation
  const int step = edgeingStep(point.y-originY);// + edgeingStep(point.y));

  const int startX = (point.x-step)>1 ? (point.x-step) : 1;
  const int startY = (point.y-step)>1 ? (point.y-step) : 1;

  const int endX = (point.x+step)<(width-1)  ? (point.x+step) : (width-1);
  const int endY = (point.y+step)<(height-1) ? (point.y+step) : (height-1);


  for (int y=startY; y<endY; ++y)
    for (int x=startX; x<endX; ++x)
    {
      if (x == point.x && y == point.y)
        continue;

      Vector2i middle = Vector2i(x,y);

      bool validPixel = false;
      fixel(middle.x,middle.y, validPixel = pxl.y == 0 );
      if (!validPixel)
        continue;

      Vector2i topLeft = Vector2i(x-1, y-1);
      Vector2i bottomRight = Vector2i(x+1, y+1);

      Pixel edgePixel = black;
      fixel(middle.x, middle.y, edgePixel = calculateEdge(topLeft, middle, bottomRight); pxl = (edgePixel.y>127)?red:black; );
    }
}

void EdgeImage::update()
{
  // [TODO] : Implement field boundary
  // [FIXME] : do something about image boundaries that become edges
pl
  _edgePoints.clear();

  if (width != _image.width/avStep || height != _image.height/avStep)
    createLookup();

  for (int y=0; y<height; ++y)
    for (int x=0; x<width; ++x)
      (*this)[y][x] = black; //_image[y*avStep][x*avStep];

  for (unsigned row=0; row<_scanGraphLookup.size(); ++row)
    for (unsigned col=0; col<_scanGraphLookup.at(row).size(); ++col)
    {
      Vector2i middle = Vector2i(
          SC_X(row, col),
          SC_Y(row, col)
          );

      Vector2i topLeft = Vector2i(
          (col>0)?SC_X(row,col-1):SC_X(row,col)-1,
          (row>0)?SC_Y(row-1,0):SC_Y(row,col)-1
          );

      Vector2i bottomRight = Vector2i(
          (col<_scanGraphLookup.at(row).size()-1)?SC_X(row,col+1):SC_X(row,col)+1,
          (row<_scanGraphLookup.size()-1)?SC_Y(row+1,0):SC_Y(row,col)+1
          );

      Pixel edgePixel = black;
      fixel(middle.x, middle.y, edgePixel = calculateEdge(topLeft, middle, bottomRight); pxl = edgePixel );
    }

pl
}

const Image::Pixel& EdgeImage::calculateEdge(const Vector2i& topLeft, const Vector2i& middle, const Vector2i& bottomRight)
{
  //-- Implementation of Sobel Filter
  //   This is Vertical Sobel Filter Parameters:
  //   [ -1  -2  -1 ]
  //   [  0   0   0 ]
  //   [ +1  +2  +1 ]
  //   And it is the same for horizontal except with a counter clockwise flip

  Pixel a0; fixel(topLeft.x,     topLeft.y, a0 = pxlOrg );
  Pixel a1; fixel(middle.x,      topLeft.y, a1 = pxlOrg );
  Pixel a2; fixel(bottomRight.x, topLeft.y, a2 = pxlOrg );

  Pixel a3; fixel(topLeft.x,     middle.y, a3 = pxlOrg );
  Pixel a4; fixel(middle.x,      middle.y, a4 = pxlOrg );
  Pixel a5; fixel(bottomRight.x, middle.y, a5 = pxlOrg );

  Pixel a6; fixel(topLeft.x,     bottomRight.y, a6 = pxlOrg );
  Pixel a7; fixel(middle.x,      bottomRight.y, a7 = pxlOrg );
  Pixel a8; fixel(bottomRight.x, bottomRight.y, a8 = pxlOrg );


  const int sobelVerticalY  = ((-a0.y  - 2*a1.y  - a2.y)  /* + 0*a3.y  + 0*a4.y  + 0*a5.y  */ + (a6.y  + 2*a7.y  + a8.y))  / 4;
  const int sobelVerticalCb = ((-a0.cb - 2*a1.cb - a2.cb) /* + 0*a3.cb + 0*a4.cb + 0*a5.cb */ + (a6.cb + 2*a7.cb + a8.cb)) / 4;
  const int sobelVerticalCr = ((-a0.cr - 2*a1.cr - a2.cr) /* + 0*a3.cr + 0*a4.cr + 0*a5.cr */ + (a6.cr + 2*a7.cr + a8.cr)) / 4;

  const int sobelHorizontalY  = ((-a0.y  - 2*a3.y  - a6.y)  /* + 0*a1.y  + 0*a4.y  + 0*a7.y  */ + (a2.y  + 2*a5.y  + a8.y))  / 4;
  const int sobelHorizontalCb = ((-a0.cb - 2*a3.cb - a6.cb) /* + 0*a1.cb + 0*a4.cb + 0*a7.cb */ + (a2.cb + 2*a5.cb + a8.cb)) / 4;
  const int sobelHorizontalCr = ((-a0.cr - 2*a3.cr - a6.cr) /* + 0*a1.cr + 0*a4.cr + 0*a7.cr */ + (a2.cr + 2*a5.cr + a8.cr)) / 4;

  Pixel result;
  result.cb = result.cr = 127; result.y = 1;
  const int ans = sqrt(
      sobelVerticalY*sobelVerticalY + sobelVerticalCb*sobelVerticalCb + sobelVerticalCr*sobelVerticalCr +
      sobelHorizontalY*sobelHorizontalY + sobelHorizontalCb*sobelHorizontalCb + sobelHorizontalCr*sobelHorizontalCr);

  //-- Thresholding:
  if (ans > 60) // [TODO] : make this threshold a configurable parameter.
  {
    result.y = 255;
    _edgePoints.push_back(middle);
  }
  //-- else: result.y remain 1 which means that this pixel has been processed

  return result;
}

#undef fixel
