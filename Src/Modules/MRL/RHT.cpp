/**
 * @file RHT.h
 * A general purpose random hough transform module for detecting circles
 * @author <a href="mailto:a.moqadam@mrl-spl.ir">Aref Moqadam</a>
 * @date Apr 2016
 */

#include "RHT.h"
#include <time.h>
#include <algorithm>

#include <iostream>
#include "Tools/Debugging/DebugDrawings.h"

RHT::RHT(const Image& image) :
  _rhtSamples(5),
  _divisions(4),
  _edgeImage(image),
  _pointsEachSegment(5),
  _selectingSigma(15)
{
  // [TODO] : read this parameters from a config file

  srand(time(NULL));
}

RHT::~RHT()
{
}

void RHT::update()
{
  _prospectiveCircles.clear();
  extractEdgePoints();

  for (unsigned i=0; i<_rhtSamples; ++i)
    selectRandomPoint();

  extractResults();
}

void RHT::extractEdgePoints()
{
  _edges.clear();

  const int offsetX = _edgeImage.width/_divisions;
  const int offsetY = _edgeImage.height/_divisions;

  for (unsigned i=0; i<_divisions; ++i)
    for (unsigned j=0; j<_divisions; ++j)
    {
      const int si = i*offsetX;
      const int sj = j*offsetY;

      std::vector<Vector2i> subImage;

      for (int x=si; x<si+offsetX; ++x)
        for (int y=sj; y<sj+offsetY; ++y)
          if (_edgeImage[y][x].y > 127)
            subImage.push_back(Vector2i(x, y));

      _edges.push_back(subImage);
    }
}

void RHT::selectRandomPoint()
{
  //-- It is better to divide image into different segments,
  //-- in order to  increase the chance of selecting random
  //-- point from a circle. This trick had a really incredible
  //-- impact on the algorithm.
  for (std::vector<Vector2i> subImage : _edges)
  {
    if (!subImage.size())
      continue;

    for (unsigned itr=0; itr<_pointsEachSegment; ++itr)
    {
      const Vector2i& p1 = subImage[rand() % subImage.size()];
      const Vector2i& p2 = subImage[rand() % subImage.size()];
      const Vector2i& p3 = subImage[rand() % subImage.size()];

      houghTransform(p1, p2, p3);
    }
  }
}

void RHT::houghTransform(const Vector2i& p1, const Vector2i& p2, const Vector2i& p3)
{
  Vector3f circle = fitACircle(p1, p2, p3);

  //-- Some experimental ball radius range
  if (circle.z < 0 || circle.z > _edgeImage.width/4)
  {
    DOT("hello", p1.x*4, p1.y*4, ColorClasses::yellow, ColorClasses::yellow);
    DOT("hello", p2.x*4, p2.y*4, ColorClasses::yellow, ColorClasses::yellow);
    DOT("hello", p3.x*4, p3.y*4, ColorClasses::yellow, ColorClasses::yellow);
    return;
  }

  int weight = 0;
  for (float theta=-M_PI; theta<0; theta+=0.1)
  {
    const int x = cos(theta) * circle.z + circle.x;
    const int y = sin(theta) * circle.z + circle.y;
    incriment(x, y, weight);
  }

  addCircle(circle, weight);
}

inline void RHT::incriment(int x, int y, int& weight)
{
  if (x > 0 && x < _edgeImage.width &&
      y > 0 && y < _edgeImage.height &&
      _edgeImage[y][x].y > 127)
      weight++;
}

void RHT::addCircle(const Vector3f& cirlce, int weight)
{
  const float radius = cirlce.z;

  for (Vector4f& s : _prospectiveCircles)
  {
    if ((Vector2f(cirlce.x, cirlce.y) - Vector2f(s.v[0], s.v[1])).sqr() < _selectingSigma &&
        abs(radius - s[2]) < _selectingSigma)
    {
      const float sWeight = s.v[3];
      s = Vector4f(
            (s.v[0]*sWeight + cirlce.x*weight) / (sWeight+weight), //-- update x
            (s.v[1]*sWeight + cirlce.y*weight) / (sWeight+weight), //-- update y
            (s.v[2]*sWeight + cirlce.z*weight) / (sWeight+weight), //-- update r
            sWeight+weight                                         //-- update weight
          );
      return;
    }
  }

  _prospectiveCircles.push_back(Vector4f(
      cirlce.x,
      cirlce.y,
      cirlce.z,
      weight
      ));
}

//-- This function had to be defined in the global space.
bool circleSearchFunction(const Vector4f& a, const Vector4f b)
{
  return (a.v[3] > b.v[3]);
}

void RHT::extractResults()
{
  _extPoints.clear();

  // [TODO] :
  // Instead of  sorting, I think it worse it to check if it is better
  // to push circles into _prospectiveCircles in uprising order at the
  // first point.
  std::sort(_prospectiveCircles.begin(), _prospectiveCircles.end(), circleSearchFunction);

  for (const Vector4f& s : _prospectiveCircles)
  {
    _extPoints.push_back(Vector3f(s.v[0], s.v[1], s.v[2]));

    // [FIXME] : this policy is completely wrong, since it does not guaranty the optimum result
    if (_extPoints.size() > 10)
      return;
  }
}
