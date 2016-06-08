/**
 * @file FRHT.cpp
 * A general purpose fast random hough transform module for detecting circles
 * @author <a href="mailto:a.moqadam@mrl-spl.ir">Aref Moqadam</a>
 * @date Apr 2016
 */

#include "FRHT.h"
#include <ctime>
#include "Tools/Debugging/DebugDrawings.h"

// [TODO] : make these configurable parameters
#define FRHT_ITERATIONS 150

FRHT::FRHT(EdgeImage& image) :
  _image(image)
{
  srand(time(0));
}

FRHT::~FRHT()
{
}

void FRHT::update()
{
  _circles.clear();

  if (!_image.edgePoints().size())
    return;

  const int maxIterations = _image.isCameraUpper?FRHT_ITERATIONS:FRHT_ITERATIONS/5;
  for (unsigned i=0; i<maxIterations; ++i)
  {
    const int edgePointsLastIndex = _image.edgePoints().size();
    int randomID = rand() % edgePointsLastIndex;
    Vector2i point = _image.edgePoints().at(randomID);
    int step = EdgeImage::edgeingStep(point.y-_image.originY) / 2;
    RECTANGLE("module:BallPerceptor:selectedPoints", point.x-step, point.y-step, point.x+step, point.y+step, 1, Drawings::bs_solid, ColorClasses::blue);


    _image.refine(point);

    const int additionalPoints = _image.edgePoints().size() - edgePointsLastIndex;
    if (additionalPoints > 0)
    {
      randomID = (rand() % additionalPoints) + edgePointsLastIndex;
      point = _image.edgePoints().at(randomID);
      step = EdgeImage::edgeingStep(point.y-_image.originY) / 2;
      CIRCLE("module:BallPerceptor:selectedPoints", point.x, point.y, 3, 1, Drawings::bs_solid, ColorClasses::yellow, Drawings::bs_null, ColorClasses::yellow);
      _image.refine(point);
    }

    findCircle(point, step);

  }

//  for (const auto& p : _image.edgePoints())
//  const Vector2i& p = _image.edgePoints().at(id);
//    _image.refine(p);
}

void FRHT::findCircle(const Vector2i& centerPoint, int step)
{
  RECTANGLE("module:BallPerceptor:selectedPoints", centerPoint.x-step, centerPoint.y-step, centerPoint.x+step, centerPoint.y+step, 1, Drawings::bs_solid, ColorClasses::orange);

  class SearchCell {
  public:
    SearchCell(int Distance, const Vector2i& Point) : distance(Distance), point(Point) {}
    int      distance;
    Vector2i point;
    // int score;
  };

  std::vector<SearchCell> searchPoints;

  // [FIXME] : there is a bug here, sometimes one point is pushed in some place with no edge in.


  for (int y=centerPoint.y-step; y<centerPoint.y+step; ++y)
    for (int x=centerPoint.x-step; x<centerPoint.x+step; ++x)
    {
      if (x < 0 || y < 0 || x > _image.width || y > _image.height)
        continue;

      if (_image[y][x].y < 127)
        continue;

      // [TODO] : make this one lookup table in order to speed up
      const int distance = sqrt((x-centerPoint.x)*(x-centerPoint.x) + (y-centerPoint.y)*(y-centerPoint.y));

      for (const auto& sp : searchPoints)
        if (sp.distance == distance)
        {
          checkCircle(centerPoint, sp.point, Vector2i(x, y));
//          return; // [FIXME] : check to see if it better to stop after first distance pair or not.
        }

      searchPoints.push_back(SearchCell(distance, Vector2i(x, y)));
      y+=2;
    }
}

void FRHT::checkCircle(const Vector2i p1, const Vector2i p2, const Vector2i p3)
{
  Vector3f circle = fitACircle(p1, p2, p3);
  _circles.push_back(circle);
  CIRCLE("module:BallPerceptor:houghPoints", circle.x, circle.y, circle.z, 1, Drawings::bs_solid, ColorClasses::blue, Drawings::bs_null, ColorClasses::blue);
}

const std::vector<Vector3f>& FRHT::extractedCircles() const
{
  return _circles;
}
