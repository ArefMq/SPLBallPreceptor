/**
 * @file RHT.h
 * A general purpose random hough transform module for detecting circles
 * @author <a href="mailto:a.moqadam@mrl-spl.ir">Aref Moqadam</a>
 * @date Apr 2016
 */

#pragma once

#include "Representations/Infrastructure/Image.h"
#include "Tools/Math/Vector.h"
#include <vector>
#include <cmath>

class RHT
{
public:
	RHT(const Image& image);
	~RHT();

	void update();
	const std::vector<Vector3f>& extractedPoints() const { return _extPoints; }

	// [FIXME] : move this function to cpp
	static inline Vector3f fitACircle(const Vector2i& p1, const Vector2i& p2, const Vector2i& p3)  //-- The result is in this format: (cx, cy), radius
	{
	  const Vector3f P1 = Vector3f(p1.x, p1.y, 0);
	  const Vector3f P2 = Vector3f(p2.x, p2.y, 0);
	  const Vector3f P3 = Vector3f(p3.x, p3.y, 0);

	  const Vector3f t = P2-P1; // u
	  const Vector3f u = P3-P1; // v
	  const Vector3f v = P3-P2;

	  //-- Cross Product of : t x u
	  const Vector3f w = Vector3f(
	      t.y*u.z - t.z*u.y,
	      t.z*u.x - t.x*u.z,
	      t.x*u.y - t.y*u.x
	  );

	  const float t2 = t.x*t.x + t.y*t.y + t.z*t.z;
	  const float u2 = u.x*u.x + u.y*u.y + u.z*u.z;
	  const float w2 = w.x*w.x + w.y*w.y + w.z*w.z;

	  const Vector3f mid = u * (t2 * (u.x*v.x + u.y*v.y + u.z*v.z)) -
	      t * (u2 * (t.x*v.x + t.y*v.y + t.z*v.z));

	  const float cx = P1.x + (mid / (2*w2)).x;
	  const float cy = P1.y + (mid / (2*w2)).y;
	  const float r = 0.5f * sqrt( t2 * u2 * (v.x*v.x + v.y*v.y + v.z*v.z) / w2 );

	  return Vector3f(cx, cy, r);
	}

private:
	int _rhtSamples;
	int _divisions;
	const Image& _edgeImage;
	std::vector<Vector3f> _extPoints;
	std::vector<Vector4f> _prospectiveCircles;
	std::vector<std::vector<Vector2i> > _edges;
	int _pointsEachSegment;
	float _selectingSigma;

	inline void incriment(int x, int y, int& weight);
	void extractEdgePoints();
	void selectRandomPoint();
	void houghTransform(const Vector2i& p1, const Vector2i& p2, const Vector2i& p3);
	void addCircle(const Vector3f& cirlce, int weight); //-- Accepting policy is here
	void extractResults();
};
