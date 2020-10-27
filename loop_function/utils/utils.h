#ifndef UTILS_H
#define UTILS_H

#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/general.h>
#include <argos3/core/utility/math/range.h>

using namespace argos;

/** 
 * Compute the distance of a point from a segment with vertices v1 and v2.
**/
Real DistanceFromSegment(const CVector2 &point, const CVector2 &s1, const CVector2 &v2);

/**
 * Return the range on X values of a given segment
 **/
CRange<Real> RangeX(const CVector2 &v1, const CVector2 &v2);

/**
 * Return the range on Y values of a given segment
 **/
CRange<Real> RangeY(const CVector2 &v1, const CVector2 &v2);

/**
 * Return the coordinates of the point in the segment whose length is k times the distance of the segment
 */
CVector2 SubdivideSegment(const CVector2 &v1, const CVector2 &v2, Real k);

/**
 * Return the angle between two points
 */
Real AngleBetweenPoints(const CVector2 &v1, const CVector2 &v2);

/**
 * Polar String
 */
std::string ToPolarString(const CVector2 &v1);

Real Map(Real x, Real in_min, Real in_max, Real out_min, Real out_max);

Real Sign(const Real &r);

#endif