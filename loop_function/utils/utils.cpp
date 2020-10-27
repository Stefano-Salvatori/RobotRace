#include "utils.h"

Real DistanceFromSegment(const CVector2 &point, const CVector2 &v1, const CVector2 &v2)
{
    // Return minimum distance between line segment vw and point p
    const float l2 = SquareDistance(v2, v1); // i.e. |w-v|^2 -  avoid a sqrt

    if (l2 == 0.0)
    {
        return Distance(point, v1); // v == w case
    }
    // Consider the line extending the segment, parameterized as v + t (w - v).
    // We find projection of point p onto the line.
    // It falls where t = [(p-v) . (w-v)] / |w-v|^2
    // We clamp t from [0,1] to handle points outside the segment vw.
    const Real t = Max(Real(0), Min(Real(1), (point - v1).DotProduct(v2 - v1) / l2));
    const CVector2 projection = v1 + t * (v2 - v1); // Projection falls on the segment
    return Distance(point, projection);
}

CRange<Real> RangeX(const CVector2 &v1, const CVector2 &v2)
{
    return CRange<Real>(Min(v1.GetX(), v2.GetX()), Max(v1.GetX(), v2.GetX()));
}

CRange<Real> RangeY(const CVector2 &v1, const CVector2 &v2)
{
    return CRange<Real>(Min(v1.GetY(), v2.GetY()), Max(v1.GetY(), v2.GetY()));
}

CVector2 SubdivideSegment(const CVector2 &v1, const CVector2 &v2, Real k)
{
    return CVector2(v1.GetX() + (v2.GetX() - v1.GetX()) * k, v1.GetY() + (v2.GetY() - v1.GetY()) * k);
}

Real AngleBetweenPoints(const CVector2 &v1, const CVector2 &v2)
{
    const Real deltaX = v2.GetX()-v1.GetX();
    const Real deltaY = v2.GetY()-v1.GetY();
    return atan2(deltaY, deltaX);
}

/**
 * Polar String
 */
std::string ToPolarString(const CVector2 &v1)
{
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2)  << "(" << v1.Length() << "," << v1.Angle().GetValue() * (180.0 / M_PI) << "°)";
    return ss.str();
}

Real Map(Real x, Real in_min, Real in_max, Real out_min, Real out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
} 

int Sign(CRadians r)
{
    if(r.GetValue() > 0) return 1;
    else if(r.GetValue() < 0) return -1;
    else return 0;
}
