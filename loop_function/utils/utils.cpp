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
