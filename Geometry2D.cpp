#include "Geometry2D.h"
#include "matrices.h"
#include "Compare.h"
#include <cmath>
#include <cfloat>



//Methods
//line2d
float Len(const Line2D& line)
{
	return Mag(line.end - line.start);
}
float LenSq(const Line2D& line)
{
	return MagSq(line.end - line.start);
}
//rectangle2D
vec2 GetMin(const Rectangle2D& rect)
{
	vec2 p1 = rect.pos;
	vec2 p2 = rect.pos + rect.size;
	return vec2(fminf(p1.x, p2.x), fminf(p1.y, p2.y));
}
vec2 GetMax(const Rectangle2D& rect)
{
	vec2 p1 = rect.pos;
	vec2 p2 = rect.pos + rect.size;
	return vec2(fmaxf(p1.x, p2.x), fmaxf(p1.y, p2.y));
}
Rectangle2D FromMinMax(const vec2& min, const vec2& max)
{
	return Rectangle2D(min,max-min);
}
//containment
bool PointOnLine(const Point2D& p, const Line2D& l)
{
	float dy = (l.end.y - l.start.y);
	float dx = (l.end.x - l.start.x);
	float M = dy / dx;
	float B = l.start.y - M * l.start.x;
	return CMP(p.y, M * p.x + B);
}
bool PointInCircle(const Point2D& p, const Circle& c)
{
	Line2D l(p, c.pos);
	if (LenSq(l) < c.r * c.r)return true;
	return false;
}
bool PointInRectangle(const Point2D& p, const Rectangle2D& r)
{
	vec2 min = GetMin(r);
	vec2 max = GetMax(r);

	return min.x <= p.x && min.y <= p.y && p.x <= max.x && p.y <= max.y;
}
bool PointInOrientedRectangle(const Point2D& p, const OrientedRectangle& r)
{
	vec2 rotVec = p - r.pos;
	float theta = -DEG2RAD(r.rot);
	float zRot2x2[] = { cosf(theta),sinf(theta),-sinf(theta),cosf(theta) };
	Mul(rotVec.asArray, vec2(rotVec.x, rotVec.y).asArray, 1, 2, zRot2x2, 2, 2);
	Rectangle2D localR(Point2D(), r.halfExtents * 2.0f);
	vec2 localP = rotVec + r.halfExtents;
	return PointInRectangle(localP, localR);
}
//intersection
bool LineCircle(const Line2D& l, const Circle& c)
{
	vec2 ab = l.end - l.start;
	float t = Dot(c.pos - l.start, ab) / Dot(ab, ab);
	if (t < 0.0f || t>1.0f)return false;
	Point2D closestPoint = l.start + ab * t;
	Line2D circleToClosest(c.pos, closestPoint);
	return LenSq(circleToClosest) < c.r * c.r;
}
bool LineRectangle(const Line2D& l, const Rectangle2D& r)
{
	if (PointInRectangle(l.start, r) || PointInRectangle(l.end, r))return true;
	vec2 norm = Nomd(l.end - l.start);
	norm.x = (norm.x != 0) ? 1.0f / norm.x : 0;
	norm.y = (norm.y != 0) ? 1.0f / norm.y : 0;
	vec2 min = (GetMin(r) - l.start) * norm;
	vec2 max = (GetMax(r) - l.start) * norm;
	float tmin = fmaxf(fminf(min.x, max.x), fminf(min.y, max.y));
	float tmax = fminf(fmaxf(min.x, max.x), fmaxf(min.y, max.y));
	if (tmax < 0 || tmin>tmax)return false;
	float t = (tmin < 0.0f) ? tmax : tmin;
	return t > 0.0f && t * t < LenSq(l);
}
bool LineOrientedRectangle(const Line2D& l, const OrientedRectangle& r)
{
	float theta = -DEG2RAD(r.rot);
	float zRot2x2[] = { cosf(theta),sinf(theta),-sinf(theta),cosf(theta) };
	Line2D localL;
	vec2 rotVec = l.start - r.pos;
	Mul(rotVec.asArray, vec2(rotVec.x, rotVec.y).asArray, 1, 2, zRot2x2, 2, 2);
	localL.start = rotVec + r.halfExtents;
	rotVec = l.end - r.pos;
	Mul(rotVec.asArray, vec2(rotVec.x, rotVec.y).asArray, 1, 2, zRot2x2, 2, 2);
	localL.end = rotVec + r.halfExtents;
	Rectangle2D localR(Point2D(), r.halfExtents * 2.0f);
	return LineRectangle(localL, localR);

}
//
