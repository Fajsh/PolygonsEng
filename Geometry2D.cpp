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

//collisions
bool CircleCircle(const Circle& c1, const Circle& c2)
{
	Line2D line(c1.pos, c2.pos);
	float radsq = (c1.r + c2.r) * (c1.r + c2.r);
	return LenSq(line) <= radsq;
}
bool CircleRectangle(const Circle& c, const Rectangle2D& r)
{
	vec2 min = GetMin(r);
	vec2 max = GetMax(r);
	Point2D closest = c.pos;
	if (closest.x < min.x)closest.x = min.x;
	else if (closest.x > max.x)closest.x = max.x;

	if (closest.y < min.y)closest.y = min.y;
	else if (closest.y > max.y)closest.y = max.y;//

	Line2D line(c.pos, closest);
	return LenSq(line) <= c.r * c.r;
}
bool CircleOrientedRectangle(const Circle& c, const OrientedRectangle& r)
{
	vec2 line = c.pos - r.pos;
	float theta = -DEG2RAD(r.rot);
	float zRot2x2[] = { cosf(theta),sinf(theta),-sinf(theta),cosf(theta) };
	Mul(line.asArray, vec2(line.x, line.y).asArray, 1, 2, zRot2x2, 2, 2);
	Circle lCircle(line + r.halfExtents, c.r);
	Rectangle2D lRect(Point2D(), r.halfExtents * 2.0f);
	return CircleRectangle(lCircle, lRect);
}
bool RectangleRectangle(const Rectangle2D& r1, const Rectangle2D& r2)
{
	vec2 aMin = GetMin(r1);
	vec2 bMin = GetMin(r2);
	vec2 aMax = GetMax(r1);
	vec2 bMax = GetMax(r2);

	bool overX = (bMin.x <= aMax.x) && (aMin.x <= bMax.x);
	bool overY = (bMin.y <= aMax.y) && (aMin.y <= bMax.y);
	return overX && overY;
}
//collisions - Separating Axis Theorem
Interval2D getInterval(const Rectangle2D& rect, const vec2& axis)
{
	Interval2D res;
	vec2 min = GetMin(rect);
	vec2 max = GetMax(rect);
	vec2 verts[] = { vec2(min.x,min.y), vec2(min.x,max.y),vec2(max.x,max.y),vec2(max.y,min.y) };
	res.min = res.max = Dot(axis, verts[0]);
	for (int i = 0; i < 4; ++i)
	{
		float projection = Dot(axis, verts[i]);
		if (projection < res.min)res.min = projection;
		if (projection > res.max)res.max = projection;
	}
	return res;
}
bool OverlapOnAxis(const Rectangle2D& r1, const Rectangle2D& r2, const vec2& axis)
{
	Interval2D a = getInterval(r1, axis);
	Interval2D b = getInterval(r2, axis);
	return ((b.min <= a.max) && (a.min <= b.max));
}
bool RectangleRectangleSAT(const Rectangle2D& r1, const Rectangle2D& r2)
{
	vec2 axisToTest[] = { vec2(1,0),vec2(0,1) };
	for (int i = 0; i < 2; ++i)
	{
		if (!OverlapOnAxis(r1, r2, axisToTest[i]))return false;
	}
	return true;
}
Interval2D getInterval(const OrientedRectangle& rect, const vec2& axis)
{
	Rectangle2D r = Rectangle2D(Point2D(rect.pos - rect.halfExtents), rect.halfExtents * 2.0f);
	vec2 min = GetMin(r);
	vec2 max = GetMax(r);
	vec2 verts[] = { min,max,vec2(min.x,max.y),vec2(max.x,min.y) };
	float theta = DEG2RAD(rect.rot);
	float zRot[] = { cosf(theta),sinf(theta), -sinf(theta),cosf(theta) };
	for (int i = 0; i < 4; ++i)
	{
		vec2 r = verts[i] - rect.pos;
		Mul(r.asArray, vec2(r.x, r.y).asArray, 1, 2, zRot, 2, 2);
		verts[i] = r + rect.pos;
	}
	Interval2D res;
	res.min = res.max = Dot(axis, verts[0]);
	for (int i = 1; i < 4; ++i)
	{
		float proj = Dot(axis, verts[i]);
		res.min = (proj < res.min) ? proj : res.min;
		res.max = (proj > res.max) ? proj : res.max;
	}
	return res;
}
bool OverlapOnAxis(const Rectangle2D& r1, const OrientedRectangle& r2, const vec2& axis)
{
	Interval2D a = getInterval(r1, axis);
	Interval2D b = getInterval(r2, axis);
	return ((b.min <= a.max) && (a.min <= b.max));
}
bool RectangleOrientedRectangle(const Rectangle2D& r1, const OrientedRectangle r2)
{
	vec2 axisToTest[]{ vec2(1,0),vec2(0,1),vec2(),vec2() };
	float theta = DEG2RAD(r2.rot);
	float zRot[] = { cosf(theta),sinf(theta), -sinf(theta),cosf(theta) };
	vec2 axis = Nomd(vec2(r2.halfExtents.x, 0));
	Mul(axisToTest[2].asArray, axis.asArray, 1, 2, zRot, 2, 2);
	axis = Nomd(vec2(0, r2.halfExtents.y));
	Mul(axisToTest[3].asArray, axis.asArray, 1, 2, zRot, 2, 2);
	for (int i = 0; i < 4; ++i)
	{
		if (!OverlapOnAxis(r1, r2, axisToTest[i]))return false;
	}
	return true;
}
bool OrientedRectangleOrientedRectangle(const OrientedRectangle& r1, const OrientedRectangle& r2)
{
	Rectangle2D local1(Point2D(), r1.halfExtents * 2.0f);
	vec2 r = r2.pos - r1.pos;
	OrientedRectangle local2(r2.pos, r2.halfExtents, r2.rot);
	local2.rot = r2.rot - r1.rot;
	float theta = -DEG2RAD(r1.rot);
	float zRot[] = { cosf(theta),sinf(theta), -sinf(theta),cosf(theta) };
	Mul(r.asArray, vec2(r.x, r.y).asArray, 1, 2, zRot, 2, 2);
	local2.pos = r + r1.halfExtents;
	return RectangleOrientedRectangle(local1, local2);
}



//
