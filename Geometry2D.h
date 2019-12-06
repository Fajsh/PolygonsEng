#ifndef _H_2D_GEOMETRY_
#define _H_2D_GEOMETRY_
#include "vectors.h"

#define PointLine(point, line) \ PointOnLine(point, line)
#define LinePoint(line, point) \ PointOnLine(point,line)
#define CircleLine(circle, line) \ LineCircle(line,circle)
#define RectangleLine(rectangle, line) \ LineRectangle(line,rectangle)
#define OrientedRectangleLine(rectangle, line) \ LineOrientedRectangle(line,rectangle)


typedef vec2 Point2D;

typedef struct Line2D
{
	Point2D start;
	Point2D end;
	inline Line2D() {}
	inline Line2D(const Point2D& s, const Point2D& e) : start(s), end(e) {}
}Line2D;

typedef struct Circle
{
	Point2D pos;
	float r;
	inline Circle() : r(1.0f) {}
	inline Circle(const Point2D& p, float radius) : pos(p), r(radius) {}
}Circle;

typedef struct Rectangle2D
{
	Point2D pos;
	vec2 size;
	inline Rectangle2D() : size(1, 1) {}
	inline Rectangle2D(const Point2D& p, const vec2& s) : pos(p), size(s) {}
}Rectangle2D;

typedef struct OrientedRectangle
{
	Point2D pos;
	vec2 halfExtents;
	float rot;
	inline OrientedRectangle() : halfExtents(1.0f, 1.0f), rot(0.0f) {}
	inline OrientedRectangle(const Point2D& p, const vec2& e) : pos(p), halfExtents(e), rot(0.0f) {}
	inline OrientedRectangle(const Point2D& p, const vec2& e, float r) : pos(p), halfExtents(e), rot(r) {}
}OrientedRectangle;




//Methods
//line2d
float Len(const Line2D& line);
float LenSq(const Line2D& line);

//rectangle2d
vec2 GetMin(const Rectangle2D& rect);
vec2 GetMax(const Rectangle2D& rect);
Rectangle2D FromMinMax(const vec2& min, const vec2& max);

//containment
bool PointOnLine(const Point2D& p, const Line2D& l);
bool PointInCircle(const Point2D& p, const Circle& c);
bool PointInRectangle(const Point2D& p, const Rectangle2D& r);
bool PointInOrientedRectangle(const Point2D& p, const OrientedRectangle& r);

//intersection(line)
bool LineCircle(const Line2D& l, const Circle& c);
bool LineRectangle(const Line2D& l, const Rectangle2D& r);
bool LineOrientedRectangle(const Line2D& l, const OrientedRectangle& r);
#endif