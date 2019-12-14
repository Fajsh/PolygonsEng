#ifndef _H_2D_GEOMETRY_
#define _H_2D_GEOMETRY_
#include "vectors.h"

#define PointLine(point, line) \ PointOnLine(point, line)
#define LinePoint(line, point) \ PointOnLine(point,line)
#define CircleLine(circle, line) \ LineCircle(line,circle)
#define RectangleLine(rectangle, line) \ LineRectangle(line,rectangle)
#define OrientedRectangleLine(rectangle, line) \ LineOrientedRectangle(line,rectangle)
#define RectangleCircle(rectangle, circle) \ CircleRectangle(circle, rectangle)
#define OrientedRectangleCircle(rectangle, circle) \ CircleOrientedRectangle(circle,rectangle)
#define OrientedRectangleRectangle(oriented, regular) \ RectangleOrientedRectangle(regular,oriented)


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

typedef struct Interval2D
{
	float min;
	float max;
}Interval2D;

typedef struct BoundingShape
{
	int circleNum;
	Circle* circles;
	int rectNum;
	Rectangle2D* rectangles;
	inline BoundingShape() : circleNum(0), circles(0), rectNum(0), rectangles(0) {}
}BoundingShape;

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



//collisions
bool CircleCircle(const Circle& c1, const Circle& c2);
bool CircleRectangle(const Circle& c, const Rectangle2D& r);
bool CircleOrientedRectangle(const Circle& c, const OrientedRectangle& r);
bool RectangleRectangle(const Rectangle2D& r1, const Rectangle2D& r2);
//collisions - Separating Axis Theorem
Interval2D getInterval(const Rectangle2D& rect, const vec2& axis);
bool OverlapOnAxis(const Rectangle2D& r1, const Rectangle2D& r2, const vec2& axis);
bool RectangleRectangleSAT(const Rectangle2D& r1, const Rectangle2D& r2);
Interval2D getInterval(const OrientedRectangle& rect, const vec2& axis);
bool OverlapOnAxis(const Rectangle2D& r1, const OrientedRectangle& r2, const vec2& axis);
bool RectangleOrientedRectangle(const Rectangle2D& r1, const OrientedRectangle r2);
bool OrientedRectangleOrientedRectangle(const OrientedRectangle& r1, const OrientedRectangle& r2);

Circle ContainingCircle(Point2D* pArray, int arrayCount);
Rectangle2D ContainingRectangle(Point2D* pArray, int arrayCount);

bool PointInShape(const BoundingShape& shape, const Point2D point);

#endif