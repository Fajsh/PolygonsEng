#include "Geometry3D.h"
#include <cmath>
#include <cfloat>

//Methods
//line
float Len(const Line& line)
{
	return Mag(line.start - line.end);
}
float LenSq(const Line& line)
{
	return MagSq(line.start - line.end);
}
//ray
Ray FromPoints(const Point& from, const Point& to)
{
	return Ray(from, Nomd(to - from));
}
//AABB
vec3 GetMin(const AABB& aabb)
{
	vec3 p1 = aabb.position + aabb.size;
	vec3 p2 = aabb.position - aabb.size;

	return vec3(fminf(p1.x, p2.x),
		fminf(p1.y, p2.y),
		fminf(p1.z, p2.z));
}
vec3 GetMax(const AABB& aabb)
{
	vec3 p1 = aabb.position + aabb.size;
	vec3 p2 = aabb.position - aabb.size;

	return vec3(fmaxf(p1.x, p2.x),
		fmaxf(p1.y, p2.y),
		fmaxf(p1.z, p2.z));
}
AABB FromMinMax(const vec3& min, const vec3& max)
{
	return AABB((min + max) * 0.5f, (max - min) * 0.5f);
}
//plane
float PlaneEquation(const Point& pt, const Plane& plane)
{
	return Dot(pt, plane.normal) - plane.distance;
}
