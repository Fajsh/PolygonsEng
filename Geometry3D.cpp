#include "Geometry3D.h"
#include <cmath>
#include <cfloat>

//********************methods********************
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
//********************point tests********************
//sphere
bool PointInSphere(const Point& point, const Sphere& sphere)
{
	float magSq = MagSq(point - sphere.position);
	float radSq = sphere.radius * sphere.radius;
	return magSq < radSq;
}
Point ClosestPoint(const Sphere& sphere, const Point& point)
{
	vec3 sphereToPoint = point - sphere.position;
	Nome(sphereToPoint);
	sphereToPoint = sphereToPoint * sphere.radius;
	return sphereToPoint + sphere.position;
}
//aabb
bool PointInAABB(const Point& point, const AABB& aabb)
{
	Point min = GetMin(aabb);
	Point max = GetMax(aabb);
	if (point.x < min.x || point.y < min.y || point.z < min.z)return false;
	if (point.x > max.x || point.y > max.y || point.z > max.z)return false;
	return true;
}
Point ClosestPoint(const AABB& aabb, const Point& point)
{
	Point res = point;
	Point min = GetMin(aabb);
	Point max = GetMax(aabb);
	res.x = (res.x < min.x) ? min.x : res.x;
	res.y = (res.y < min.y) ? min.y : res.y;
	res.z = (res.z < min.z) ? min.z : res.z;

	res.x = (res.x > max.x) ? max.x : res.x;
	res.y = (res.y > max.y) ? max.y : res.y;
	res.z= (res.z > max.z) ? max.z : res.z;
}
//obb
bool PointInOBB(const Point& point, const OBB& obb)
{
	vec3 dir = point - obb.position;
	for (int i = 0; i < 3;++i)
	{
		const float* orientation = &obb.orientation.asArray[i * 3];
		vec3 axis(orientation[0], orientation[1], orientation[2]);
		float distance = Dot(dir, axis);
		if (distance > obb.size.asArray[i])return false;
		if (distance < -obb.size.asArray[i])return false;
	}
	return true;
}
Point ClosestPoint(const OBB& obb, const Point& point)
{
	Point res = obb.position;
	vec3 dir = point - obb.position;
	for (int i = 0; i < 3; ++i)
	{
		const float* orientation = &obb.orientation.asArray[i * 3];
		vec3 axis(orientation[0], orientation[1], orientation[2]);
		float distance = Dot(dir, axis);

		if (distance > obb.size.asArray[i])distance = obb.size.asArray[i];
		if (distance < -obb.size.asArray[i])distance = -obb.size.asArray[i];
		res = res + (axis * distance);
	}
	return res;
}
//plane
bool PointOnPlane(const Point& point, const Plane& plane)
{
	float dot = Dot(point, plane.normal);
	return dot - plane.distance == 0.0f;
}
Point ClosestPoint(const Plane& plane, const Point& point)
{
	float dot = Dot(plane.normal, point);
	float dist = dot - plane.distance;
	return point - plane.normal * dist;
}
//line
bool PointOnLine(const Point& point, const Line& line)
{
	Point closest = ClosestPoint(line, point);
	float distanceSq = MagSq(closest - point);
	return distanceSq == 0.0f;
}
Point ClosestPoint(const Line& line, const Point& point)
{
	vec3 lVec = line.end - line.start;
	float t = Dot(point - line.start, lVec) / Dot(lVec, lVec);
	t = fmaxf(t, 0.0f);
	t = fminf(t, 1.0f);
	return line.start + lVec * t;
}
//ray
bool PointOnRay(const Point& point, const Ray& ray)
{
	if (point == ray.origin)return true;
	vec3 norm = point - ray.origin;
	Nomd(norm);
	float diff = Dot(norm, ray.dir);
	return diff == 1.0f;
}
Point ClosestPoint(const Ray& ray, const Point& point)
{
	float t = Dot(point - ray.origin, ray.dir);
	t = fmaxf(t,0.0f);
	return Point(ray.origin + ray.dir * t);
}

//********************shape collisions********************
bool SphereSphere(const Sphere& s1, const Sphere& s2)
{
	float radSum = s1.radius + s2.radius;
	float sqDist = MagSq(s1.position - s2.position);
	return sqDist < radSum * radSum;
}
bool SphereAABB(const Sphere& sphere, const AABB& aabb)
{
	Point closestPoint = ClosestPoint(aabb, sphere.position);
	float distSq = MagSq(sphere.position - closestPoint);
	float radiusSq = sphere.radius * sphere.radius;
	return distSq < radiusSq;
}
bool SphereOBB(const Sphere& sphere, const OBB& obb)
{
	Point closestPoint = ClosestPoint(obb, sphere.position);
	float distSq = MagSq(sphere.position - closestPoint);
	float radiusSq = sphere.radius * sphere.radius;
	return distSq < radiusSq;
}
bool SpherePlane(const Sphere& sphere, const Plane& plane)
{
	Point closestPoint = ClosestPoint(plane, sphere.position);
	float distSq = MagSq(sphere.position - closestPoint);
	float radiusSq = sphere.radius * sphere.radius;
	return distSq < radiusSq;
}

bool AABBAABB(const AABB& aabb1, const AABB& aabb2)
{
	Point aMax = GetMax(aabb1);
	Point aMin = GetMin(aabb1);

	Point bMax = GetMax(aabb2);
	Point bMin = GetMin(aabb2);

	return (aMin.x <= bMax.x && aMax.x >= bMin.x) &&
		(aMin.y <= bMax.y && aMax.y >= bMin.y) &&
		(aMin.z <= bMax.z && aMax.z >= bMin.z);
}

