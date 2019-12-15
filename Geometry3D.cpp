#include "Geometry3D.h"
#include <cmath>
#include <cfloat>
#include "Compare.h"

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
	return res;
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
Interval GetInterval(const AABB& aabb, const vec3& axis)
{
	vec3 i = GetMin(aabb);
	vec3 a = GetMax(aabb);
	vec3 vertex[8] =
	{
		vec3(i.x, a.y, a.z),
		vec3(i.x, a.y, i.z),
		vec3(i.x, i.y, a.z),
		vec3(i.x, i.y, i.z),
		vec3(a.x, a.y, a.z),
		vec3(a.x, a.y, i.z),
		vec3(a.x, i.y, a.z),
		vec3(a.x, i.y, i.z)
	};
	Interval res;
	res.min = res.max = Dot(axis, vertex[0]);

	for (int i = 1; i < 8; ++i)
	{
		float projection = Dot(axis, vertex[i]);
		res.min = (projection < res.min) ? projection : res.min;
		res.max = (projection > res.max) ? projection : res.max;
	}
	return res;
}
Interval GetInterval(const OBB& obb, const vec3& axis)
{
	vec3 vertex[8];
	vec3 C = obb.position;
	vec3 E = obb.size;
	const float* o = obb.orientation.asArray;
	vec3 A[] =
	{
		vec3(o[0],o[1],o[2]),
		vec3(o[3],o[4],o[5]),
		vec3(o[6],o[7],o[8])
	};
	vertex[0] = C + A[0] * E[0] + A[1] * E[1] + A[2] * E[2];
	vertex[1] = C - A[0] * E[0] + A[1] * E[1] + A[2] * E[2];
	vertex[2] = C + A[0] * E[0] - A[1] * E[1] + A[2] * E[2];
	vertex[3] = C + A[0] * E[0] + A[1] * E[1] - A[2] * E[2];
	vertex[4] = C - A[0] * E[0] - A[1] * E[1] - A[2] * E[2];
	vertex[5] = C + A[0] * E[0] - A[1] * E[1] - A[2] * E[2];
	vertex[6] = C - A[0] * E[0] + A[1] * E[1] - A[2] * E[2];
	vertex[7] = C - A[0] * E[0] - A[1] * E[1] + A[2] * E[2];

	Interval res;
	res.min = res.max = Dot(axis, vertex[0]);
	for (int i = 1; i < 8; ++i)
	{
		float projection = Dot(axis, vertex[i]);
		res.min = (projection < res.min) ? projection : res.min;
		res.max = (projection > res.max) ? projection : res.max;
	}
	return res;
}
bool OverlapOnAxis(const AABB& aabb, const OBB& obb,const vec3& axis)
{
	Interval a = GetInterval(aabb, axis);
	Interval b = GetInterval(obb, axis);
	return((b.min <= a.max) && (a.min <= b.max));
}
bool AABBOBB(const AABB& aabb, const OBB& obb)
{
	const float* o = obb.orientation.asArray;
	vec3 test[15] =
	{
		vec3(1,0,0),
		vec3(0,1,0),
		vec3(0,0,1),
		vec3(o[0],o[1],o[2]),
		vec3(o[3],o[4],o[5]),
		vec3(o[6],o[7],o[8])
	};
	for (int i = 0; i < 3; ++i)
	{
		test[6 + i * 3 + 0] = Cross(test[i], test[0]);
		test[6 + i * 3 + 1] = Cross(test[i], test[1]);
		test[6 + i * 3 + 2] = Cross(test[i], test[2]);
	}

	for (int i = 0; i < 15; ++i)
	{
		if (!OverlapOnAxis(aabb, obb, test[i]))return false;
	}
	return true;
}

bool AABBPlane(const AABB& aabb, const Plane& plane)
{
	float pLen = aabb.size.x * fabsf(plane.normal.x) +
		aabb.size.y * fabsf(plane.normal.y) +
		aabb.size.z * fabsf(plane.normal.z);
	float dot = Dot(plane.normal, aabb.position);
	float dist = dot - plane.distance;
	return fabsf(dist) <= pLen;
}

bool OverlapOnAxis(const OBB& obb1, const OBB& obb2, const vec3& axis)
{
	Interval a = GetInterval(obb1, axis);
	Interval b = GetInterval(obb2, axis);
	return((b.min <= a.max) && (a.min <= b.max));
}

bool OBBOBB(const OBB& obb1, const OBB& obb2)
{
	const float* o1 = obb1.orientation.asArray;
	const float* o2 = obb2.orientation.asArray;
	vec3 test[15] =
	{
		vec3(o1[0],o1[1],o1[2]),
		vec3(o1[3],o1[4],o1[5]),
		vec3(o1[6],o1[7],o1[8]),
		vec3(o2[0],o2[1],o2[2]),
		vec3(o2[3],o2[4],o2[5]),
		vec3(o2[6],o2[7],o2[8])
	};
	for (int i = 0; i < 3; ++i)
	{
		test[6 + i * 3 + 0] = Cross(test[i], test[0]);
		test[6 + i * 3 + 1] = Cross(test[i], test[1]);
		test[6 + i * 3 + 2] = Cross(test[i], test[2]);
	}
	for (int i = 0; i < 15; ++i)
	{
		if (!OverlapOnAxis(obb1, obb2, test[i]))return false;
	}
	return true;
}

bool OBBPlane(const OBB& obb, const Plane& plane)
{
	const float* o = obb.orientation.asArray;
	vec3 rot[] =
	{
		vec3(o[0],o[1],o[2]),
		vec3(o[3],o[4],o[5]),
		vec3(o[6],o[7],o[8])
	};
	vec3 normal = plane.normal;
	float pLen = obb.size.x * fabsf(Dot(normal, rot[0])) +
		obb.size.y * fabsf(Dot(normal, rot[1])) +
		obb.size.z * fabsf(Dot(normal, rot[2]));
	float dot = Dot(plane.normal, obb.position);
	float dist = dot - plane.distance;
	return fabsf(dist) <= pLen;
}

bool PlanePlane(const Plane& plane1, const Plane& plane2)
{
	vec3 d = Cross(plane1.normal, plane2.normal);
	return Dot(d, d) != 0;
}

float Raycast(const Sphere& sphere, const Ray& ray)
{
	vec3 e = sphere.position - ray.origin;
	float rSq = sphere.radius * sphere.radius;
	float eSq = MagSq(e);
	float a = Dot(e, ray.dir);
	float bSq = eSq - (a * a);
	float f = sqrt(rSq - bSq);
	if (rSq - (eSq - (a * a)) < 0.0f)return -1;
	else if (eSq < rSq)return a + f;
	return a - f;
}

float Raycast(const AABB& aabb, const Ray& ray)
{
	vec3 min = GetMin(aabb);
	vec3 max = GetMax(aabb);

	float t1 = (min.x - ray.origin.x) / ray.dir.x;
	float t2 = (max.x - ray.origin.x) / ray.dir.x;
	float t3 = (min.y - ray.origin.y) / ray.dir.y;
	float t4 = (max.y - ray.origin.y) / ray.dir.y;
	float t5 = (min.z - ray.origin.z) / ray.dir.z;
	float t6 = (max.z - ray.origin.z) / ray.dir.z;

	float tmin = fmaxf(fmaxf(fminf(t1, t2), fminf(t3, t4)), fminf(t5, t6));
	float tmax = fminf(fminf(fmaxf(t1, t2), fmaxf(t3, t4)), fmaxf(t5, t6));
	if (tmax < 0)return -1;
	if (tmin > tmax)return - 1;
	if (tmin < 0.0f)return tmax;
	return tmin;

}

float Raycast(const OBB& obb, const Ray& ray)
{
	const float* o = obb.orientation.asArray;
	const float* size = obb.size.asArray;
	vec3 X(o[0], o[1], o[2]);
	vec3 Y(o[3], o[4], o[5]);
	vec3 Z(o[6], o[7], o[8]);
	vec3 p = obb.position - ray.origin;
	vec3 f(Dot(X, ray.dir), Dot(Y, ray.dir), Dot(Z, ray.dir));
	vec3 e(Dot(X, p), Dot(Y, p), Dot(Z, p));
	float t[6] = { 0,0,0,0,0,0 };
	for (int i = 0; i < 3; ++i)
	{
		if (CMP(f[i], 0))
		{
			if (-e[i] - size[i] > 0 || -e[i] + size[i] < 0)return -1;
			f[i] = 0.00001f;
		}
		t[i * 2 + 0] = (e[i] + size[i]) / f[i];
		t[i * 2 + 1] = (e[i] - size[i]) / f[i];
	}
	float tmin = fmaxf(fmaxf(fminf(t[0], t[1]), fminf(t[2], t[3])), fminf(t[4], t[5]));
	float tmax = fminf(fminf(fmaxf(t[0], t[1]), fmaxf(t[2], t[3])), fmaxf(t[4], t[5]));
	if (tmax < 0)return -1;
	if (tmin > tmax)return -1;
	if (tmin < 0.0f)return tmax;
	return tmin;
}

float Raycast(const Plane& plane, const Ray& ray)
{
	float nd = Dot(ray.dir, plane.normal);
	float pn = Dot(ray.origin, plane.normal);
	if (nd >= 0.0f)return -1;
	float t = (plane.distance - pn) / nd;
	if (t >= 0.0f)return t;
	return -1;
}

bool Linetest(const Sphere& sphere, const Line& line)
{
	Point closest = ClosestPoint(line, sphere.position);
	float distSq = MagSq(sphere.position - closest);
	return distSq <= (sphere.radius * sphere.radius);
}

bool Linetest(const AABB& aabb, const Line& line)
{
	Ray ray;
	ray.origin = line.start;
	ray.dir = Nomd(line.end - line.start);
	float t = Raycast(aabb, ray);
	return t >= 0 && t * t <= LenSq(line);
}

bool Linetest(const OBB& obb, const Line& line)
{
	Ray ray;
	ray.origin = line.start;
	ray.dir = Nomd(line.end - line.start);
	float t = Raycast(obb, ray);
	return t >= 0 && t * t <= LenSq(line);
}

bool Linetest(const Plane& plane, const Line& line)
{
	vec3 ab = line.end - line.start;
	float nA = Dot(plane.normal, line.start);
	float nAB = Dot(plane.normal, ab);
	float t = (plane.distance - nA) / nAB;
	return t >= 0.0f && t <= 1.0f;
}

bool PointInTriangle(const Point& point, const Triangle& triangle)
{
	vec3 a = triangle.a - point;
	vec3 b = triangle.b - point;
	vec3 c = triangle.c - point;

	vec3 normPBC = Cross(b, c);
	vec3 normPCA = Cross(c, a);
	vec3 normPAB = Cross(a, b);
	if (Dot(normPBC, normPCA) < 0.0f)return false;
	else if (Dot(normPBC, normPAB) < 0.0f)return false;
	return true;

}

Plane FromTriangle(const Triangle& t)
{
	Plane res;
	res.normal = Nomd(Cross(t.b - t.a, t.c - t.a));
	res.distance = Dot(res.normal, t.a);
	return res;
}

Point ClosestPoint(const Triangle& t, const Point& p)
{
	Plane plane = FromTriangle(t);
	if (PointInTriangle(p, t))return p;
	Point c1 = ClosestPoint(Line(t.a, t.b), p);
	Point c2 = ClosestPoint(Line(t.b, t.c), p);
	Point c3 = ClosestPoint(Line(t.c, t.a), p);
	float magSq1 = MagSq(p - c1);
	float magSq2 = MagSq(p - c2);
	float magSq3 = MagSq(p - c3);

	if (magSq1 < magSq2 && magSq1 < magSq3)return c1;
	else if (magSq2 < magSq1 && magSq2 < magSq3)return c2;
	return c3;
}

bool TriangleSphere(const Triangle& t, const Sphere& s)
{
	Point closest = ClosestPoint(t, s.position);
	float magSq = MagSq(closest - s.position);
	return magSq <= (s.radius * s.radius);
}

Interval GetInterval(const Triangle& triangle, const vec3& axis)
{
	Interval res;
	res.min = Dot(axis, triangle.points[0]);
	res.max = res.min;

	for (int i = 1; i < 3; ++i)
	{
		float val = Dot(axis, triangle.points[i]);
		res.min = fminf(res.min, val);
		res.max = fmaxf(res.max, val);
	}
	return res;
}

bool OverlapOnAxis(const AABB& aabb, const Triangle& triangle, const vec3& axis)
{
	Interval a = GetInterval(aabb, axis);
	Interval b = GetInterval(triangle, axis);
	return((b.min <= a.max) && (a.min <= b.max));
}

bool TriangleAABB(const Triangle& t, const AABB& a)
{
	vec3 f0 = t.b - t.a;
	vec3 f1 = t.c - t.b;
	vec3 f2 = t.a - t.c;
	vec3 u0(1.0f, 0.0f, 0.0f);
	vec3 u1(0.0f, 1.0f, 0.0f);
	vec3 u2(0.0f, 0.0f, 1.0f);
	vec3 test[13] =
	{
		u0,
		u1,
		u2,
		Cross(f0,f1),
		Cross(u0,f0),
		Cross(u0,f1),
		Cross(u0,f2),
		Cross(u1,f0),
		Cross(u1,f1),
		Cross(u1,f2),
		Cross(u2,f0),
		Cross(u2,f1),
		Cross(u2,f2)
	};
	for (int i = 0; i < 13; ++i)
	{
		if (!OverlapOnAxis(a, t, test[i]))return false;
	}
	return true;
}

bool OverlapOnAxis(const OBB& obb, const Triangle& triangle, const vec3& axis)
{
	Interval a = GetInterval(obb, axis);
	Interval b = GetInterval(triangle, axis);
	return((b.min <= a.max) && (a.min <= b.max));
}

bool TriangleOBB(const Triangle& t, const OBB& obb)
{
	vec3 f0 = t.b - t.a;
	vec3 f1 = t.c - t.b;
	vec3 f2 = t.a - t.c;

	const float* orientation = obb.orientation.asArray;
	vec3 u0(orientation[0], orientation[1], orientation[2]);
	vec3 u1(orientation[3], orientation[4], orientation[5]);
	vec3 u2(orientation[6], orientation[7], orientation[8]);

	vec3 test[13] =
	{
		u0,
		u1,
		u2,
		Cross(f0,f1),
		Cross(u0,f0),
		Cross(u0,f1),
		Cross(u0,f2),
		Cross(u1,f0),
		Cross(u1,f1),
		Cross(u1,f2),
		Cross(u2,f0),
		Cross(u2,f1),
		Cross(u2,f2)
	};
	for (int i = 0; i < 13; ++i)
	{
		if (!OverlapOnAxis(obb, t, test[i]))return false;
	}
	return true;
}

bool TrianglePlane(const Triangle& t, const Plane& p)
{
	float side1 = PlaneEquation(t.a, p);
	float side2 = PlaneEquation(t.b, p);
	float side3 = PlaneEquation(t.c, p);
	if (CMP(side1, 0) && CMP(side2, 0) && CMP(side3, 0))return true;
	if (side1 > 0 && side2 > 0 && side3 > 0)return false;
	if (side1 < 0 && side2 < 0 && side3 < 0)return false;
	return true;
}

bool OverlapOnAxis(const Triangle& t1, const Triangle& t2, const vec3& axis)
{
	Interval a = GetInterval(t1, axis);
	Interval b = GetInterval(t2, axis);
	return((b.min <= a.max) && (a.min <= b.max));
}

bool TriangleTriangle(const Triangle& t1, const Triangle& t2)
{
	vec3 t1_f0 = t1.b - t1.a;
	vec3 t1_f1 = t1.c - t1.b;
	vec3 t1_f2 = t1.a - t1.c;

	vec3 t2_f0 = t2.b - t2.a;
	vec3 t2_f1 = t2.c - t2.b;
	vec3 t2_f2 = t2.a - t2.c;

	vec3 axisToTest[] =
	{
		Cross(t1_f0,t1_f1),
		Cross(t2_f0,t2_f1),

		Cross(t2_f0,t1_f0),
		Cross(t2_f0,t1_f1),

		Cross(t2_f0,t2_f2),
		Cross(t2_f1,t1_f0),

		Cross(t2_f1,t1_f1),
		Cross(t2_f1,t1_f2),

		Cross(t2_f2,t1_f0),
		Cross(t2_f2,t1_f1),
		Cross(t2_f2,t1_f2)
	};

	for (int i = 0; i < 11; ++i)
	{
		if (!OverlapOnAxis(t1, t2, axisToTest[i]))return false;
	}
	return true;
}

vec3 SatCrossEdge(const vec3& a, const vec3& b, const vec3& c, const vec3& d)
{
	vec3 ab = a - b;
	vec3 cd = c - d;
	vec3 res = Cross(ab, cd);
	if (!CMP(MagSq(res), 0))return res;
	else
	{
		vec3 axis = Cross(ab, c - a);
		res = Cross(ab, axis);
		if (!CMP(MagSq(res), 0))return res;
	}
	return vec3();
}

bool TriangleTriangleRobust(const Triangle& t1, const Triangle& t2)
{
	vec3 axisToTest[] =
	{
		SatCrossEdge(t1.a,t1.b,t1.b,t1.c),
		SatCrossEdge(t2.a,t2.b,t2.b,t2.c),

		SatCrossEdge(t2.a,t2.b,t1.a,t1.b),
		SatCrossEdge(t2.a,t2.b,t1.b,t1.c),
		SatCrossEdge(t2.a,t2.b,t1.c,t1.a),

		SatCrossEdge(t2.b,t2.c,t1.a,t1.b),
		SatCrossEdge(t2.b,t2.c,t1.b,t1.c),
		SatCrossEdge(t2.b,t2.c,t1.c,t1.a),

		SatCrossEdge(t2.c,t2.a,t1.a,t1.b),
		SatCrossEdge(t2.c,t2.a,t1.b,t1.c),
		SatCrossEdge(t2.c,t2.a,t1.c,t1.a)
	};
	for (int i = 0; i < 11; ++i)
	{
		if (!OverlapOnAxis(t1, t2, axisToTest[i]))
		{
			if (!CMP(MagSq(axisToTest[i]), 0))return false;
		}
	}
	return true;
}

vec3 Barycentric(const Point& p, const Triangle& t)
{
	vec3 ap = p - t.a;
	vec3 bp = p - t.b;
	vec3 cp = p - t.c;

	vec3 ab = t.b - t.a;
	vec3 ac = t.c - t.a;
	vec3 bc = t.c - t.b;
	vec3 cb = t.b - t.c;
	vec3 ca = t.a - t.c;

	vec3 v = ab - Project(ab, cb);
	float a = 1.0f - (Dot(v, ap) / Dot(v, ab));

	v = bc - Project(bc, ac);
	float b = 1.0f - (Dot(v, bp) / Dot(v, bc));

	v = ca - Project(ca, ab);
	float c = 1.0f - (Dot(v, cp) / Dot(v, ca));

	return vec3(a, b, c);
}

float Raycast(const Triangle& triangle, const Ray& ray)
{
	Plane plane = FromTriangle(triangle);
	float t = Raycast(plane, ray);
	if (t < 0.0f)return t;
	Point res = ray.origin + ray.dir * t;

	vec3 barycentric = Barycentric(res, triangle);
	if (barycentric.x >= 0.0f && barycentric.x <= 1.0f && barycentric.y >= 0.0f && barycentric.y <= 1.0f && barycentric.z >= 0.0f && barycentric.z <= 1.0f)return t;
	return -1;
}

bool Linetest(const Triangle& triangle, const Line& line)
{
	Ray ray;
	ray.origin = line.start;
	ray.dir = Nomd(line.end - line.start);
	float t = Raycast(triangle, ray);
	return t >= 0 && t * t <= LenSq(line);
}



