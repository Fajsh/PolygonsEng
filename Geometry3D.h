#ifndef _H_GEOMETRY_3D_
#define _H_GEOMETRY_3D_
#include "vectors.h"
#include "matrices.h"

#define AABBSphere(aabb, sphere) \ SphereAABB(sphere, aabb)
#define OBBSphere(obb, sphere) \ SphereObb(sphere, obb)
#define PlaneSphere(plane, sphere) \ SpherePlane(sphere, plane)
#define OBBAABB(obb, aabb) \ AABBOBB(aabb, obb)
#define PlaneAABB(plane, aabb) \ AABBPlane(aabb, plane)
#define PlaneOBB(plane, obb) \ OBBPlane(obb,plane)
#define SphereTriangle(s,t) \ TriangleSphere(t,s)
#define AABBTriangle(a,t) \ TriangleAABB(t,a)
#define OBBTriangle(o,t) \ TriangleOBB(t,0)
#define PlaneTriangle(p,t) \ TrianglePlane(t,p)

//********************structs********************

typedef vec3 Point;

typedef struct Line
{
	Point start;
	Point end;
	inline Line() {}
	inline Line(const Point& s, const Point& e) : start(s), end(e) {}
}Line;

typedef struct Ray
{
	Point origin;
	vec3 dir;
	inline Ray() : dir(0.0f, 0.0f, 1.0f) {}
	inline Ray(const Point& o, const vec3& d) : origin(o), dir(d) { NormalizeDirection(); }
	inline void NormalizeDirection() { Nomd(dir); }
}Ray;

typedef struct Sphere
{
	Point position;
	float radius;
	inline Sphere() : radius(1.0f) {}
	inline Sphere(const Point& p, float r) : position(p), radius(r) {}
}Sphere;

typedef struct AABB
{
	Point position;
	vec3 size;
	inline AABB() : size(1, 1, 1) {}
	inline AABB(const Point& o, const vec3& s) : position(o), size(s) {}
}AABB;

typedef struct OBB
{
	Point position;
	vec3 size;
	mat3 orientation;
	inline OBB() : size(1, 1, 1) {}
	inline OBB(const Point& p, const vec3& s) : position(p), size(s) {}
	inline OBB(const Point& p, const vec3& s, const mat3& o) : position(p), size(s), orientation(o) {}
}OBB;

typedef struct Plane
{
	vec3 normal;
	float distance;
	inline Plane() : normal(1, 0, 0) {}
	inline Plane(const vec3& n, float d) : normal(n), distance(d) {}
}Plane;

typedef struct Triangle
{
	union 
	{
		struct
		{
			Point a;
			Point b;
			Point c;
		};
		Point points[3];
		float values[9];
	};
	inline Triangle() {}
	inline Triangle(const Point& p1, const Point& p2, const Point& p3) : a(p1), b(p2), c(p3) {}
}Triangle;

typedef struct Interval
{
	float min;
	float max;
}Interval;

typedef struct Mesh
{
	int N;
	union
	{
		Triangle* triangles;
		Point* vertices;
		float* values;
	};
	BVHNode* accelerator;
	Mesh() :N(0), values(0), accelerator(0) {}
}Mesh;

typedef struct BVHNode //bounding volume hierarchy
{
	AABB bounds;
	BVHNode* children;
	int N;
	int* triangles;
	BVHNode() : children(0), N(0), triangles(0) {}
}BVHNode;


//********************methods********************
//line

float Len(const Line& line);		//get line length
float LenSq(const Line& line);		//get squared line length

//ray
Ray FromPoints(const Point& from, const Point& to);	//construct ray from points

//AABB
vec3 GetMin(const AABB& aabb);				//get min of aabb
vec3 GetMax(const AABB& aabb);				//get max of aabb
AABB FromMinMax(const vec3& min, const vec3& max);	//construct aabb from min, max

//plane
float PlaneEquation(const Point& pt, const Plane& plane);	//return planeEq

//********************point tests********************
//sphere
bool PointInSphere(const Point& point, const Sphere& sphere);
Point ClosestPoint(const Sphere& sphere, const Point& point);
//aabb
bool PointInAABB(const Point& point, const AABB& aabb);
Point ClosestPoint(const AABB& aabb, const Point& point);
//obb
bool PointInOBB(const Point& point, const OBB& obb);
Point ClosestPoint(const OBB& obb, const Point& point);
//plane
bool PointOnPlane(const Point& point, const Plane& plane);
Point ClosestPoint(const Plane& plane, const Point& point);
//line
bool PointOnLine(const Point& point, const Line& line);
Point ClosestPoint(const Line& line, const Point& point);
//ray
bool PointOnRay(const Point& point, const Ray& ray);
Point ClosestPoint(const Ray& ray, const Point& point);

//********************shape collisions********************

bool SphereSphere(const Sphere& s1, const Sphere& s2);
bool SphereAABB(const Sphere& sphere, const AABB& aabb);
bool SphereOBB(const Sphere& sphere, const OBB& obb);
bool SpherePlane(const Sphere& sphere, const Plane& plane);
bool AABBAABB(const AABB& aabb1, const AABB& aabb2);

Interval GetInterval(const AABB& aabb, const vec3& axis);	//helper function for AABB x OBB collision
Interval GetInterval(const OBB& obb, const vec3& axis);		//helper function for AABB x OBB collision
bool OverlapOnAxis(const AABB& aabb, const OBB& obb, const vec3& axis);	//check if given AABB and OBB overlap on given axis
bool AABBOBB(const AABB& aabb, const OBB& obb);

bool AABBPlane(const AABB& aabb, const Plane& plane);
bool OverlapOnAxis(const OBB& obb1, const OBB& obb2, const vec3& axis);	//check if two given OBB's overlap on given axis
bool OBBOBB(const OBB& obb1, const OBB& obb2);

bool OBBPlane(const OBB& obb, const Plane& plane);
bool PlanePlane(const Plane& plane1, const Plane& plane2);

//********************raycasts, linetests********************
float Raycast(const Sphere& sphere, const Ray& ray);
float Raycast(const AABB& aabb, const Ray& ray);
float Raycast(const OBB& obb, const Ray& ray);
float Raycast(const Plane& plane, const Ray& ray);
bool Linetest(const Sphere& sphere, const Line& line);
bool Linetest(const AABB& aabb, const Line& line);
bool Linetest(const OBB& obb, const Line& line);
bool Linetest(const Plane& plane, const Line& line);

bool PointInTriangle(const Point& point, const Triangle& triangle);
Plane FromTriangle(const Triangle& t);
Point ClosestPoint(const Triangle& t, const Point& p);
bool TriangleSphere(const Triangle& t, const Sphere& s);
Interval GetInterval(const Triangle& triangle, const vec3& axis);
bool OverlapOnAxis(const AABB& aabb, const Triangle& triangle, const vec3& axis);
bool TriangleAABB(const Triangle& t, const AABB& a);
bool OverlapOnAxis(const OBB& obb, const Triangle& triangle, const vec3& axis);
bool TriangleOBB(const Triangle& t, const OBB& obb);
bool TrianglePlane(const Triangle& t, const Plane& p);
bool OverlapOnAxis(const Triangle& t1, const Triangle& t2, const vec3& axis);
bool TriangleTriangle(const Triangle& t1, const Triangle& t2);
vec3 SatCrossEdge(const vec3& a, const vec3& b, const vec3& c, const vec3& d);
bool TriangleTriangleRobust(const Triangle& t1, const Triangle& t2);

vec3 Barycentric(const Point& p, const Triangle& t);
float Raycast(const Triangle& triangle, const Ray& ray);
bool Linetest(const Triangle& triangle, const Line& line);



#endif