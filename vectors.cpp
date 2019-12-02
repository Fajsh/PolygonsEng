#include "vectors.h"
#include "Compare.h"
#include <cmath>
#include <cfloat>

//
//Operators
//vec2
vec2 operator+(const vec2& l, const vec2& r)
{
	return { l.x + r.x,l.y + r.y };
}
vec2 operator-(const vec2& l, const vec2& r)
{
	return { l.x - r.x,l.y - r.y };
}
vec2 operator*(const vec2& l, const vec2& r)
{
	return { l.x * r.x,l.y * r.y };
}
vec2 operator*(const vec2& l, float r)
{
	return { l.x * r,l.y * r };
}
bool operator==(const vec2& l, const vec2& r)
{
	return CMP(l.x, r.x) && CMP(l.y, r.y);
}
bool operator!=(const vec2& l, const vec2& r)
{
	return !(l == r);
}
std::ostream& operator<<(std::ostream& os, const vec2& m)
{
	os << "[" << m.x << "; " << m.y << "]";
	return os;
}

//vec3
vec3 operator+(const vec3& l, const vec3& r)
{
	return { l.x + r.x,l.y + r.y,l.z + r.z };
}
vec3 operator-(const vec3& l, const vec3& r)
{
	return { l.x - r.x,l.y - r.y,l.z - r.z };
}
vec3 operator*(const vec3& l, const vec3& r)
{
	return { l.x * r.x,l.y * r.y,l.z * r.z };
}
vec3 operator*(const vec3& l, float r)
{
	return { l.x * r,l.y * r,l.z * r };
}
bool operator==(const vec3& l, const vec3& r)
{
	return CMP(l.x, r.x) && CMP(l.y, r.y) && CMP(l.z, r.z);
}
bool operator!=(const vec3& l, const vec3& r)
{
	return !(l == r);
}
std::ostream& operator<<(std::ostream& os, const vec3& m)
{
	os << "[" << m.x << "; " << m.y << "; " << m.z << "]";
	return os;
}


//Methods
//vec2
float Dot(const vec2& l, const vec2& r)
{
	return l.x * r.x + l.y * r.y;
}
float Mag(const vec2& v)
{
	return sqrtf(Dot(v, v));
}
float MagSq(const vec2& v)
{
	return Dot(v, v);
}
float Dist(const vec2& l, const vec2& r)
{
	vec2 t = l - r;
	return Mag(t);
}
void Nome(vec2& v)
{
	v = v * (1.0f / Mag(v));
}
vec2 Nomd(const vec2& v)
{
	return v * (1.0f / Mag(v));
}
float Angle(const vec2& l, const vec2& r)
{
	float m = sqrtf(MagSq(l) * MagSq(r));
	return acos(Dot(l, r) / m);
}
vec2 Project(const vec2& len, const vec2& dir)
{
	float dot = Dot(len, dir);
	float magSq = MagSq(dir);
	return dir * (dot / magSq);
}
vec2 Perp(const vec2& len, const vec2& dir)
{
	return len - Project(len, dir);
}

vec2 Refl(const vec2& v, const vec2& norm)
{
	float d = Dot(v, norm);
	return v - norm * (d * 2.0f);
}


//vec3
float Dot(const vec3& l, const vec3& r)
{
	return l.x * r.x + l.y * r.y + l.z * r.z;
}
float Mag(const vec3& v)
{
	return sqrtf(Dot(v, v));
}
float MagSq(const vec3& v)
{
	return Dot(v, v);
}
float Dist(const vec3& l, const vec3& r)
{
	vec3 t = l - r;
	return Mag(t);
}
void Nome(vec3& v)
{
	v = v * (1.0f / Mag(v));
}
vec3 Nomd(const vec3& v)
{
	return v * (1.0f / Mag(v));
}
vec3 Cross(const vec3& l, const vec3& r)
{
	vec3 res;
	res.x = l.y * r.z - l.z * r.y;
	res.y = l.z * r.x - l.x * r.z;
	res.z = l.x * r.y - l.y * r.x;
	return res;
}
float Angle(const vec3& l, const vec3& r)
{
	float m = sqrtf(MagSq(l) * MagSq(r));
	return acos(Dot(l, r) / m);
}
vec3 Project(const vec3& len, const vec3& dir)
{
	float dot = Dot(len, dir);
	float magSq = MagSq(dir);
	return dir * (dot / magSq);
}
vec3 Perp(const vec3& len, const vec3& dir)
{
	return len - Project(len, dir);
}

vec3 Refl(const vec3& v, const vec3& norm)
{
	float d = Dot(v, norm);
	return v - norm * (d * 2.0f);
}




