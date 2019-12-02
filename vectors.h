#ifndef _H_MATH_VECTORS_
#define _H_MATH_VECTORS_

#define RAD2DEG(x) ((x)*57.295754f)
#define DEG2RAD(x) ((x)*0.0174533f)

#include <ostream>
typedef struct vec2 {
	union {
		struct {
			float x;
			float y;
		};
		float asArray[2];
	};

	inline float& operator[](int i) {
		return asArray[i];
	}

	inline vec2() : x(0.0f), y(0.0f) { }
	inline vec2(float _x, float _y) : x(_x), y(_y) { }
} vec2;

typedef struct vec3 {
	union {
		struct {
			float x;
			float y;
			float z;
		};
		float asArray[3];
	};

	inline float& operator[](int i) {
		return asArray[i];
	}

	inline vec3() : x(0.0f), y(0.0f), z(0.0f) { }
	inline vec3(float _x, float _y, float _z) : x(_x), y(_y), z(_z) { }

} vec3;


//Operators
//vec2
vec2 operator+(const vec2& l, const vec2& r);
vec2 operator-(const vec2& l, const vec2& r);
vec2 operator*(const vec2& l, const vec2& r);
vec2 operator*(const vec2& l, float r);
bool operator==(const vec2& l, const vec2& r);
bool operator!=(const vec2& l, const vec2& r);
std::ostream& operator<<(std::ostream& os, const vec2& m);

//vec3
vec3 operator+(const vec3& l, const vec3& r);
vec3 operator-(const vec3& l, const vec3& r);
vec3 operator*(const vec3& l, const vec3& r);
vec3 operator*(const vec3& l, float r);
bool operator==(const vec3& l, const vec3& r);
bool operator!=(const vec3& l, const vec3& r);
std::ostream& operator<<(std::ostream& os, const vec3& m);

//Methods
//vec2
float Dot(const vec2& l, const vec2& r);		//get dot product
float Mag(const vec2& v);						//get magnitude(length)
float MagSq(const vec2& v);						//get magnitude^2 - faster than Mag()
float Dist(const vec2& l, const vec2& r);		//get distance between(points!)
void Nome(vec2& v);								//normalize this vector
vec2 Nomd(const vec2& v);						//get normalized vector
float Angle(const vec2& l, const vec2& r);		//get angle between vectors
vec2 Project(const vec2& len, const vec2& dir);	//get projection of the vector
vec2 Perp(const vec2& len, const vec2& dir);	//get perpendicular component of the vector
vec2 Refl(const vec2& v, const vec2& norm);		//get reflection of the vector

//vec3
float Dot(const vec3& l, const vec3& r);
float Mag(const vec3& v);
float MagSq(const vec3& v);
float Dist(const vec3& l, const vec3& r);
void Nome(vec3& v);
vec3 Nomd(const vec3& v);
vec3 Cross(const vec3& l, const vec3& r);	//get cross product
float Angle(const vec3& l, const vec3& r);
vec3 Project(const vec3& len, const vec3& dir);
vec3 Perp(const vec3& len, const vec3& dir);
vec3 Refl(const vec3& v, const vec3& norm);
#endif

