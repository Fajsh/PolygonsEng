#ifndef _H_MATH_MATRICES_
#define _H_MATH_MATRICES_
#include "vectors.h"

typedef struct mat2
{
	union
	{
		struct
		{
			float _11, _12,
				  _21, _22;
		};
		float asArray[4];
	};
	inline float* operator[](int i)
	{
		return &(asArray[i * 2]);
	}
	inline mat2()
	{
		_11 = _22 = 1.0f;
		_12 = _21 = 1.0f;
	}
	inline mat2(float f11, float f12, float f21, float f22)
	{
		_11 = f11; _12 = f12;
		_21 = f21; _22 = f22;
	}
}mat2;

typedef struct mat3
{
	union
	{
		struct
		{
			float _11, _12, _13, 
				  _21, _22, _23, 
				  _31, _32, _33;
		};
		float asArray[9];
	};
	inline float* operator[](int i)
	{
		return &(asArray[i * 3]);
	}
	inline mat3()
	{
		_11 = _22 = _33 = 1.0f;
		_12 = _13 = _21 = 0.0f;
		_23 = _31 = _32 = 0.0f;
	}
	inline mat3(float f11, float f12, float f13, float f21, float f22, float f23, float f31, float f32,float f33)
	{
		_11 = f11; _12 = f12; _13 = f13;
		_21 = f21; _22 = f22; _23 = f23;
		_31 = f31; _32 = f32; _33 = f33;
	}
}mat3;

typedef struct mat4
{
	union
	{
		struct
		{
			float _11, _12, _13, _14,
				  _21, _22, _23, _24,
				  _31, _32, _33, _34,
				  _41, _42, _43, _44;
		};
		float asArray[16];
	};
	inline float* operator[](int i)
	{
		return &(asArray[i * 4]);
	}
	inline mat4()
	{
		_11 = _22 = _33 = _44 = 1.0f;
		_12 = _13 = _14 = _21 = 0.0f;
		_23 = _24 = _31 = _32 = 0.0f;
		_34 = _41 = _42 = _43 = 0.0f;
	}
	inline mat4(float f11, float f12, float f13, float f14,
				float f21, float f22, float f23, float f24,
				float f31, float f32, float f33, float f34,
				float f41, float f42, float f43, float f44)
	{
		_11 = f11; _12 = f12; _13 = f13; _14 = f14;
		_21 = f21; _22 = f22; _23 = f23; _24 = f24;
		_31 = f31; _32 = f32; _33 = f33; _34 = f34;
		_41 = f41; _42 = f42; _43 = f43; _44 = f44;
	}

}mat4;
/*
	m - matrix
	i - row
	j - column
	ACCESS TYPES:
	m[i][j]		i,j start from 0!
	m._ij		i,j start from 1!
	m.asArray[dimension * i + j]
*/

//Operators
//mat2
mat2 operator*(const mat2& m, float s);
mat2 operator*(const mat2& matL, const mat2& matR);

//mat3
mat3 operator*(const mat3& m, float s);
mat3 operator*(const mat3& matL, const mat3& matR);

//mat4
mat4 operator*(const mat4& m, float s);
mat4 operator*(const mat4& matL, const mat4& matR);

//Methods
//general
void Transpose(const float* srcMat, float* dstMat, int srcRow, int srcCol);	//get transposed matrix
bool Mul(float* res, const float* matL, int lRow, int lCol, const float* matR, int rRow, int rCol); //multiply two matrices
void Cofactor(float* out, const float* min, int row, int col);// Get cofactor of matrix

//mat2
mat2 Transpose(const mat2& m);					//get transposed matrix
float Determinant(const mat2& m);				//get eterminant of matrix
mat2 Minor(const mat2& mat);					//get minor of matrix
mat2 Cut(const mat3& mat, int row, int col);	//get matrix without specific row and column
mat2 Cofactor(const mat2& mat);					//get cofactor of matrix
mat2 Adjugate(const mat2& mat);					//get adjugate matrix
mat2 Inv(const mat2& mat);						//get inversed matrix

//mat3
mat3 Transpose(const mat3& m);
mat3 Minor(const mat3& mat);
mat3 Cofactor(const mat3& mat);
mat3 Cut(const mat4& mat, int row, int col);
float Determinant(const mat3& m);
mat3 Adjugate(const mat3& mat);
mat3 Inv(const mat3& mat);

//mat4
mat4 Transpose(const mat4& m);
mat4 Minor(const mat4& mat);
mat4 Cofactor(const mat4& mat);
float Determinant(const mat4& mat);
mat4 Adjugate(const mat4& mat);
mat4 Inv(const mat4& mat);

//matrix transformations
//translation
mat4 Translation(float x, float y, float z);
mat4 Translation(const vec3& pos);
vec3 GetTranslation(const mat4& mat);
//scale
mat4 Scale(float x, float y, float z);
mat4 Scale(const vec3& vec);
vec3 GetScale(const mat4& mat);
//rotate
mat4 Rot(float pitch, float yaw, float roll);
mat3 Rot3x3(float pitch, float yaw, float roll);

mat4 ZRot(float angle);
mat3 ZRot3x3(float angle);

mat4 XRot(float angle);
mat3 XRot3x3(float angle);

mat4 YRot(float angle);
mat3 YRot3x3(float angle);

// rotate around given axis
mat4 AxisAngle(const vec3& axis, float angle);
mat3 AxisAngle3x3(const vec3& axis, float angle);

//multiply vec and mat
vec3 MultiplyPoint(const vec3& vec, const mat4& mat);
vec3 MultiplyVector(const vec3& vec, const mat4& mat);
vec3 MultiplyVector(const vec3& vec, const mat3& mat);

//transform
mat4 Transform(const vec3& scale, const vec3& eulerRot, const vec3& translate);
mat4 Transform(const vec3& scale, const vec3& rotAxis, float rotAngle, const vec3& translate);

//view
mat4 LookAt(const vec3& pos, const vec3& trg, const vec3& up);

//projection
mat4 Projection(float fov, float aspect, float zNear, float zFar);
mat4 Ortho(float left, float right, float bottom,float top, float zNear, float zFar);
#endif


