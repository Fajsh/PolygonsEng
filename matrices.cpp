#include "matrices.h"
#include "Compare.h"
#include <cmath>
#include <cfloat>

//Operators
//mat2
mat2 operator*(const mat2& m, float s)
{
	mat2 res;
	for (int i = 0; i < 4; ++i)
	{
		res.asArray[i] = m.asArray[i] * s;
	}
	return res;
}
mat2 operator*(const mat2& matL, const mat2& matR)
{
	mat2 res;
	Mul(res.asArray, matL.asArray, 2, 2, matR.asArray, 2, 2);
	return res;
}
//mat3
mat3 operator*(const mat3& m, float s)
{
	mat3 res;
	for (int i = 0; i < 9; ++i)
	{
		res.asArray[i] = m.asArray[i] * s;
	}
	return res;
}
mat3 operator*(const mat3& matL, const mat3& matR)
{
	mat3 res;
	Mul(res.asArray, matL.asArray, 3, 3, matR.asArray, 3, 3);
	return res;
}
//mat4
mat4 operator*(const mat4& m, float s)
{
	mat4 res;
	for (int i = 0; i < 16; ++i)
	{
		res.asArray[i] = m.asArray[i] * s;
	}
	return res;
}
mat4 operator*(const mat4& matL, const mat4& matR)
{
	mat4 res;
	Mul(res.asArray, matL.asArray, 4, 4, matR.asArray, 4, 4);
	return res;
}

//Methods
//general
void Transpose(const float* srcMat, float* dstMat, int srcRow, int srcCol)
{
	for (int i = 0; i < srcRow * srcCol; i++)
	{
		int row = i / srcRow;
		int col = i % srcRow;
		dstMat[i] = srcMat[srcCol * col + row];
	}
}		
bool Mul(float* res, const float* matL, int lRow, int lCol, const float* matR, int rRow, int rCol)
{
	if (lCol != rRow)return false;
	for (int i = 0; i < lRow; ++i)
	{
		for (int j = 0;j < rCol; ++j)
		{
			res[rCol * i + j] = 0.0f;
			for (int k = 0; k < rRow; ++k)
			{
				int l = lCol * i + k;
				int r = rCol * k + j;
				res[rCol * i + j] += matL[l] * matR[r];
			}
		}
	}
	return true;
}
void Cofactor(float* out, const float* min, int row, int col)
{
	for (int i = 0; i < row; ++i)
	{
		for (int j = 0; j < col; ++j)
		{
			int trg = col * j + i;
			int src = col * j + i;
			float sign = powf(-1.0f, i + j);
			out[trg] = min[src] * sign;
		}
	}
}

//mat2
mat2 Transpose(const mat2& m)
{
	mat2 res;
	Transpose(m.asArray, res.asArray, 2, 2);
	return res;
}
float Determinant(const mat2& m)
{
	return m._11 * m._22 - m._12 * m._21;
}
mat2 Minor(const mat2& mat)
{
	return mat2(mat._22, mat._21, mat._12, mat._11);
}
mat2 Cut(const mat3& mat, int row, int col)
{
	mat2 res;
	int idx = 0;
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			if (i == row || j == col)continue;
			int trg = idx++;
			int src = 3 * i + j;
			res.asArray[trg] = mat.asArray[src];
		}
	}
	return res;
}
mat2 Cofactor(const mat2& mat)
{
	mat2 res;
	Cofactor(res.asArray, Minor(mat).asArray, 2, 2);
	return res;
}
mat2 Adjugate(const mat2& mat)
{
	return Transpose(Cofactor(mat));
}
mat2 Inv(const mat2& mat)
{
	float det = Determinant(mat);
	if (CMP(det, 0.0f))return mat2();
	return Adjugate(mat) * (1.0f / det);
}

//mat3
mat3 Transpose(const mat3& m)
{
	mat3 res;
	Transpose(m.asArray, res.asArray, 3, 3);
	return res;
}
mat3 Minor(const mat3& mat)
{
	mat3 res;
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			res[i][j] == Determinant(Cut(mat, i, j));
		}
	}
	return res;
}
mat3 Cofactor(const mat3& mat)
{
	mat3 res;
	Cofactor(res.asArray, Minor(mat).asArray, 3, 3);
	return res;
}
mat3 Cut(const mat4& mat, int row, int col)
{
	mat3 res;
	int idx = 0;
	for (int i = 0; i < 4; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			if (i == row || j == col)continue;
			int trg = idx++;
			int src = 4 * i + j;
			res.asArray[trg] = mat.asArray[src];
		}
	}
	return res;
}
float Determinant(const mat3& m)
{
	float res = 0.0f;
	mat3 cof = Cofactor(m);
	for (int i = 0; i < 3; ++i)
	{
		int idx = 3 * 0 + i;
		res += m.asArray[idx] * cof[0][i];
	}
	return res;
}
mat3 Adjugate(const mat3& mat)
{
	return Transpose(Cofactor(mat));
}
mat3 Inv(const mat3& mat)
{
	float det = Determinant(mat);
	if (CMP(det, 0.0f))return mat3();
	return Adjugate(mat) * (1.0f / det);
}

//mat4
mat4 Transpose(const mat4& m)
{
	mat4 res;
	Transpose(m.asArray, res.asArray, 4, 4);
	return res;
}
mat4 Minor(const mat4& mat)
{
	mat4 res;
	for (int i = 0; i < 4; ++i)
	{
		for (int j = 0; j < 4; j++)
		{
			res[i][j] = Determinant(Cut(mat, i, j));
		}
	}
	return res;
}
mat4 Cofactor(const mat4& mat)
{
	mat4 res;
	Cofactor(res.asArray, Minor(mat).asArray, 4, 4);
	return res;
}
float Determinant(const mat4& mat)
{
	float res = 0.0f;
	mat4 cof = Cofactor(mat);
	for (int i = 0; i < 4; ++i)
	{
		res += mat.asArray[4 * 0 + i] * cof[0][i];
	}
	return res;
}
mat4 Adjugate(const mat4& mat)
{
	return Transpose(Cofactor(mat));
}
mat4 Inv(const mat4& mat)
{
	float det = Determinant(mat);
	if (CMP(det, 0.0f))return mat4();
	return Adjugate(mat) * (1.0f / det);
}

//matrix transformations
//translation
mat4 Translation(float x, float y, float z)
{
	return mat4(
		1.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 1.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 1.0f, 0.0f,
		x, y, z, 1.0f
	);
}
mat4 Translation(const vec3& pos)
{
	return mat4(
		1.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 1.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 1.0f, 0.0f,
		pos.x, pos.y, pos.z, 1.0f
	);
}
vec3 GetTranslation(const mat4& mat)
{
	return vec3(mat._41,mat._42,mat._43);
}
//scale
mat4 Scale(float x, float y, float z)
{
	return mat4(
		x, 0.0f, 0.0f, 0.0f,
		0.0f, y, 0.0f, 0.0f,
		0.0f, 0.0f, z, 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f
	);
}
mat4 Scale(const vec3& vec)
{
	return mat4(
		vec.x, 0.0f, 0.0f, 0.0f,
		0.0f, vec.y, 0.0f, 0.0f,
		0.0f, 0.0f, vec.z, 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f
	);
}
vec3 GetScale(const mat4& mat)
{
	return vec3(mat._11,mat._22,mat._33);
}
//rotate
mat4 Rot(float pitch, float yaw, float roll)
{
	return ZRot(roll) * XRot(pitch) * YRot(yaw);
}
mat3 Rot3x3(float pitch, float yaw, float roll)
{
	return ZRot3x3(roll) * XRot3x3(pitch) * YRot3x3(yaw);
}
mat4 ZRot(float angle)
{
	angle = DEG2RAD(angle);
	return mat4(
		cosf(angle), sinf(angle), 0.0f, 0.0f,
		-sinf(angle), cosf(angle), 0.0f, 0.0f,
		0.0f, 0.0f, 1.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f
	);
}
mat3 ZRot3x3(float angle)
{
	angle = DEG2RAD(angle);
	return mat3(
		cosf(angle), sinf(angle), 0.0f,
		-sinf(angle), cosf(angle), 0.0f,
		0.0f, 0.0f, 1.0f
	);
}
mat4 XRot(float angle)
{
	angle = DEG2RAD(angle);
	return mat4(
		1.0f, 0.0f, 0.0f, 0.0f,
		0.0f, cosf(angle), sinf(angle), 0.0f,
		0.0f, -sinf(angle), cosf(angle), 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f
	);
}
mat3 XRot3x3(float angle)
{
	angle = DEG2RAD(angle);
	return mat3(
		1.0f, 0.0f, 0.0f,
		0.0f, cosf(angle), sinf(angle),
		0.0f, -sinf(angle), cosf(angle)
	);
}
mat4 YRot(float angle)
{
	angle = DEG2RAD(angle);
	return mat4(
		cosf(angle), 0.0f, -sinf(angle), 0.0f,
		0.0f, 1.0f, 0.0f, 0.0f,
		sinf(angle), 0.0f, cosf(angle), 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f
	);
}
mat3 YRot3x3(float angle)
{
	angle = DEG2RAD(angle);
	return mat3(
		cosf(angle), 0.0f, -sinf(angle),
		0.0f, 1.0f, 0.0f,
		sinf(angle), 0.0f, cosf(angle)
	);
}
// rotate around given axis
mat4 AxisAngle(const vec3& axis, float angle)
{
	angle = DEG2RAD(angle);
	float c = cosf(angle);
	float s = sinf(angle);
	float t = 1.0f - cosf(angle);

	float x = axis.x;
	float y = axis.y;
	float z = axis.z;
	if (!CMP(MagSq(axis), 1.0f))
	{
		float inv_len = 1.0f / Mag(axis);
		x *= inv_len;
		y *= inv_len;
		z *= inv_len;
	}
	return mat4(
		t * (x * x) + c, t * x * y + s * z, t * x * z - s * y, 0.0f,
		t * x * y + s * z, t * (y * y) + c, t * y * z + s * x, 0.0f,
		t * x * z + s * y, t * y * z - s * x, t * (z * z) + c, 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f
	);
}
mat3 AxisAngle3x3(const vec3& axis, float angle)
{
	angle = DEG2RAD(angle);
	float c = cosf(angle);
	float s = sinf(angle);
	float t = 1.0f - cosf(angle);

	float x = axis.x;
	float y = axis.y;
	float z = axis.z;
	if (!CMP(MagSq(axis), 1.0f))
	{
		float inv_len = 1.0f / Mag(axis);
		x *= inv_len;
		y *= inv_len;
		z *= inv_len;
	}
	return mat3(
		t * (x * x) + c, t * x * y + s * z, t * x * z - s * y,
		t * x * y + s * z, t * (y * y) + c, t * y * z + s * x,
		t * x * z + s * y, t * y * z - s * x, t * (z * z) + c
	);
}
//multiply vec and mat
vec3 MultiplyPoint(const vec3& vec, const mat4& mat)
{
	vec3 res;
	res.x = vec.x * mat._11 + vec.y * mat._21 + vec.z * mat._31 + 1.0f * mat._41;
	res.y = vec.x * mat._12 + vec.y * mat._22 + vec.z * mat._32 + 1.0f * mat._42;
	res.z = vec.x * mat._13 + vec.y * mat._23 + vec.z * mat._33 + 1.0f * mat._43;
	return res;
}
vec3 MultiplyVector(const vec3& vec, const mat4& mat)
{
	vec3 res;
	res.x = vec.x * mat._11 + vec.y * mat._21 + vec.z * mat._31 + 0.0f * mat._41;
	res.y = vec.x * mat._12 + vec.y * mat._22 + vec.z * mat._32 + 0.0f * mat._42;
	res.z = vec.x * mat._13 + vec.y * mat._23 + vec.z * mat._33 + 0.0f * mat._43;
	return res;
}
vec3 MultiplyVector(const vec3& vec, const mat3& mat)
{
	vec3 res;
	res.x = Dot(vec, vec3(mat._11, mat._21, mat._31));
	res.y = Dot(vec, vec3(mat._12, mat._22, mat._32));
	res.z = Dot(vec, vec3(mat._13, mat._23, mat._33));
	return res;
}
//transform
mat4 Transform(const vec3& scale, const vec3& eulerRot, const vec3& translate)
{
	return Scale(scale) * Rot(eulerRot.x, eulerRot.y, eulerRot.z) * Translation(translate);
}
mat4 Transform(const vec3& scale, const vec3& rotAxis, float rotAngle, const vec3& translate)
{
	return Scale(scale) * AxisAngle(rotAxis, rotAngle) * Translation(translate);
}
//view
mat4 LookAt(const vec3& pos, const vec3& trg, const vec3& up)
{
	vec3 forward = Nomd(trg - pos);
	vec3 right = Nomd(Cross(up, forward));
	vec3 newUp = Cross(forward, right);

	return mat4(
		right.x, newUp.x, forward.x, 0.0f,
		right.y, newUp.y, forward.y, 0.0f,
		right.z, newUp.z, forward.z, 0.0f,
		-Dot(right, pos), -Dot(newUp, pos),
		-Dot(forward, pos), 1.0f
	);
}

mat4 Projection(float fov, float aspect, float zNear, float zFar)
{
	float tanHalfFov = tanf(DEG2RAD((fov * 0.5f)));
	float fovY = 1.0f / tanHalfFov;
	float fovX = fovY / aspect;
	mat4 res;
	res._11 = fovX;
	res._22 = fovY;
	res._33 = zFar / (zFar - zNear);
	res._34 = 1.0f;
	res._43 = -zNear * res._33;
	res._44 = 0.0f;
	return res;
}

mat4 Ortho(float left, float right, float bottom, float top, float zNear, float zFar)
{
	float _11 = 2.0f / (right - left);
	float _22 = 2.0f / (top - bottom);
	float _33 = 1.0f / (zFar - zNear);
	float _41 = (left + right) / (left - right);
	float _42 = (top + bottom) / (bottom - top);
	float _43 = (zNear) / (zNear - zFar);

	return mat4(
		_11, 0.0f, 0.0f, 0.0f,
		0.0f, _22, 0.0f, 0.0f,
		0.0f, 0.0f, _33, 0.0f,
		_41, _42, _43, 1.0f
	);
}

