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

float Determinant(const mat3& m)
{
	float res = 0.0f;
	mat3 cof = Cofactor(m);
	for (int i = 0; i < 3; ++i)
	{
		int idx = 3 * 0 + i;
		res += m.asArray[idx] * cof[0][j];
	}
	return res;
}

//mat4
mat4 Transpose(const mat4& m)
{
	mat4 res;
	Transpose(m.asArray, res.asArray, 4, 4);
	return res;
}
