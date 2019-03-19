#pragma once

#include "Vector.h"

class Matrix3x3
{
public:
	Matrix3x3();
	Vector StandardRotation(Vector point, Vector axis, float angle);
private:
	float values_[3][3];	//	A 3x3 multidimensional array of floats (values[i][j] : where i is row and j is column)
};