#pragma once

#include "Vector.h"

//	3x3 Matrix class that provides functionality to rotate a point by this matrix. 
class Matrix3x3
{
public:
	Matrix3x3();
	Vector StandardRotation(Vector point, Vector axis, float angle);
private:
	float values_[3][3];	//	A 3x3 multidimensional array of floats (values[y][x] : where y is row and x is column)
};