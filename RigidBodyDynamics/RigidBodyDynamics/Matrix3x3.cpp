#include "Matrix3x3.h"
#include<math.h>

Matrix3x3::Matrix3x3()
{
	//	Initialise the matrix to the identity matrix.
	values_[0][0] = 1.0f; values_[0][1] = 0.0f; values_[0][2] = 0.0f;
	values_[1][0] = 0.0f; values_[1][1] = 1.0f; values_[1][2] = 0.0f;
	values_[2][0] = 0.0f; values_[2][1] = 0.0f; values_[2][2] = 1.0f;
}

//	Return a point rotated by an angle about an axis passing through the origin
Vector Matrix3x3::StandardRotation(Vector p, Vector axis, float angle)
{
	//	Define constants.
	const float alpha = axis.x;
	const float beta = axis.y;
	const float gamma = axis.x;
	const float cos_theta = cos(angle);
	const float sin_theta = sin(angle);

	//	Lamba rotation matrix;
	//		First Row.
	values_[0][0] = (alpha * alpha * (1.0f - cos_theta)) + cos_theta;
	values_[0][1] = (alpha * beta * (1.0f - cos_theta)) - (gamma * sin_theta);
	values_[0][2] = (alpha * gamma * (1.0f - cos_theta)) + (beta * sin_theta);
	//		Second Row.
	values_[1][0] = (alpha * beta * (1.0f - cos_theta)) + (gamma * sin_theta);
	values_[1][1] = (beta * beta * (1.0f - cos_theta)) + cos_theta; 
	values_[1][2] = (beta * gamma * (1.0f - cos_theta)) - (alpha * sin_theta);
	//		Third Row.
	values_[2][0] = (alpha * gamma * (1.0f - cos_theta)) - (beta * sin_theta); 
	values_[2][1] = (beta * gamma * (1.0f - cos_theta)) + (alpha * sin_theta); 
	values_[2][2] = (gamma * gamma * (1.0f - cos_theta)) + cos_theta;

	//	Now multiply the position vector by the matrix to get the position of the transformed point.
	Vector new_p;
	new_p.x = (values_[0][0] * p.x) + (values_[0][1] * p.y) + (values_[0][2] * p.z);
	new_p.y = (values_[1][0] * p.x) + (values_[1][1] * p.y) + (values_[1][2] * p.z);
	new_p.z = (values_[2][0] * p.x) + (values_[2][1] * p.y) + (values_[2][2] * p.z);

	return new_p;
}
