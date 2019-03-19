#include "Cone.h"
#include "Matrix3x3.h"
#include <math.h>
#include <iostream>
Cone::Cone(float radius, float height, float mass, Vector angular_velocity, Vector linear_velocity) : RigidBody(mass, radius, height, angular_velocity, linear_velocity)
{
	float m_factor = mass * (3.0f / 20.0f);

	//	Define the principal moments of intertia for this cone.
	float i1 = ((radius * radius) + (0.25f * height * height)) * m_factor;
	float i2 = i1;
	float i3 = 2.0f * radius * radius * m_factor;

	//	Define the values for gamma specific to this cone.
	g1_ = (i3 - i2) / i1;
	g2_ = (i1 - i3) / i2;
	g3_ = (i2 - i1) / i3;

	point_.x = 0.0f;
	point_.y = 0.75f * radius;
	point_.z = 0.0f;
}

void Cone::Step()
{
	//	Calculate the new angular velocity:
	w_.x = RK4(g1_, w_.y, w_.z, w_.x);
	w_.y = RK4(g2_, w_.x, w_.z, w_.y);
	w_.z = RK4(g3_, w_.x, w_.y, w_.z);

	//	Calculate the new velocity and displacement.
	SemiImplicitEuler();

	//	Calculate the additional velocity component from the rotation of the cone.
	Matrix3x3 rotation_matrix;
	float length_squared = (w_.x * w_.x) + (w_.y * w_.y) + (w_.z * w_.z);
	float length = sqrt(length_squared);
	float angle = length * STEP_SIZE;
	Vector axis;
	axis.x = w_.x / length;
	axis.y = w_.y / length;
	axis.z = w_.z / length;
	Vector rotation_component = rotation_matrix.StandardRotation(point_, axis, angle);
	
	//	Calculate the new position of the point on the cone, given the linear and angular velocities of the cone at its centre of mass:
	point_.x = r_.x + v_.x + rotation_component.x;
	point_.y = r_.y + v_.y + rotation_component.y;
	point_.z = r_.z + v_.z + rotation_component.z;

	std::cout << rotation_component.x << " " << rotation_component.y << " " << rotation_component.z << std::endl;
}

//	4th Order Runge-Kutta Algorithm. 
float Cone::RK4(float gamma, float x, float y, float prev_value)
{
	float k1, k2, k3, k4, new_x, new_y;
	float half_step = STEP_SIZE / 2.0f;
	float n_gamma = gamma * -1.0f;

	//	Evaluate at original location.
	k1 = n_gamma * x * y;

	new_x = x + half_step;
	new_y = y + (half_step * k1);

	k2 = n_gamma * new_x * new_y;

	new_y = y + (half_step * k2);

	k3 = n_gamma * new_x * new_y;

	new_x = x + STEP_SIZE;
	new_y = y + (STEP_SIZE * k3);

	k4 = n_gamma * new_x * new_y;

	//	Return approximation of the change in angular velocity.
	return prev_value + ((STEP_SIZE / 6.0f) * (k1 + (2 * k2) + (2 * k3) + k4));
}

//	Calculate the cone's linear velocity and displacement at the centre of mass this timestep.
void Cone::SemiImplicitEuler()
{
	Vector a;
	a.x = 0.0f;
	a.y = 0.0f;
	a.z = GRAVITY;

	v_.x = v_.x - (a.x * STEP_SIZE);
	v_.y = v_.y - (a.y * STEP_SIZE);
	v_.z = v_.z - (a.z * STEP_SIZE);

	r_.x = r_.x + (v_.x * STEP_SIZE);
	r_.y = r_.y + (v_.y * STEP_SIZE);
	r_.z = r_.z + (v_.z * STEP_SIZE);
}
