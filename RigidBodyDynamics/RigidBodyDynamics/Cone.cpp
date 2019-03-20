#include "Cone.h"
#include "Matrix3x3.h"
#include <math.h>

Cone::Cone(float radius, float height, float mass, Vector angular_velocity, Vector linear_velocity) : RigidBody(mass, angular_velocity, linear_velocity)
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

	//	Define a point on the rigid body.
	original_point_.x = 0.0f;
	original_point_.y = 0.75f * radius;
	original_point_.z = 0.0f;
}

//	Simulate motion for the cone during timestep at time t.
void Cone::Step(float t)
{
	//	Calculate the new angular velocity:
	w_.x = RK4(g1_, w_.y, w_.z, w_.x);
	w_.y = RK4(g2_, w_.x, w_.z, w_.y);
	w_.z = RK4(g3_, w_.x, w_.y, w_.z);

	//	Calculate the new linear velocity and displacement.
	SemiImplicitEuler();

	//	Calculate the new position of a point on the cone.
	GeneralMotion(t);
}

//	4th Order Runge-Kutta Algorithm. 
float Cone::RK4(float gamma, float x, float y, float prev_value)
{
	float k1, k2, k3, k4, new_x, new_y;
	float half_step = STEP_SIZE / 2.0f;
	float n_gamma = gamma * -1.0f;

	//	Evaluate w at original location.
	k1 = n_gamma * x * y;

	//	Approximate w half-way through timestep.
	new_x = x + half_step;
	new_y = y + (half_step * k1);
	k2 = n_gamma * new_x * new_y;
	new_y = y + (half_step * k2);
	k3 = n_gamma * new_x * new_y;

	//	Approximate w at the end of the timestep.
	new_x = x + STEP_SIZE;
	new_y = y + (STEP_SIZE * k3);
	k4 = n_gamma * new_x * new_y;

	//	Calculate the velocity change this timestep based on a weighted average of w. 
	float velocity_change = (STEP_SIZE / 6.0f) * (k1 + (2 * k2) + (2 * k3) + k4);

	//	Return the new angular velocity based on the approximation of the change in angular velocity.
	return prev_value + velocity_change;
}

//	Calculate the cone's linear velocity and displacement at the centre of mass this timestep.
void Cone::SemiImplicitEuler()
{
	//	Acceleration of the cone due to gravity.
	Vector a;
	a.x = 0.0f;
	a.y = 0.0f;
	a.z = GRAVITY;

	//	New linear velocity of the cone's centre of mass based on how much the cone has accelerated by this timestep.
	v_.x = v_.x - (a.x * STEP_SIZE);
	v_.y = v_.y - (a.y * STEP_SIZE);
	v_.z = v_.z - (a.z * STEP_SIZE);

	//	New position of the cone's centre of mass based on how much the linear veloctiy has increased this timestep.
	r_.x = r_.x + (v_.x * STEP_SIZE);
	r_.y = r_.y + (v_.y * STEP_SIZE);
	r_.z = r_.z + (v_.z * STEP_SIZE);
}

//	Find the position of a point on the cone.
//		Velocity and acceleration of the point on the cone would also be calculated here if required.
void Cone::GeneralMotion(float t)
{
	//	Find magnitude of the angular velocity vector at this timestep.
	float length_squared = (w_.x * w_.x) + (w_.y * w_.y) + (w_.z * w_.z);
	float length = sqrt(length_squared);
	//	Find the angle that the angular velocity vector has rotated by.
	float angle = length * t;
	//	Find the axis of rotation that the angular velocity rotates about (normalised).
	Vector axis;
	axis.x = w_.x / length;
	axis.y = w_.y / length;
	axis.z = w_.z / length;

	//	Calculate the velocity component from the rotation of the cone:
	Matrix3x3 rotation_matrix;
	Vector rotation_component = rotation_matrix.StandardRotation(original_point_, axis, angle);

	//	Calculate the new position of the point on the cone, given the linear and angular velocities of the cone at its centre of mass:
	new_point_.x = r_.x + v_.x + rotation_component.x;
	new_point_.y = r_.y + v_.y + rotation_component.y;
	new_point_.z = r_.z + v_.z + rotation_component.z;
}
