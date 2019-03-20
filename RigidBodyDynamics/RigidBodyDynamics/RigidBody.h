#pragma once

#include "Vector.h"

//	Simple class that stores rigid body information.
//	Any type of rigid body should be derived from this class.
//		Implementation of calculations for general motion are specific to derived classes - e.g. inertia tensor.
class RigidBody
{
public:
	RigidBody(float mass, Vector angular_velocity, Vector linear_velocity);
protected:
	//	Pure virtual functions for motion that derived classes must provide implementation for.
	virtual float RK4(float gamma, float x, float y, float prev_value) = 0;
	virtual void SemiImplicitEuler() = 0;
	virtual void GeneralMotion(float t) = 0;
protected:
	//	Angular velocity.
	Vector w_;
	//	Linear velocity.
	Vector v_;
	//	Displacement.
	Vector r_;
	float mass_;

	//	Step size constant as not changing based on actual frame rate since the simulation does not take place in real time.
	const float STEP_SIZE = 0.016666666f;
	const float GRAVITY = 9.8f;
};