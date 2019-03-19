#pragma once

#include "Vector.h"

//	Simple class that stores rigid body information.
class RigidBody
{
public:
	RigidBody(float mass, float radius, float height, Vector angular_velocity, Vector linear_velocity);
protected:
	//	Pure virtual functions that derived classes must provide implementation for.
	virtual float RK4(float gamma, float x, float y, float prev_value) = 0;
	virtual void SemiImplicitEuler() = 0;
protected:
	Vector w_;
	Vector v_;
	Vector r_;
	float mass_;
	float radius_;
	float height_;
};