#include "RigidBody.h"

//	Simple class that stores rigid body information.
//	Any type of rigid body should be derived from this class.
//		Implementation of calculations for general motion are specific to derived classes.
RigidBody::RigidBody(float mass, float radius, float height, Vector angular_velocity, Vector linear_velocity)
{
	mass_ = mass;
	radius_ = radius;
	height_ = height;
	w_ = angular_velocity;
	v_ = linear_velocity;

	r_.x = 0.0f;
	r_.y = 0.0f;
	r_.z = 0.0f;
}
