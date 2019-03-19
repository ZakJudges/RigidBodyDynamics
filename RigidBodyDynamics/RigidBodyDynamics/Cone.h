#pragma once

#include "RigidBody.h"

class Cone : public RigidBody
{
public: 

	Cone(float radius, float height, float mass, Vector angular_velcity, Vector linear_velocity);

	void Step();

	//	Implementation of the virtual functions.
	//		For motion specific to this type of rigid body motion.
	virtual float RK4(float gamma, float x, float y, float prev_value);
	virtual void SemiImplicitEuler();

	//	Getters.
	inline Vector GetAngularVelocity() { return w_; }
	inline Vector GetLinearVelocity() { return v_; }
	inline Vector GetDisplacement() { return r_; }
	inline Vector GetPoint() { return point_; }

private:
	float g1_;
	float g2_;
	float g3_;
	Vector point_;
	const float STEP_SIZE = 0.016666666f;
	const float GRAVITY = 9.8f;
};