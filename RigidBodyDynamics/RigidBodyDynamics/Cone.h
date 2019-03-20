#pragma once

#include "RigidBody.h"

class Cone : public RigidBody
{
public: 

	Cone(float radius, float height, float mass, Vector angular_velcity, Vector linear_velocity);

	void Step(float t);

	//	Implementation of the virtual functions.
	//		For motion specific to this type of rigid body.
	virtual float RK4(float gamma, float x, float y, float prev_value);
	virtual void SemiImplicitEuler();
	virtual void GeneralMotion(float time);

	//	Getters.
	inline Vector GetAngularVelocity() { return w_; }
	inline Vector GetLinearVelocity() { return v_; }
	inline Vector GetDisplacement() { return r_; }
	inline Vector GetPoint() { return new_point_; }

private:
	float radius_;
	float height_;
	float g1_;
	float g2_;
	float g3_;
	Vector original_point_;
	Vector new_point_;
};