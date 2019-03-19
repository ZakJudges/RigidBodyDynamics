#include "Cone.h"


#include <iostream>
#include <fstream>

int main()
{
	//	Define frame time (60 updates per second).
	const float step_size = 0.016666666f;

	//	Define initial conditions for the cone:
	float radius = 1.0f;
	float height = 4.0f;
	float mass = 10.0f;
	Vector angular_velocity;
	angular_velocity.x = 3.0f;
	angular_velocity.y = 1.0f;
	angular_velocity.z = 2.0f;
	Vector linear_velocity;
	linear_velocity.x = 0.0f;
	linear_velocity.y = 0.0f;
	linear_velocity.z = 200.0f;
	
	//	Create the cone.
	Cone cone(radius, height, mass, angular_velocity, linear_velocity);

	//	Create the files to output the rigid body data to.
	std::ofstream file_w("w.txt");
	std::ofstream file_v("v.txt");
	std::ofstream file_r("r.txt");
	std::ofstream file_p("p.txt");

	float timer = 0.0f;
	Vector w;
	Vector v;
	Vector r;
	Vector p;

	//	Simulate 20 seconds of motion for the cone.
	while (timer < 20.0f)
	{
		//	Update velocities and displacement of cone.
		cone.Step();

		//	Get the velocities and displacement of the cone.
		w = cone.GetAngularVelocity();
		v = cone.GetLinearVelocity();
		r = cone.GetDisplacement();
		p = cone.GetPoint();

		//	Write out the data from the motion of the cone to files.
		if (file_w.is_open())
		{
			file_w << timer << " " << w.x << " " << w.y << " " << w.z << std::endl;
		}
		if (file_v.is_open())
		{
			file_v << timer << " " << v.x << " " << v.y << " " << v.z << std::endl;
		}
		if (file_r.is_open())
		{
			file_r << timer << " " << r.x << " " << r.y << " " << r.z << std::endl;
		}
		if (file_p.is_open())
		{
			file_p << timer << " " << p.x << " " << p.y << " " << p.z << std::endl;
		}


		//	Time step has finished, so increment the counter.
		timer += step_size;
	}
	getchar();
	return 0;
}