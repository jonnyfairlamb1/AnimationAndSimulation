#include "MyUtils.hpp"
#include <tyga/BasicWorldClock.hpp>
#include <tyga/Math.hpp>




namespace MyUtils {
	const float time = tyga::BasicWorldClock::CurrentTime();



	tyga::Matrix4x4 translate(float x, float y, float z) {
		return tyga::Matrix4x4(1,0,0,0,
							   0,1,0,0,
							   0,0,1,0,
							   x,y,z,0);
	}

	tyga::Matrix4x4 rotateX(float radians) {
		return tyga::Matrix4x4(1, 0, 0, 0,
			0, cos(radians), -sin(radians), 0,
			0, sin(radians), cos(radians), 0,
			0, 0, 0, 1);
	}

	tyga::Matrix4x4 rotateY(float radians) {
		return tyga::Matrix4x4(cos(radians), 0, sin(radians), 0,
			0, 1, 0, 0,
			-sin(radians), 0, cos(radians), 0,
			0, 0, 0, 1);
	}

	tyga::Matrix4x4 rotateZ(float radians) {
		return tyga::Matrix4x4(cos(radians), -sin(radians), 0, 0,
							sin(radians), cos(radians), 0, 0,
									0, 0, 1, 0,
									0, 0, 0, 1);					
	}


	tyga::Matrix4x4 scale(float x, float y, float z) {
		return tyga::Matrix4x4(x, 0, 0, 0,
			0, x, 0, 0,
			0, 0, y, 0,
			0, 0, 0, z);
	}


	float continuous(float startValue, float speed, float time) {
		float angle = startValue + speed * time;

		return angle;
	}

	float linearStep(float min, float max, float x)
	{
		return clamp(0, 1, (x - min) / (max - min));
	}

	float smootherStep(float min, float max, float x)
	{
		x = linearStep(min, max, x);
		return 3 * x * x - 2 * x * x * x;
	}

	float clamp(float time, float startTime, float stopTime)
	{
		if (time < startTime) {
			return startTime;
		}
		if (time > stopTime) {
			return stopTime;
		}

		return time;
	}

	tyga::Matrix4x4 positioning_euler(float oriantationX, float oriantationY, float oriantationZ, float transformX, float transformY, float transformZ){

		tyga::Vector3 oriantation = { oriantationX * (float)M_PI / 180, oriantationY * (float)M_PI / 180, oriantationZ * (float)M_PI / 180 };
		tyga::Vector3 position = { transformX, transformY, transformZ };


		auto rotation = MyUtils::rotateX(oriantationX) * MyUtils::rotateY(oriantationY) * MyUtils::rotateZ(oriantationZ);
		auto translation = MyUtils::translate(position.x, position.y, position.z);

		auto combined_xform = translation * rotation ;

		return combined_xform;
	}

	float getAcceleration(float mass, tyga::Vector3 force){
		
		float acceleration = force.x / mass + force.y / mass + force.z / mass;

		return acceleration;
	}

	float getVelocity(float mass, tyga::Vector3 force){
		float velocity = velocity + getAcceleration(mass, force) * time;
		return velocity;
	}

	float getDistance(float mass, tyga::Vector3 force) {
		float distance = getVelocity(mass, force) * time;
		return distance;
	}

	float calcDrag(float sphere_dc, float area, tyga::Vector3 velocity, float rho)
	{
		return sphere_dc * 0.5 * rho* tyga::dot(velocity, velocity) * area;
	}

	float AreaofSphere(float radius)
	{
		return  M_PI*(radius* radius);
	}

	tyga::Vector3 GetRotation(tyga::Matrix4x4 mat){
		tyga::Vector3 euler;

		if (mat._00 == 1.0f ||
			mat._00 == -1.0f)
		{
			euler.x = atan2f(mat._02, mat._23);
			euler.y = 0;
			euler.z = 0;
		}
		else
		{
			euler.x = atan2f(-mat._20, mat._00);
			euler.y = asin(mat._10);
			euler.z = atan2(-mat._12, mat._11);
		}

		return euler;
	}

	tyga::Vector3 Getposition(tyga::Matrix4x4 mat){
		return tyga::Vector3(mat._30, mat._31, mat._32);
	}

	tyga::Vector3 GetFoward(tyga::Matrix4x4 mat){
		return tyga::Vector3(mat._20, mat._21, mat._22);
	}

	

}


