#pragma once

#include <tyga/Math.hpp>

namespace MyUtils {

	template <class T, class S>
	T lerp(T a, T b, S s) {
		return(a * (1 - s) + b * s);
	}


	tyga::Matrix4x4 translate(float x, float y, float z);
	tyga::Matrix4x4 rotateX(float radians);
	tyga::Matrix4x4 rotateY(float radians);
	tyga::Matrix4x4 rotateZ(float radians);
	tyga::Matrix4x4 scale(float x, float y, float z); // This will scale the actor on the set x, y, z dimensions. if all are the same it will scale in uniform.
	float continuous(float startValue, float speed, float time);
	float linearStep(float min, float max, float x);
	float smootherStep(float min, float max, float x);
	float clamp(float time, float startTime, float stopTime);
	tyga::Matrix4x4 positioning_euler(float oriantationX, float oriantationY, float oriantationZ, float transformX, float transformY, float transformZ);
	float getAcceleration(float mass, tyga::Vector3 force);
	float getVelocity(float mass, tyga::Vector3 force);
	float getDistance(float mass, tyga::Vector3 force);
	
	float calcDrag(float drag_ceo, float area, tyga::Vector3 velocity, float rho);
	float AreaofSphere(float radius);

	tyga::Vector3 GetRotation(tyga::Matrix4x4 mat);
	tyga::Vector3 Getposition(tyga::Matrix4x4 mat);
	tyga::Vector3 GetFoward(tyga::Matrix4x4 mat);
}
