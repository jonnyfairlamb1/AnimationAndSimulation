#include "MyPhysicsCentre.hpp"
#include <tyga/BasicWorldClock.hpp>
#include <tyga/Log.hpp>
#include "MyUtils.hpp"

using namespace MyUtils;

PhysicsObject::PhysicsObject() : mass(0)
{
}

tyga::Vector3 PhysicsObject::
position() const
{
	auto actor = this->Actor();
	if (actor != nullptr) {
		const auto& xform = actor->Transformation();
		return tyga::Vector3(xform._30, xform._31, xform._32);
	}
	return tyga::Vector3();
}

PhysicsPlane::PhysicsPlane()
{
}

tyga::Vector3 PhysicsPlane::
normal() const
{
	auto actor = this->Actor();
	if (actor != nullptr) {
		const auto& xform = actor->Transformation();
		return tyga::Vector3(xform._20, xform._21, xform._22);
	}
	return tyga::Vector3();
}

PhysicsSphere::PhysicsSphere() : radius(1)
{
}

PhysicsBox::PhysicsBox()
{
}

tyga::Vector3 PhysicsBox::
U() const
{
	auto actor = this->Actor();
	if (actor != nullptr) {
		const auto& xform = actor->Transformation();
		return tyga::Vector3(xform._00, xform._01, xform._02);
	}
	return tyga::Vector3();
}

tyga::Vector3 PhysicsBox::
V() const
{
	auto actor = this->Actor();
	if (actor != nullptr) {
		const auto& xform = actor->Transformation();
		return tyga::Vector3(xform._10, xform._11, xform._12);
	}
	return tyga::Vector3();
}

tyga::Vector3 PhysicsBox::
W() const
{
	auto actor = this->Actor();
	if (actor != nullptr) {
		const auto& xform = actor->Transformation();
		return tyga::Vector3(xform._20, xform._21, xform._22);
	}
	return tyga::Vector3();
}

std::shared_ptr<MyPhysicsCentre> MyPhysicsCentre::default_centre_;

std::shared_ptr<MyPhysicsCentre> MyPhysicsCentre::
defaultCentre()
{
	if (default_centre_ == nullptr) {
		default_centre_ = std::make_shared<MyPhysicsCentre>();
	}
	return default_centre_;
}

MyPhysicsCentre::
MyPhysicsCentre()
{
}

MyPhysicsCentre::
~MyPhysicsCentre()
{
}

template<typename T> static void
_prune(std::list<T>& list)
{
	for (auto it = list.begin(); it != list.end(); ) {
		if (it->expired()) {
			it = list.erase(it);
		}
		else {
			it++;
		}
	}
}

void MyPhysicsCentre::
pruneDeadObjects()
{
	_prune(planes_);
	_prune(spheres_);
	_prune(boxes_);
}

std::shared_ptr<PhysicsPlane> MyPhysicsCentre::
newPlane()
{
	auto new_plane = std::make_shared<PhysicsPlane>();
	planes_.push_back(new_plane);
	return new_plane;
}

std::shared_ptr<PhysicsSphere> MyPhysicsCentre::
newSphere()
{
	auto new_sphere = std::make_shared<PhysicsSphere>();
	spheres_.push_back(new_sphere);
	return new_sphere;
}

std::shared_ptr<PhysicsBox> MyPhysicsCentre::
newBox()
{
	auto new_box = std::make_shared<PhysicsBox>();
	boxes_.push_back(new_box);
	return new_box;
}

void MyPhysicsCentre::runloopWillBegin()
{
	const auto delta_time = tyga::BasicWorldClock::CurrentTickInterval();

	for (auto s_iter = spheres_.begin(); s_iter != spheres_.end(); s_iter++)
	{
		// only continue if a strong reference is available
		if (s_iter->expired()) continue;
		auto sphere = s_iter->lock();

		for (auto box_iter = boxes_.begin(); box_iter != boxes_.end(); box_iter++)
		{
			if (box_iter->expired()) continue;
			auto box = box_iter->lock();
			auto box_pos = box->position();

			tyga::Vector3 normal = tyga::unit(box_pos - sphere->position());

			float radius = abs(tyga::dot(box->U(), normal)) +
				abs(tyga::dot(box->V(), normal)) +
				abs(tyga::dot(box->W(), normal));

			if (tyga::dot(box_pos, normal) - tyga::dot(sphere->position(), normal) <= radius)
			{
				tyga::Vector3 speed = (box_pos - previous_pos) / delta_time;
				float forceScalar = tyga::length(speed);
				const tyga::Vector3 force = tyga::unit(speed) * forceScalar;
				sphere->velocity += force;
			}
		}


		const auto& p = sphere->position();
		if (p.y < sphere->radius) {
			sphere->velocity.y = 0;
			auto actor = sphere->Actor();
			if (actor != nullptr) {
				auto xform = actor->Transformation();
				xform._31 = sphere->radius;
				actor->setTransformation(xform);
			}
		}
	}

	for (auto box_iter = boxes_.begin(); box_iter != boxes_.end(); box_iter++)
	{
		if (box_iter->expired()) continue;
		auto box = box_iter->lock();

		previous_pos = box->position();
	}
}

void MyPhysicsCentre::runloopExecuteTask()
{
	const float time = tyga::BasicWorldClock::CurrentTime();
	const float delta_time = tyga::BasicWorldClock::CurrentTickInterval();

	for (auto ptr : spheres_) {
		// only continue if a strong reference is available
		if (ptr.expired()) continue;
		auto model = ptr.lock();

		// only continue if the model is attached to an actor
		auto actor = model->Actor();
		if (actor == nullptr) continue;

		//Calculate the drag applied to the object
		float drag = calcDrag(sphere_dc, AreaofSphere(model->radius), model->velocity, 1.225);

		//Calculate new velositys and positions
		tyga::Vector3 force = model->force += drag *-model->velocity;
		tyga::Vector3& position = model->position();
		tyga::Vector3 velosity = model->velocity;

		position += velosity * delta_time;
		velosity += gravity * delta_time; //Apply gravity
		velosity += (force / model->mass) * delta_time; // Combine all the forces

		model->velocity = velosity;
		model->Actor()->setTransformation(translate(position.x, position.y, position.z));

		// TODO: update the actor's transformation

		// reset the force
		model->force = tyga::Vector3(0, 0, 0);
	}
}

void MyPhysicsCentre::
runloopDidEnd()
{
	pruneDeadObjects();
}

//} // end namespace