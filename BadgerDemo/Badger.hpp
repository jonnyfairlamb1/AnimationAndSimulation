#pragma once

#include <tyga/ActorDelegate.hpp>
#include <tyga/Actor.hpp>
#include <tyga/BasicWorldClock.hpp>
#include <tyga/Math.hpp>
#include <tyga/ActorWorld.hpp>
#include <tyga/GraphicsCentre.hpp>
#include <vector>
class MyParticleSystem : public tyga::GraphicsSpriteDelegate, 
						public std::enable_shared_from_this<MyParticleSystem>,
						public tyga::ActorDelegate
{
public:

	struct Particle {
		
		bool active = false;
		//size
		float size;
		float initial_size;
		float final_size;

		float alpha;
		float speed = 3.3;
		tyga::Vector3 position;
		tyga::Vector3 velocity;

		float initial_time;

	};

	std::vector<Particle> particles;

	std::string graphicsSpriteTexture() const override;
	int graphicsSpriteVertexCount() const override;
	void graphicsSpriteGenerate(tyga::GraphicsSpriteVertex vertex_array[]) const override;



};

class Badger : public tyga::ActorDelegate 
{

public:
	
	int max_particals = 1000;
	tyga::Vector3 initial_position;
	tyga::Vector3 initial_velocity;
	tyga::Vector3 acceleration;

	int random_scale = 2;
	float min_size = 0.3;
	float max_size = 1.3;

	float emit_delay = 0.04;
	float life_span = 1.6f;
	float next_emission = 0;
		
		void setSource(std::shared_ptr<tyga::Actor>);
		void setSink(std::shared_ptr<tyga::Actor>);
	
		std::shared_ptr<tyga::Actor>
			boundsActor();
	
	private:
		void SpawnBadger(std::shared_ptr<tyga::Actor> actor);
		void SpawnBloke(std::shared_ptr<tyga::Actor> actor);
	
		void PositionBloke(std::shared_ptr<tyga::Actor> actor);
	
		virtual void
			actorDidEnterWorld(std::shared_ptr<tyga::Actor> actor) ;
	
		virtual void
			actorWillLeaveWorld(std::shared_ptr<tyga::Actor> actor) ;
	
		virtual void
			actorClockTick(std::shared_ptr<tyga::Actor> actor) ;
		
	
		void BlokeLeft(std::shared_ptr<tyga::Actor> actor);
		void BlokeRight(std::shared_ptr<tyga::Actor> actor);
	
		void EmittParticles(std::shared_ptr<tyga::Actor> actor);


		std::shared_ptr<MyParticleSystem> particle_system_;
		std::shared_ptr<tyga::GraphicsSprite> graphic_sprite_;
		std::shared_ptr<tyga::Actor> source_actor_;
		std::shared_ptr<tyga::Actor> sink_actor_;
	


		std::shared_ptr<tyga::Actor> physics_actor_;
	


		std::shared_ptr<tyga::Actor> chassis_actor_;
		std::shared_ptr<tyga::Actor> handlebars_actor_;
		std::shared_ptr<tyga::Actor> luggage_actor_;
		std::vector<std::shared_ptr<tyga::Actor>> wheels_actor_;
	
		
	
	
		std::shared_ptr<tyga::Actor> pelvis_actor_;
		std::shared_ptr<tyga::Actor> torso_actor_;
		std::shared_ptr<tyga::Actor> helmet_actor_;
	
		std::shared_ptr<tyga::Actor> arm_left_actor_;
		std::shared_ptr<tyga::Actor> forearm_left_actor_;
		std::shared_ptr<tyga::Actor> hand_left_actor_;
	
		std::shared_ptr<tyga::Actor> arm_right_actor_;
		std::shared_ptr<tyga::Actor> forearm_right_actor_;
		std::shared_ptr<tyga::Actor> hand_right_actor_;
	
		std::shared_ptr<tyga::Actor> leg_left_actor_;
		std::shared_ptr<tyga::Actor> shin_left_actor_;
		std::shared_ptr<tyga::Actor> foot_left_actor_;
	
		std::shared_ptr<tyga::Actor> leg_right_actor_;
		std::shared_ptr<tyga::Actor> shin_right_actor_;
		std::shared_ptr<tyga::Actor> foot_right_actor_;
	
	
		
		
		float currentSpeed{ 0 }, turnRate{ 0 }, wheelAngle{ 0 }, wheelTurn{ 0 }, handleTurn{0};
	
		float distanceTravelled{ 0 }, deltaHeadingAngle{ 0 };

		float randomX{ 0 }, randomY{ 0 }, randomZ{ 0 }, randomDeathDistance{ 0 }, randomAngle{0};

};




