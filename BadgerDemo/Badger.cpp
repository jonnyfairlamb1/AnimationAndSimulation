#include "Badger.hpp"
#include "MyUtils.hpp"
#include "Bloke.hpp"

#include <tyga/Actor.hpp>
#include <tyga/BasicWorldClock.hpp>
#include <tyga/Math.hpp>
#include <tyga/ActorWorld.hpp>
#include <tyga/GraphicsCentre.hpp>
#include <ctime>


void Badger::setSource(std::shared_ptr<tyga::Actor> source){
	source_actor_ = source;
}

void Badger::setSink(std::shared_ptr<tyga::Actor> sink){

	sink_actor_ = sink;
}

std::shared_ptr<tyga::Actor> Badger::boundsActor()
{
	return physics_actor_;
}

void Badger::SpawnBadger(std::shared_ptr<tyga::Actor> actor){
	auto world = tyga::ActorWorld::defaultWorld();
	auto graphics = tyga::GraphicsCentre::defaultCentre();

	physics_actor_ = std::make_shared<tyga::Actor>();
	world->addActor(physics_actor_);

	//Creates the chassis
	auto chassis_material = graphics->newMaterial();
	chassis_material->colour = tyga::Vector3(0.5f, 0.5f, 0.5f);
	auto chassis_mesh = graphics->newMeshWithIdentifier("badger.tcf/chassis");
	auto chassis_model = graphics->newModel();
	chassis_model->material = chassis_material;
	chassis_model->mesh = chassis_mesh;
	chassis_actor_ = std::make_shared<tyga::Actor>();

	chassis_actor_->attachComponent(chassis_model);
	world->addActor(chassis_actor_);

	//end of creating chassis

	//Creates the handlbars
	auto handlebars_material = graphics->newMaterial();
	handlebars_material->colour = tyga::Vector3(0.5f, 0.5f, 0.5f);
	auto handlebars_mesh = graphics->newMeshWithIdentifier("badger.tcf/handlebar");
	auto handlebars_model = graphics->newModel();
	handlebars_model->material = handlebars_material;
	handlebars_model->mesh = handlebars_mesh;
	handlebars_actor_ = std::make_shared<tyga::Actor>();
	handlebars_actor_->attachComponent(handlebars_model);

	world->addActor(handlebars_actor_);

	//end of creating handlebars

	//Creates the LuggageRack
	auto luggage_material = graphics->newMaterial();
	luggage_material->colour = tyga::Vector3(0.5f, 0.5f, 0.5);
	auto luggage_mesh = graphics->newMeshWithIdentifier("badger.tcf/luggage_rack");
	auto luggage_model = graphics->newModel();
	luggage_model->material = luggage_material;
	luggage_model->mesh = luggage_mesh;
	luggage_actor_ = std::make_shared<tyga::Actor>();
	luggage_actor_->attachComponent(luggage_model);

	world->addActor(luggage_actor_);

	//end of creating LuggageRack

	//Creates the Wheels
	wheels_actor_.resize(4);
	for (unsigned int i = 0; i < 4; i++) {
		auto wheels_material = graphics->newMaterial();
		wheels_material->colour = tyga::Vector3(0.5f, 0.5f, 0.5f);
		auto wheels_mesh = graphics->newMeshWithIdentifier("badger.tcf/wheel");
		auto wheels_model = graphics->newModel();
		wheels_model->material = wheels_material;
		wheels_model->mesh = wheels_mesh;
		wheels_actor_[i] = std::make_shared<tyga::Actor>();
		wheels_actor_[i]->attachComponent(wheels_model);

		world->addActor(wheels_actor_[i]);
	}

	//end of creating Wheels
}

void Badger::SpawnBloke(std::shared_ptr<tyga::Actor> actor){
	auto world = tyga::ActorWorld::defaultWorld();
	auto graphics = tyga::GraphicsCentre::defaultCentre();

	auto bloke_material = graphics->newMaterial();
	bloke_material->colour = tyga::Vector3(0.f, 0.5f, 0.f);


	//creates Pelvis
	auto pelvis_mesh = graphics->newMeshWithIdentifier("bloke.tcf/pelvis");
	auto pelvis_model = graphics->newModel();
	pelvis_model->material = bloke_material;
	pelvis_model->mesh = pelvis_mesh;
	pelvis_actor_ = std::make_shared<tyga::Actor>();
	pelvis_actor_->attachComponent(pelvis_model);

	world->addActor(pelvis_actor_);
	//end creating pelvis

	//creates Torso
	auto torso_mesh = graphics->newMeshWithIdentifier("bloke.tcf/torso");
	auto torso_model = graphics->newModel();
	torso_model->material = bloke_material;
	torso_model->mesh = torso_mesh;
	torso_actor_ = std::make_shared<tyga::Actor>();
	torso_actor_->attachComponent(torso_model);

	world->addActor(torso_actor_);
	//end creating Torso

	//creates Helmet
	auto helmet_mesh = graphics->newMeshWithIdentifier("bloke.tcf/helmet");
	auto helmet_model = graphics->newModel();
	helmet_model->material = bloke_material;
	helmet_model->mesh = helmet_mesh;
	helmet_actor_ = std::make_shared<tyga::Actor>();
	helmet_actor_->attachComponent(helmet_model);

	world->addActor(helmet_actor_);
	//end creating Helmet

	//creates Arm_Left
	auto arm_left_mesh = graphics->newMeshWithIdentifier("bloke.tcf/arm_left");
	auto arm_left_model = graphics->newModel();
	arm_left_model->material = bloke_material;
	arm_left_model->mesh = arm_left_mesh;
	arm_left_actor_ = std::make_shared<tyga::Actor>();
	arm_left_actor_->attachComponent(arm_left_model);

	world->addActor(arm_left_actor_);
	//end creating Arm_Left

	//creates Forearm_Left
	auto forearm_left_mesh = graphics->newMeshWithIdentifier("bloke.tcf/forearm_left");
	auto forearm_left_model = graphics->newModel();
	forearm_left_model->material = bloke_material;
	forearm_left_model->mesh = forearm_left_mesh;
	forearm_left_actor_ = std::make_shared<tyga::Actor>();
	forearm_left_actor_->attachComponent(forearm_left_model);

	world->addActor(forearm_left_actor_);
	//end creating Forearm_Left

	//creates Hand_Left
	auto hand_left_mesh = graphics->newMeshWithIdentifier("bloke.tcf/hand_left");
	auto hand_left_model = graphics->newModel();
	hand_left_model->material = bloke_material;
	hand_left_model->mesh = hand_left_mesh;
	hand_left_actor_ = std::make_shared<tyga::Actor>();
	hand_left_actor_->attachComponent(hand_left_model);

	world->addActor(hand_left_actor_);
	//end creating Hand_Left


	//creates Arm_Right
	auto arm_right_mesh = graphics->newMeshWithIdentifier("bloke.tcf/arm_right");
	auto arm_right_model = graphics->newModel();
	arm_right_model->material = bloke_material;
	arm_right_model->mesh = arm_right_mesh;
	arm_right_actor_ = std::make_shared<tyga::Actor>();
	arm_right_actor_->attachComponent(arm_right_model);

	world->addActor(arm_left_actor_);
	//end creating Arm_Right

	//creates Forearm_Right
	auto forearm_right_mesh = graphics->newMeshWithIdentifier("bloke.tcf/forearm_right");
	auto forearm_right_model = graphics->newModel();
	forearm_right_model->material = bloke_material;
	forearm_right_model->mesh = forearm_right_mesh;
	forearm_right_actor_ = std::make_shared<tyga::Actor>();
	forearm_right_actor_->attachComponent(forearm_right_model);

	world->addActor(forearm_right_actor_);
	//end creating Forearm_Right

	//creates Hand_Right
	auto hand_right_mesh = graphics->newMeshWithIdentifier("bloke.tcf/hand_right");
	auto hand_right_model = graphics->newModel();
	hand_right_model->material = bloke_material;
	hand_right_model->mesh = hand_right_mesh;
	hand_right_actor_ = std::make_shared<tyga::Actor>();
	hand_right_actor_->attachComponent(hand_right_model);

	world->addActor(forearm_left_actor_);
	//end creating Hand_Right



	//creates Leg_left
	auto leg_left_mesh = graphics->newMeshWithIdentifier("bloke.tcf/leg_left");
	auto leg_left_model = graphics->newModel();
	leg_left_model->material = bloke_material;
	leg_left_model->mesh = leg_left_mesh;
	leg_left_actor_ = std::make_shared<tyga::Actor>();
	leg_left_actor_->attachComponent(leg_left_model);

	world->addActor(leg_left_actor_);
	//end creating Leg_left

	//creates Shin_Left
	auto shin_left_mesh = graphics->newMeshWithIdentifier("bloke.tcf/shin_left");
	auto shin_left_model = graphics->newModel();
	shin_left_model->material = bloke_material;
	shin_left_model->mesh = shin_left_mesh;
	shin_left_actor_ = std::make_shared<tyga::Actor>();
	shin_left_actor_->attachComponent(shin_left_model);

	world->addActor(shin_left_actor_);
	//end creating Shin_Left

	//creates Foot_Left
	auto foot_left_mesh = graphics->newMeshWithIdentifier("bloke.tcf/foot_left");
	auto foot_left_model = graphics->newModel();
	foot_left_model->material = bloke_material;
	foot_left_model->mesh = foot_left_mesh;
	foot_left_actor_ = std::make_shared<tyga::Actor>();
	foot_left_actor_->attachComponent(foot_left_model);

	world->addActor(hand_left_actor_);
	//end creating Foot_Left

	//creates Leg_right
	auto leg_right_mesh = graphics->newMeshWithIdentifier("bloke.tcf/leg_right");
	auto leg_right_model = graphics->newModel();
	leg_right_model->material = bloke_material;
	leg_right_model->mesh = leg_right_mesh;
	leg_right_actor_ = std::make_shared<tyga::Actor>();
	leg_right_actor_->attachComponent(leg_right_model);

	world->addActor(leg_right_actor_);
	//end creating Leg_right

	//creates Shin_right
	auto shin_right_mesh = graphics->newMeshWithIdentifier("bloke.tcf/shin_right");
	auto shin_right_model = graphics->newModel();
	shin_right_model->material = bloke_material;
	shin_right_model->mesh = shin_right_mesh;
	shin_right_actor_ = std::make_shared<tyga::Actor>();
	shin_right_actor_->attachComponent(shin_right_model);

	world->addActor(shin_left_actor_);
	//end creating Shin_right

	//creates Foot_right
	auto foot_right_mesh = graphics->newMeshWithIdentifier("bloke.tcf/foot_right");
	auto foot_right_model = graphics->newModel();
	foot_right_model->material = bloke_material;
	foot_right_model->mesh = foot_right_mesh;
	foot_right_actor_ = std::make_shared<tyga::Actor>();
	foot_right_actor_->attachComponent(foot_right_model);

	world->addActor(hand_right_actor_);
	//end creating Foot_right
}


void Badger::PositionBloke(std::shared_ptr<tyga::Actor> actor){


	auto pelvis_xform = MyUtils::translate(0.f, 0.757f, -0.243f) * chassis_actor_->Transformation();
	auto torso_xform = MyUtils::positioning_euler(36.f, 0.f, 0.f, 0.f, 0.f, 0.f) * pelvis_xform;
	auto helmet_xform = MyUtils::positioning_euler(-20.713, 4.205f, 9.615f, 0.f, 0.544, -0.019f) * torso_xform;

	auto arm_left_xform = MyUtils::positioning_euler(-45.f, 18.f, -10.f, 0.213f, 0.39f, -0.052f) * torso_xform;
	auto forearm_left_xform = MyUtils::positioning_euler(-56.f, 0.f, 0.f, 0.04f, -0.332f, 0.024) * arm_left_xform;
	auto hand_left_xform = MyUtils::translate(-0.003f, -0.265f, 0.048f) * forearm_left_xform;

	auto arm_right_xform = MyUtils::positioning_euler(-45.f, -18.f, 10.f, -0.213f, 0.39f, -0.052f) * torso_xform;
	auto forearm_right_xform = MyUtils::positioning_euler(-56.f, 0.f, 0.f, -0.04f, -0.332f, 0.024) * arm_right_xform;
	auto hand_right_xform = MyUtils::translate(0.003f, -0.265f, 0.048f) * forearm_right_xform;

	auto leg_left_xform = MyUtils::positioning_euler(-16.f, 18.f, 9.f, 0.101f, -0.074f, 0.008f) * torso_xform;
	auto shin_left_xform = MyUtils::positioning_euler(30.f, 0.f, 0.f, 0.041f, -0.462f, -0.039f) * leg_left_xform;
	auto foot_left_xform = MyUtils::translate(0.019f, -0.416f, 0.006f) * shin_left_xform;

	auto leg_right_xform = MyUtils::positioning_euler(-16.f, -18.f, -9.f, -0.101f, -0.074f, 0.008f) * torso_xform;
	auto shin_right_xform = MyUtils::positioning_euler(30.f, 0.f, 0.f, -0.041f, -0.462f, -0.039f) * leg_right_xform;
	auto foot_right_xform = MyUtils::translate(-0.019f, -0.416f, 0.006f) * shin_right_xform;



	pelvis_actor_->setTransformation(pelvis_xform);
	torso_actor_->setTransformation(torso_xform);
	helmet_actor_->setTransformation(helmet_xform);

	arm_left_actor_->setTransformation(arm_left_xform);
	forearm_left_actor_->setTransformation(forearm_left_xform);
	hand_left_actor_->setTransformation(hand_left_xform);

	arm_right_actor_->setTransformation(arm_right_xform);
	forearm_right_actor_->setTransformation(forearm_right_xform);
	hand_right_actor_->setTransformation(hand_right_xform);

	leg_left_actor_->setTransformation(leg_left_xform);
	shin_left_actor_->setTransformation(shin_left_xform);
	foot_left_actor_->setTransformation(foot_left_xform);

	leg_right_actor_->setTransformation(leg_right_xform);
	shin_right_actor_->setTransformation(shin_right_xform);
	foot_right_actor_->setTransformation(foot_right_xform);
}


void Badger::actorDidEnterWorld(std::shared_ptr<tyga::Actor> actor)
{
	auto world = tyga::ActorWorld::defaultWorld();
	auto graphics = tyga::GraphicsCentre::defaultCentre();

   SpawnBadger(actor);
   SpawnBloke(actor);
	 
   PositionBloke(actor);

   particle_system_ = std::make_shared<MyParticleSystem>();
   graphic_sprite_ = graphics->newSpriteWithDelegate(particle_system_);

   setSource(actor);
   setSink(actor);
}

void Badger::actorWillLeaveWorld(std::shared_ptr<tyga::Actor> actor)
{
    auto world = tyga::ActorWorld::defaultWorld();

    world->removeActor(physics_actor_);
}

void Badger::actorClockTick(std::shared_ptr<tyga::Actor> actor)
{
	const float global_time = tyga::BasicWorldClock::CurrentTime();
	const float delta_time = tyga::BasicWorldClock::CurrentTickInterval();

    //auto physics_local_xform = tyga::Matrix4x4(0.75f,     0,     0,     0,
    //                                               0,   1.f,     0,     0,
    //                                               0,     0,  10.5f,     0,
    //                                               0,     1,     0,     1);
    //auto physics_xform = physics_local_xform * actor->Transformation();
    //physics_actor_->setTransformation(physics_xform);

	

	 const float wheel_diameter = 1.f;


	const float loopedTime = fmodf(global_time, 12.f);

	const float WHEEL_BASE = 2.065f;
	const float MAX_SPEED = 10.f;
	const float MAX_WHEEL_TURN_ANGLE = 0.35f; // radians

	if (delta_time <= 0) {
		auto chassis_xform = MyUtils::translate(0.f, 0.756f, -0.075f);
		chassis_actor_->setTransformation(chassis_xform);
	}

	if (loopedTime < 3) {
			currentSpeed = MyUtils::linearStep(0, 3, loopedTime);
	}
	else if (loopedTime > 3 && loopedTime < 4) {
		BlokeLeft(chassis_actor_);
		turnRate = 0 - MyUtils::linearStep(3, 4, loopedTime);
		EmittParticles(actor);
	}
	else if (loopedTime > 4 && loopedTime < 7) {
		BlokeRight(chassis_actor_);
		turnRate = -1 + (MyUtils::linearStep(4, 7, loopedTime) * 2);
		EmittParticles(actor);
	}
	else if (loopedTime > 7 && loopedTime < 9) {
		turnRate = 1 - MyUtils::linearStep(7, 9, loopedTime);
		currentSpeed = 1 - MyUtils::linearStep(7, 9, loopedTime);
	}

	wheelAngle += MAX_SPEED * currentSpeed * delta_time;
	wheelTurn = turnRate * MAX_WHEEL_TURN_ANGLE;
	handleTurn = turnRate * MAX_WHEEL_TURN_ANGLE;


	distanceTravelled = MAX_SPEED * currentSpeed * delta_time;
	deltaHeadingAngle = (distanceTravelled * sin(wheelTurn)) / 2;



	auto currentPosition = chassis_actor_->Transformation();
	auto newPosition = MyUtils::rotateY(deltaHeadingAngle) * MyUtils::translate(0.f, 0.765f, distanceTravelled) * currentPosition;


	auto handlebar_angle = MyUtils::rotateY(handleTurn);
	auto handlebar_xform = handlebar_angle * MyUtils::rotateX(-0.67690f) * MyUtils::translate(0.f, (0.633f + 0.765), 0.411f);
	auto luggage_xform = MyUtils::translate(0.f, (0.630f + 0.765), -1.075f);
	auto Wheel1_rotate = MyUtils::rotateZ(1.5708f);
	auto Wheel2_rotate = MyUtils::rotateZ(-1.5708f);
	auto Wheel1_xform = Wheel1_rotate * MyUtils::translate(0.654f, (-0.235f + 0.765f), 1.065f);
	auto Wheel2_xform = Wheel2_rotate * MyUtils::translate(-0.654f, (-0.235f + 0.765f), 1.065f);
	auto Wheel3_xform = Wheel1_rotate * MyUtils::translate(0.654f, (-0.235f + 0.765f), -1.f);
	auto Wheel4_xform = Wheel2_rotate * MyUtils::translate(-0.654f, (-0.235f + 0.765f), -1.f);

	auto RWheelRotation = MyUtils::rotateY(wheelAngle);
	auto LWheelRotation = MyUtils::rotateY(wheelAngle*-1);

	chassis_actor_->setTransformation(newPosition);


	handlebars_actor_->setTransformation(handlebar_xform * chassis_actor_->Transformation());
	luggage_actor_->setTransformation(luggage_xform *chassis_actor_->Transformation());
	wheels_actor_[0]->setTransformation(LWheelRotation * MyUtils::rotateX(-wheelTurn) * Wheel1_xform * chassis_actor_->Transformation());
	wheels_actor_[1]->setTransformation(RWheelRotation * MyUtils::rotateX(wheelTurn) * Wheel2_xform * chassis_actor_->Transformation());
	wheels_actor_[2]->setTransformation(LWheelRotation  * Wheel3_xform * chassis_actor_->Transformation());
	wheels_actor_[3]->setTransformation(RWheelRotation * Wheel4_xform * chassis_actor_->Transformation());

	EmittParticles(actor);
}

void Badger::BlokeLeft(std::shared_ptr<tyga::Actor> actor){


	auto pelvis_xform = MyUtils::translate(0.f, 0.757f, -0.24f) * chassis_actor_->Transformation();
	auto torso_xform = MyUtils::positioning_euler(41.f, 9.f, -10.f, 0.f, 0.f, 0.f) * pelvis_xform;
	auto helmet_xform = MyUtils::positioning_euler(-25.f, -4.f, 6.f, 0.f, 0.544, -0.019f) * torso_xform;

	auto arm_left_xform = MyUtils::positioning_euler(-26.f, 7.f, 5.f, 0.213f, 0.39f, -0.052f) * torso_xform;
	auto forearm_left_xform = MyUtils::positioning_euler(-81.f, 0.f, 0.f, 0.04f, -0.332f, 0.024) * arm_left_xform;
	auto hand_left_xform = MyUtils::translate(-0.003f, -0.265f, 0.048f) * forearm_left_xform;

	auto arm_right_xform = MyUtils::positioning_euler(-45.f, -18.f, 10.f, -0.213f, 0.39f, -0.052f) * torso_xform;
	auto forearm_right_xform = MyUtils::positioning_euler(-56.f, 0.f, 0.f, -0.04f, -0.332f, 0.024) * arm_right_xform;
	auto hand_right_xform = MyUtils::translate(0.003f, -0.265f, 0.048f) * forearm_right_xform;

	auto leg_left_xform = MyUtils::positioning_euler(-16.f, 18.f, 9.f, 0.101f, -0.074f, 0.008f) * torso_xform;
	auto shin_left_xform = MyUtils::positioning_euler(30.f, 0.f, 0.f, 0.041f, -0.462f, -0.039f) * leg_left_xform;
	auto foot_left_xform = MyUtils::translate(0.019f, -0.416f, 0.006f) * shin_left_xform;

	auto leg_right_xform = MyUtils::positioning_euler(-16.f, -18.f, -9.f, -0.101f, -0.074f, 0.008f) * torso_xform;
	auto shin_right_xform = MyUtils::positioning_euler(30.f, 0.f, 0.f, -0.041f, -0.462f, -0.039f) * leg_right_xform;
	auto foot_right_xform = MyUtils::translate(-0.019f, -0.416f, 0.006f) * shin_right_xform;



	pelvis_actor_->setTransformation(pelvis_xform);
	torso_actor_->setTransformation(torso_xform);
	helmet_actor_->setTransformation(helmet_xform);

	arm_left_actor_->setTransformation(arm_left_xform);
	forearm_left_actor_->setTransformation(forearm_left_xform);
	hand_left_actor_->setTransformation(hand_left_xform);

	arm_right_actor_->setTransformation(arm_right_xform);
	forearm_right_actor_->setTransformation(forearm_right_xform);
	hand_right_actor_->setTransformation(hand_right_xform);

	leg_left_actor_->setTransformation(leg_left_xform);
	shin_left_actor_->setTransformation(shin_left_xform);
	foot_left_actor_->setTransformation(foot_left_xform);

	leg_right_actor_->setTransformation(leg_right_xform);
	shin_right_actor_->setTransformation(shin_right_xform);
	foot_right_actor_->setTransformation(foot_right_xform);
}

void Badger::BlokeRight(std::shared_ptr<tyga::Actor> actor){


	auto pelvis_xform = MyUtils::translate(0.f, 0.757f, -0.24f) * chassis_actor_->Transformation();
	auto torso_xform = MyUtils::positioning_euler(42.f, -5.f, 6.f, 0.f, 0.f, 0.f) * pelvis_xform;
	auto helmet_xform = MyUtils::positioning_euler(-21.f, 2.f, -2.f, 0.f, 0.544, -0.019f) * torso_xform;

	auto arm_left_xform = MyUtils::positioning_euler(-45.f, 22.f, 8.f, 0.213f, 0.39f, -0.052f) * torso_xform;
	auto forearm_left_xform = MyUtils::positioning_euler(-66.f, 0.f, 0.f, 0.04f, -0.332f, 0.024) * arm_left_xform;
	auto hand_left_xform = MyUtils::translate(-0.003f, -0.265f, 0.048f) * forearm_left_xform;

	auto arm_right_xform = MyUtils::positioning_euler(-27.f, -11.f, 6.f, -0.213f, 0.39f, -0.052f) * torso_xform;
	auto forearm_right_xform = MyUtils::positioning_euler(-79.f, 0.f, 0.f, -0.04f, -0.332f, 0.024) * arm_right_xform;
	auto hand_right_xform = MyUtils::translate(0.003f, -0.265f, 0.048f) * forearm_right_xform;

	auto leg_left_xform = MyUtils::positioning_euler(-16.f, 18.f, 9.f, 0.101f, -0.074f, 0.008f) * torso_xform;
	auto shin_left_xform = MyUtils::positioning_euler(30.f, 0.f, 0.f, 0.041f, -0.462f, -0.039f) * leg_left_xform;
	auto foot_left_xform = MyUtils::translate(0.019f, -0.416f, 0.006f) * shin_left_xform;

	auto leg_right_xform = MyUtils::positioning_euler(-16.f, -18.f, -9.f, -0.101f, -0.074f, 0.008f) * torso_xform;
	auto shin_right_xform = MyUtils::positioning_euler(30.f, 0.f, 0.f, -0.041f, -0.462f, -0.039f) * leg_right_xform;
	auto foot_right_xform = MyUtils::translate(-0.019f, -0.416f, 0.006f) * shin_right_xform;



	pelvis_actor_->setTransformation(pelvis_xform);
	torso_actor_->setTransformation(torso_xform);
	helmet_actor_->setTransformation(helmet_xform);

	arm_left_actor_->setTransformation(arm_left_xform);
	forearm_left_actor_->setTransformation(forearm_left_xform);
	hand_left_actor_->setTransformation(hand_left_xform);

	arm_right_actor_->setTransformation(arm_right_xform);
	forearm_right_actor_->setTransformation(forearm_right_xform);
	hand_right_actor_->setTransformation(hand_right_xform);

	leg_left_actor_->setTransformation(leg_left_xform);
	shin_left_actor_->setTransformation(shin_left_xform);
	foot_left_actor_->setTransformation(foot_left_xform);

	leg_right_actor_->setTransformation(leg_right_xform);
	shin_right_actor_->setTransformation(shin_right_xform);
	foot_right_actor_->setTransformation(foot_right_xform);
}

void Badger::EmittParticles(std::shared_ptr<tyga::Actor> actor) {

	const float current_time = tyga::BasicWorldClock::CurrentTime();
	const float delta_time = tyga::BasicWorldClock::CurrentTickInterval();

	particle_system_->particles.resize(max_particals);

	if (!source_actor_ || !sink_actor_) {
		return;
	}
	tyga::Matrix4x4 source_xform = source_actor_->Transformation();

	tyga::Vector3 source_position = MyUtils::Getposition(source_xform);
	tyga::Vector3 source_direction = MyUtils::GetRotation(source_xform);

	tyga::Vector3 emiter_pos = source_position;

	//Particals are held in an object pool
	//if its time to emit another partical 
	if (current_time > next_emission)
	{
		//reset timer
		next_emission = current_time + emit_delay;

		//loop through object and find an unused object
		for (int i = 0; i < particle_system_->particles.size(); i++)
		{
			if (!particle_system_->particles[i].active)
			{
				//Create new partical rest all the values
				particle_system_->particles[i].active = true;

				particle_system_->particles[i].position = source_position;

				particle_system_->particles[i].initial_time = current_time;
				particle_system_->particles[i].alpha = 1.0f;
				particle_system_->particles[i].velocity = MyUtils::GetFoward(source_xform) * particle_system_->particles[i].speed;

				//random sizes, can be negative or positive 
				float rand_scale = rand() % (random_scale * 2) - random_scale;
				particle_system_->particles[i].final_size = min_size + (rand_scale / 10);
				//do this for min and max scale
				rand_scale = rand() % (random_scale * 2) - random_scale;
				particle_system_->particles[i].initial_size = max_size + (rand_scale / 10);
				break;
			}
		}
	}

	//Update all the particals set to inactive or projecting them
	for (int i = 0; i < particle_system_->particles.size(); i++)
	{
		if (particle_system_->particles[i].active)
		{
			if (current_time > particle_system_->particles[i].initial_time + life_span)//if dead
			{
				particle_system_->particles[i].active = false;
			}
			else//Update the object
			{

				tyga::Vector3 vel = particle_system_->particles[i].velocity;
				tyga::Vector3 pos = particle_system_->particles[i].position;

				//calc pos
				particle_system_->particles[i].position = pos + vel * delta_time;
				//calc velocity				
				particle_system_->particles[i].velocity = vel + acceleration * delta_time;

				//calc size over time
				float start_size = particle_system_->particles[i].final_size;
				float end_size = particle_system_->particles[i].initial_size;

				float ellapst_time = current_time - particle_system_->particles[i].initial_time;
				float phase_time = ellapst_time / life_span;
				float size_dif = end_size - start_size;

				float size = start_size + size_dif * phase_time;
				particle_system_->particles[i].size = size;

				//Alpha over time
				particle_system_->particles[i].alpha = 1 - phase_time;


			}

		}
	}

	tyga::Matrix4x4& sink_xform = sink_actor_->Transformation();
	tyga::Vector3 sink_position;

}

std::string MyParticleSystem::graphicsSpriteTexture() const
{
	return "Dirt.png";
}

int MyParticleSystem::graphicsSpriteVertexCount() const
{
	// NB: you may need to adjust this count if you keep dead particles
	return (int)particles.size();
}

void MyParticleSystem::graphicsSpriteGenerate(tyga::GraphicsSpriteVertex vertex_array[]) const
{

	// NB: you may need to adjust this if you want to control the sprite look
	for (unsigned int i=0; i<particles.size(); ++i) {
		vertex_array[i].position = particles[i].position;
		vertex_array[i].size = 0 + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(1.5-0)));
		vertex_array[i].colour = tyga::Vector3(1, 1, 1);
		vertex_array[i].alpha = 1.f;
		vertex_array[i].rotation = 0.f; // NB: has no effect in basic renderer
	}
}
