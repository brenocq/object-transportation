//--------------------------------------------------
// Box Pushing
// pusherScript.h
// Date: 2022-10-31
// By Breno Cunha Queiroz
//--------------------------------------------------
#include "pusherScript.h"

#include <atta/component/components/rigidBody2D.h>
#include <atta/component/components/transform.h>

namespace cmp = atta::component;

void PusherScript::update(cmp::Entity entity, float dt) {
    _entity = entity;
    _dt = dt;

    switch (_entity.get<PusherComponent>()->state) {
        case PusherComponent::State::RANDOM_WALK:
            randomWalk();
            break;
        case PusherComponent::State::APPROACH_OBJECT:
            approachObject();
            break;
        case PusherComponent::State::MOVE_AROUND_OBJECT:
            moveAroundObject();
            break;
        case PusherComponent::State::PUSH_OBJECT:
            pushObject();
            break;
        case PusherComponent::State::BE_A_GOAL:
            beAGoal();
            break;
    }
}

//---------- States ----------//
void PusherScript::randomWalk() { changeState(PusherComponent::State::APPROACH_OBJECT); }

void PusherScript::approachObject() { move({1, 0}); }

void PusherScript::moveAroundObject() {}

void PusherScript::pushObject() {}

void PusherScript::beAGoal() {}

//---------- Low level functions ----------//
void PusherScript::changeState(PusherComponent::State state) {
    //LOG_DEBUG("Pusher" + std::to_string(_entity.getCloneId()), "Changed to state $0", int(state));
    _entity.get<PusherComponent>()->state = state;
}

void PusherScript::move(atta::vec2 direction) {
    constexpr float wheelD = 0.06f; // Wheel distance
    constexpr float wheelR = 0.01f; // Wheel radius
    constexpr float maxPwr = 50.0f; // Motor maximum power (max 0.5m/s)

    // Calculate motor velocities
    direction.normalize();
    float dirAngle = atan2(direction.y, direction.x);                             // Direction angle
    atta::vec2 pwr(cos(dirAngle) - sin(dirAngle), cos(dirAngle) + sin(dirAngle)); // Power
    pwr.normalize();
    pwr *= maxPwr;

    // Calculate linear/angular velocities (differential drive robot)
    float linVel = wheelR / 2.0f * (pwr.x + pwr.y);
    float angVel = wheelR / wheelD * (pwr.x - pwr.y);

    // Apply velocities
    auto r = _entity.get<cmp::RigidBody2D>();
    float angle = _entity.get<cmp::Transform>()->orientation.get2DAngle();
    r->setLinearVelocity(atta::vec2(linVel * cos(angle), linVel * sin(angle)));
    r->setAngularVelocity(angVel);
}

void PusherScript::processCamera() {}
