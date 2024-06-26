//--------------------------------------------------
// Box Pushing
// pusherScript.h
// Date: 2022-10-31
//--------------------------------------------------
#include "pusherScript.h"
#include "pusherCommon.h"
#include <atta/component/components/material.h>
#include <atta/component/components/rigidBody2D.h>
#include <atta/component/components/transform.h>

void PusherScript::update(cmp::Entity entity, float dt) {
    PROFILE();
    _entity = entity;
    _dt = dt;

    // Get cameras
    cmp::Entity cameras = _entity.getChild(0);
    _cams[0] = cameras.getChild(0).get<cmp::CameraSensor>();
    _cams[1] = cameras.getChild(1).get<cmp::CameraSensor>();
    _cams[2] = cameras.getChild(2).get<cmp::CameraSensor>();
    _cams[3] = cameras.getChild(3).get<cmp::CameraSensor>();

    // Get infrareds
    cmp::Entity infrareds = _entity.getChild(1);
    for (int i = 0; i < 8; i++)
        _irs[i] = infrareds.getChild(i).get<cmp::InfraredSensor>()->measurement;

    _pusher = _entity.get<PusherComponent>();
    _pusher->timer += dt;
    _pusher->beAGoalWait = std::max(0.0f, _pusher->beAGoalWait - dt);

    PusherCommon::processCameras(_pusher, _cams);

    // Stop motors
    PusherCommon::move(_entity, atta::vec2(0.0f));

    switch (_pusher->state) {
        case PusherComponent::RANDOM_WALK:
            randomWalk();
            break;
        case PusherComponent::APPROACH_OBJECT:
            approachObject();
            break;
        case PusherComponent::MOVE_AROUND_OBJECT:
            moveAroundObject();
            break;
        case PusherComponent::PUSH_OBJECT:
            pushObject();
            break;
        case PusherComponent::BE_A_GOAL:
            beAGoal();
            break;
    }

    // Avoid turning into a goal if it was a goal a short time ago
    if (_pusher->beAGoalWait > 0.0f && _pusher->state == PusherComponent::BE_A_GOAL)
        PusherCommon::changeState(_pusher, PusherComponent::RANDOM_WALK);

    _entity.get<cmp::Material>()->set(_pusher->state == PusherComponent::BE_A_GOAL ? "goal" : "pusher");
}

//---------- States ----------//
void PusherScript::randomWalk() { PusherCommon::randomWalk(_entity, _pusher, _dt, false); }

void PusherScript::approachObject() { PusherCommon::approachObject(_entity, _pusher, _irs, false); }

void PusherScript::moveAroundObject() { PusherCommon::moveAroundObject(_entity, _pusher, _irs, _dt, false); }

void PusherScript::pushObject() { PusherCommon::pushObject(_entity, _pusher); }

void PusherScript::beAGoal() {
    bool objectIsClose = _pusher->objectDistance == 0.0f && PusherCommon::distInDirection(_irs, _pusher->objectDirection) < 0.1;
    bool timeout = _pusher->timer >= PusherComponent::beAGoalTimeout;
    if (_pusher->canSeeGoal() || objectIsClose || timeout) {
        _pusher->beAGoalWait = rand() / float(RAND_MAX) * 5.0f; // Wait up to 5 seconds
        PusherCommon::changeState(_pusher, PusherComponent::RANDOM_WALK);
    }
}
