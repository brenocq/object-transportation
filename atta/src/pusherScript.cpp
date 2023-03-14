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

    _entity.get<cmp::Material>()->set(_pusher->state == PusherComponent::BE_A_GOAL ? "goal" : "pusher");
}

//---------- States ----------//
void PusherScript::randomWalk() {
    const float a = 2.0f;
    const float b = 0.2f;
    _pusher->randomWalkAux += _dt * (rand() / float(RAND_MAX) * a * 2 - a); // Add Unif(-a, a)
    // Clip randomWalkAux angle
    if (_pusher->randomWalkAux < -b)
        _pusher->randomWalkAux = -b;
    if (_pusher->randomWalkAux > b)
        _pusher->randomWalkAux = b;
    PusherCommon::move(_entity, PusherCommon::dirToVec(_pusher->randomWalkAux));

    // If the goal is no longer visible
    if (!_pusher->canSeeGoal() && _pusher->couldSeeGoal) {
        PusherCommon::changeState(_pusher, PusherComponent::BE_A_GOAL);
        return;
    }

    // If can see both the goal and the object
    if (_pusher->canSeeObject() && _pusher->canSeeGoal()) {
        PusherCommon::changeState(_pusher, PusherComponent::APPROACH_OBJECT);
        return;
    }
}

void PusherScript::approachObject() { PusherCommon::approachObject(_entity, _pusher, _irs); }

void PusherScript::moveAroundObject() {
    //----- Check lost object -----//
    if (!_pusher->canSeeObject()) {
        PusherCommon::changeState(_pusher, PusherComponent::RANDOM_WALK);
        return;
    }

    //----- Check should push -----//
    if (!_pusher->canSeeGoal()) {
        PusherCommon::changeState(_pusher, PusherComponent::PUSH_OBJECT);
        return;
    }

    //----- Initialize state -----//
    if (_pusher->timer == _dt) {
        // Choose to move around cw/ccw
        _pusher->clockwise = _pusher->goalDirection > 0;
    }

    //----- Parameters -----//
    atta::vec2 moveVec(1.0f, 0.0f); // Robot move vector (X is to the forward, Y is left)
    const float minDist = 0.1f;
    const float maxDist = 0.25f;

    //----- Timeout -----//
    // If timer reached zero
    if (_pusher->timer >= PusherComponent::moveAroundObjectTimeout) {
        PusherCommon::changeState(_pusher, PusherComponent::RANDOM_WALK);
        return;
    }

    //----- Input -----//
    unsigned idxF = _pusher->clockwise ? 0 : 4;
    if (_pusher->objectDirection < 0) {
        // If robot moving "backward", front sensor is behind
        idxF = (idxF == 4) ? 0 : 4;
    }

    //----- Force field -----//
    atta::vec2 objVec = PusherCommon::dirToVec(_pusher->objectDirection);

    // Force to move around the object
    moveVec = atta::vec2(-objVec.y, objVec.x);
    if (_pusher->clockwise)
        moveVec *= -1;

    // Force to keep distance from object
    if (_pusher->objectDistance > 0.2 || PusherCommon::distInDirection(_irs, _pusher->objectDirection) > 0.2)
        moveVec += objVec;

    //----- Output - move -----//
    PusherCommon::move(_entity, moveVec);
}

void PusherScript::pushObject() { PusherCommon::pushObject(_entity, _pusher); }

void PusherScript::beAGoal() {
    if (_pusher->canSeeGoal() || _pusher->timer >= PusherComponent::beAGoalTimeout ||
        (_pusher->objectDistance == 0.0f && PusherCommon::distInDirection(_irs, _pusher->objectDirection) < 0.1))
        PusherCommon::changeState(_pusher, PusherComponent::RANDOM_WALK);
}
