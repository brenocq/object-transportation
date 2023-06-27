//--------------------------------------------------
// Box Pushing
// pusherPaperScript.h
// Date: 2022-10-31
//--------------------------------------------------
#include "pusherPaperScript.h"
#include "pusherCommon.h"
#include <atta/component/components/material.h>
#include <atta/component/components/transform.h>

void PusherPaperScript::update(cmp::Entity entity, float dt) {
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
    }
}

//---------- States ----------//
void PusherPaperScript::randomWalk() { PusherCommon::randomWalk(_entity, _pusher, _dt, true); }

void PusherPaperScript::approachObject() { PusherCommon::approachObject(_entity, _pusher, _irs, true); }

void PusherPaperScript::moveAroundObject() { PusherCommon::moveAroundObject(_entity, _pusher, _irs, _dt, false); }

void PusherPaperScript::pushObject() { PusherCommon::pushObject(_entity, _pusher); }
