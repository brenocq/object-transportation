//--------------------------------------------------
// Box Pushing
// pusherCommon.h
// Date: 2023-02-08
//--------------------------------------------------
#ifndef PUSHER_COMMON_H
#define PUSHER_COMMON_H
#include "common.h"
#include "pusherComponent.h"
#include <atta/component/components/cameraSensor.h>

namespace PusherCommon {

// Auxiliary
void changeState(PusherComponent* pusher, PusherComponent::State state);
void move(cmp::Entity entity, atta::vec2 direction);
atta::vec2 dirToVec(float dir);
float distInDirection(const std::array<float, 8>& irs, float dir);

// States
void randomWalk(cmp::Entity entity, PusherComponent* pusher, float dt, bool isPaperScript);
void approachObject(cmp::Entity entity, PusherComponent* pusher, const std::array<float, 8>& irs, bool isPaperScript);
void moveAroundObject(cmp::Entity entity, PusherComponent* pusher, const std::array<float, 8>& irs, float dt, bool isPaperScript);
void pushObject(cmp::Entity entity, PusherComponent* pusher);

// Processing
void processCameras(PusherComponent* pusher, std::array<cmp::CameraSensor*, 4> cams);

} // namespace PusherCommon

#endif // PUSHER_COMMON_H
