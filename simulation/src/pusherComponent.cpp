//--------------------------------------------------
// Box Pushing
// pusherComponent.cpp
// Date: 2022-11-06
//--------------------------------------------------
#include "pusherComponent.h"

template <>
cmp::ComponentDescription& cmp::TypedComponentRegistry<PusherComponent>::getDescription() {
    static cmp::ComponentDescription desc = {
        "Pusher",
        {
            {AttributeType::UINT32,
             offsetof(PusherComponent, state),
             "state",
             {},
             {},
             {},
             {"RANDOM_WALK", "APPROACH_OBJECT", "MOVE_AROUND_OBJECT", "PUSH_OBJECT", "BE_A_GOAL"}},
            {AttributeType::FLOAT32, offsetof(PusherComponent, timer), "timer"},
            {AttributeType::FLOAT32, offsetof(PusherComponent, lastFrameTime), "lastFrameTime"},
            {AttributeType::FLOAT32, offsetof(PusherComponent, randomWalkAux), "randomWalkAux"},
            {AttributeType::BOOL, offsetof(PusherComponent, clockwise), "clockwise"},
            {AttributeType::BOOL, offsetof(PusherComponent, couldSeeGoal), "couldSeeGoal"},
            {AttributeType::BOOL, offsetof(PusherComponent, angleGreater90), "angleGreater90"},
            {AttributeType::FLOAT32, offsetof(PusherComponent, objectDirection), "objectDirection"},
            {AttributeType::FLOAT32, offsetof(PusherComponent, objectDistance), "objectDistance"},
            {AttributeType::FLOAT32, offsetof(PusherComponent, goalDirection), "goalDirection"},
            {AttributeType::FLOAT32, offsetof(PusherComponent, goalDistance), "goalDistance"},
            {AttributeType::FLOAT32, offsetof(PusherComponent, pushDirection), "pushDirection"},
        },
        // Max instances
        1024,
    };

    return desc;
}
