//--------------------------------------------------
// Box Pushing
// pusherComponent.cpp
// Date: 2022-11-06
// By Breno Cunha Queiroz
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
            {AttributeType::FLOAT32, offsetof(PusherComponent, timer1), "timer1"},
            {AttributeType::FLOAT32, offsetof(PusherComponent, lastFrameTime), "lastFrameTime"},
            {AttributeType::FLOAT32, offsetof(PusherComponent, randomWalkAux), "randomWalkAux"},
            {AttributeType::BOOL, offsetof(PusherComponent, clockwise), "clockwise"},
            {AttributeType::FLOAT32, offsetof(PusherComponent, objectDirection), "objectDirection"},
            {AttributeType::FLOAT32, offsetof(PusherComponent, objectDistance), "objectDistance"},
            {AttributeType::FLOAT32, offsetof(PusherComponent, goalDirection), "goalDirection"},
            {AttributeType::FLOAT32, offsetof(PusherComponent, goalDistance), "goalDistance"},
        },
        // Max instances
        1024,
    };

    return desc;
}
