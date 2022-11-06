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
            {AttributeType::UINT8,
             offsetof(PusherComponent, state),
             "state",
             {},
             {},
             {},
             {"RANDOM_WALK", "APPROACH_OBJECT", "MOVE_AROUND_OBJECT", "PUSH_OBJECT", "BE_A_GOAL"}},
            {AttributeType::FLOAT32, offsetof(PusherComponent, timer), "timer"},
        },
        // Max instances
        1024,
    };

    return desc;
}
