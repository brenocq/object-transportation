//--------------------------------------------------
// Box Pushing
// pusherComponent.h
// Date: 2022-11-06
// By Breno Cunha Queiroz
//--------------------------------------------------
#ifndef PUSHER_COMPONENT_H
#define PUSHER_COMPONENT_H
#include <atta/component/interface.h>

namespace cmp = atta::component;

struct PusherComponent final : public cmp::Component {
    enum class State : uint8_t {
        RANDOM_WALK = 0,
        APPROACH_OBJECT,
        MOVE_AROUND_OBJECT,
        PUSH_OBJECT,
        BE_A_GOAL,
    };

    State state;
    float timer;
};
ATTA_REGISTER_COMPONENT(PusherComponent);
template <>
cmp::ComponentDescription& cmp::TypedComponentRegistry<PusherComponent>::getDescription();

#endif // PUSHER_COMPONENT_H
