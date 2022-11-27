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
    enum class State : uint32_t {
        RANDOM_WALK = 0,
        APPROACH_OBJECT,
        MOVE_AROUND_OBJECT,
        PUSH_OBJECT,
        BE_A_GOAL,
    };

    State state = State::RANDOM_WALK;
    float timer = 0.0f;
    float lastFrameTime = 0.0f;

    bool canSeeObject() { return objectDistance >= 0; }
    bool canSeeGoal() { return goalDistance >= 0; }

    float objectDirection = 0.0f;// Direction [-pi, pi]
    float objectDistance = -1.0f;// Distance in pixels from top to bottom
    float goalDirection = 0.0f;// Direction [-pi, pi]
    float goalDistance = -1.0f;// Distance in pixels from top to bottom
};
ATTA_REGISTER_COMPONENT(PusherComponent);
template <>
cmp::ComponentDescription& cmp::TypedComponentRegistry<PusherComponent>::getDescription();

#endif // PUSHER_COMPONENT_H
