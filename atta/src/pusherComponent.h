//--------------------------------------------------
// Box Pushing
// pusherComponent.h
// Date: 2022-11-06
//--------------------------------------------------
#ifndef PUSHER_COMPONENT_H
#define PUSHER_COMPONENT_H
#include <atta/component/interface.h>

namespace cmp = atta::component;

struct PusherComponent final : public cmp::Component {
    enum State : uint32_t {
        RANDOM_WALK = 0,
        APPROACH_OBJECT,
        MOVE_AROUND_OBJECT,
        PUSH_OBJECT,
        BE_A_GOAL,
    };
    static constexpr float moveAroundObjectTimeout = 2 * 60.0f;
    static constexpr float beAGoalTimeout = 2 * 60.0f;
    static constexpr float pushObjectTimeout = 2 * 60.0f;

    State state = State::RANDOM_WALK;
    float timer = 0.0f;// Timer to change state
    float timer1 = 0.0f;// State specific timer
    float lastFrameTime = 0.0f;

    // Auxiliar parameters
    float randomWalkAux = 0.0f;// Auxiliar parameter to perform random walk
    bool clockwise = true;// If should walk around object clockwise
    bool couldSeeGoal = false;// If could see goal in the last frame

    bool canSeeObject() { return !std::isnan(objectDistance); }
    bool canSeeGoal() { return !std::isnan(goalDistance); }

    // Cache image processing result
    float objectDirection = NAN; // Direction [-pi, pi]
    float objectDistance = NAN;  // Distance in pixels from top to bottom
    float goalDirection = NAN;   // Direction [-pi, pi]
    float goalDistance = NAN;    // Distance in pixels from top to bottom

};
ATTA_REGISTER_COMPONENT(PusherComponent);
template <>
cmp::ComponentDescription& cmp::TypedComponentRegistry<PusherComponent>::getDescription();

#endif // PUSHER_COMPONENT_H
