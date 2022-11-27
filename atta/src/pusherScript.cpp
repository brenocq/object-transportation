//--------------------------------------------------
// Box Pushing
// pusherScript.h
// Date: 2022-10-31
// By Breno Cunha Queiroz
//--------------------------------------------------
#include "pusherScript.h"

#include <atta/component/components/rigidBody2D.h>
#include <atta/component/components/transform.h>

const PusherScript::Color PusherScript::goalColor(0, 255, 0);
const PusherScript::Color PusherScript::objectColor(255, 0, 0);

void PusherScript::update(cmp::Entity entity, float dt) {
    _entity = entity;
    _dt = dt;
    cmp::Entity cameras = _entity.getChild(0);
    _cams[0] = cameras.getChild(0).get<cmp::Camera>();
    _cams[1] = cameras.getChild(1).get<cmp::Camera>();
    _cams[2] = cameras.getChild(2).get<cmp::Camera>();
    _cams[3] = cameras.getChild(3).get<cmp::Camera>();
    _pusher = _entity.get<PusherComponent>();

    processCameras();

    switch (_pusher->state) {
        case PusherComponent::State::RANDOM_WALK:
            randomWalk();
            break;
        case PusherComponent::State::APPROACH_OBJECT:
            approachObject();
            break;
        case PusherComponent::State::MOVE_AROUND_OBJECT:
            moveAroundObject();
            break;
        case PusherComponent::State::PUSH_OBJECT:
            pushObject();
            break;
        case PusherComponent::State::BE_A_GOAL:
            beAGoal();
            break;
    }
}

//---------- States ----------//
void PusherScript::randomWalk() { changeState(PusherComponent::State::APPROACH_OBJECT); }

void PusherScript::approachObject() { move({1, 0}); }

void PusherScript::moveAroundObject() {}

void PusherScript::pushObject() {}

void PusherScript::beAGoal() {}

//---------- Low level functions ----------//
void PusherScript::changeState(PusherComponent::State state) {
    // LOG_DEBUG("Pusher" + std::to_string(_entity.getCloneId()), "Changed to state $0", int(state));
    _pusher->state = state;
}

void PusherScript::move(atta::vec2 direction) {
    constexpr float wheelD = 0.06f; // Wheel distance
    constexpr float wheelR = 0.01f; // Wheel radius
    constexpr float maxPwr = 50.0f; // Motor maximum power (max 0.5m/s)

    // Calculate motor velocities
    direction.normalize();
    float dirAngle = atan2(direction.y, direction.x);                             // Direction angle
    atta::vec2 pwr(cos(dirAngle) - sin(dirAngle), cos(dirAngle) + sin(dirAngle)); // Power
    pwr.normalize();
    pwr *= maxPwr;

    // Calculate linear/angular velocities (differential drive robot)
    float linVel = wheelR / 2.0f * (pwr.x + pwr.y);
    float angVel = wheelR / wheelD * (pwr.x - pwr.y);

    // Apply velocities
    auto r = _entity.get<cmp::RigidBody2D>();
    float angle = _entity.get<cmp::Transform>()->orientation.get2DAngle();
    r->setLinearVelocity(atta::vec2(linVel * cos(angle), linVel * sin(angle)));
    r->setAngularVelocity(angVel);
}

float PusherScript::calcDirection(unsigned y, Color color) {
    // Create row of colors
    std::array<const uint8_t*, 4> images = {_cams[0]->getFrame(), _cams[1]->getFrame(), _cams[2]->getFrame(), _cams[3]->getFrame()};
    const unsigned w = _cams[0]->width;
    const unsigned h = _cams[0]->height;
    std::vector<Color> row(w * 4);

    for (unsigned i = 0; i < w * 4; i++) {
        const uint8_t* img = images[i / w];
        unsigned idx = (y * w + (i % w)) * 3;
        Color color(img[idx + 0], img[idx + 1], img[idx + 2]);
        row[i] = color;
    }

    // Calculate intervals
    std::vector<std::pair<int, int>> intervals;
    int start = 0;
    int end = 0;
    for (unsigned i = 0; i < row.size(); i++) {
        if (row[i] != color) {
            if (start != end)
                intervals.push_back({start, end});
            start = i;
            end = i;
        } else
            end = i;
    }
    if (start != end)
        intervals.push_back({start, end});

    // Merge intervals
    if (intervals.front().first == 0 && intervals.back().second == row.size() - 1) {
        intervals.front().first = intervals.back().first;
        intervals.pop_back();
    }

    //for (auto [b, e] : intervals)
    //    LOG_DEBUG("Pusher", "$0 {$1,$2}", _entity.getId(), b, e);

    // Get largest interval
    return 0.0f;
}

void PusherScript::processCameras() {
    PROFILE();
    // Check if it is a new frame
    if (_cams[0]->frameTime == _pusher->lastFrameTime || _cams[0]->frameTime == 0)
        return;
    _pusher->lastFrameTime = _cams[0]->frameTime;

    // Initialize values as default
    _pusher->objectDirection = NAN;
    _pusher->objectDistance = NAN;
    _pusher->goalDirection = NAN;
    _pusher->goalDistance = NAN;

    // Process images
    std::array<const uint8_t*, 4> images = {_cams[0]->getFrame(), _cams[1]->getFrame(), _cams[2]->getFrame(), _cams[3]->getFrame()};
    const unsigned w = _cams[0]->width;
    const unsigned h = _cams[0]->height;
    for (int y = h - 1; y >= 0; y--)
        for (unsigned x = 0; x < w * 4; x++) {
            // Get pixel values
            const uint8_t* img = images[x / w];
            unsigned idx = (y * w + (x % w)) * 3;
            Color color = {img[idx + 0], img[idx + 1], img[idx + 2]};

            // Update distances
            if (color == objectColor)
                _pusher->objectDistance = y / float(h);
            if (color == goalColor)
                _pusher->goalDistance = y / float(h);

            // Update directions
            if (color == objectColor && std::isnan(_pusher->objectDirection))
                _pusher->objectDirection = calcDirection(y, objectColor);
            if (color == goalColor && std::isnan(_pusher->goalDirection))
                _pusher->goalDirection = calcDirection(y, goalColor);
        }
}
