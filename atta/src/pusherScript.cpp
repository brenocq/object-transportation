//--------------------------------------------------
// Box Pushing
// pusherScript.h
// Date: 2022-10-31
// By Breno Cunha Queiroz
//--------------------------------------------------
#include "pusherScript.h"

#include <atta/component/components/material.h>
#include <atta/component/components/rigidBody2D.h>
#include <atta/component/components/transform.h>

void PusherScript::update(cmp::Entity entity, float dt) {
    PROFILE();
    //PROFILE_NAME("PusherScript::update("+std::to_string(entity.getId())+")");
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
    for(int i = 0; i < 8; i++)
        _ir[i] = infrareds.getChild(i).get<cmp::InfraredSensor>()->measurement;

    _pusher = _entity.get<PusherComponent>();
    _pusher->timer += dt;

    processCameras();

    // Stop motors
    move(atta::vec2(0.0f));

    _entity.get<cmp::Material>()->set(_pusher->state == PusherComponent::BE_A_GOAL ? "goal" : "pusher");

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
}

//---------- States ----------//
void PusherScript::randomWalk() {
    const float a = 1.0f;
    const float b = 0.2f;
    _pusher->randomWalkAux += _dt * (rand() / float(RAND_MAX) * a * 2 - a); // Add Unif(-a, a)
    if (_pusher->randomWalkAux < -b)
        _pusher->randomWalkAux = -b;
    if (_pusher->randomWalkAux > b)
        _pusher->randomWalkAux = b;
    move(dirToVec(_pusher->randomWalkAux));

    if (!_pusher->canSeeGoal() && _cams[0]->captureTime > 0) {
        changeState(PusherComponent::BE_A_GOAL);
        return;
    }

    if (_pusher->canSeeObject() && _pusher->canSeeGoal()) {
        changeState(PusherComponent::APPROACH_OBJECT);
        return;
    }
}

void PusherScript::approachObject() {
    const float minDist = 0.15f;  // Minimum distance to the object to change state
    const float minAngle = 0.15f; // The front angle interval is [-minAngle, minAngle]

    if (_pusher->canSeeObject()) {
        float dir = _pusher->objectDirection;

        // Move to object
        move(dirToVec(dir));

        // Check if arrived
        bool isInFront = (dir < minAngle && dir > -minAngle) || (dir < (-M_PI + minAngle) || dir > (M_PI - minAngle));
        if (_pusher->objectDistance < minDist && isInFront) {
            if (_pusher->canSeeGoal())
                changeState(PusherComponent::MOVE_AROUND_OBJECT);
            else
                changeState(PusherComponent::PUSH_OBJECT);
            return;
        }
    } else
        changeState(PusherComponent::RANDOM_WALK);
}

void PusherScript::moveAroundObject() {
    //----- Parameters -----//
    atta::vec2 moveVec(1.0f, 0.0f); // Robot move vector (X is to the forward, Y is left)
    const float minDist = 0.1f;
    const float maxDist = 0.25f;

    //----- Check lost object -----//
    if (!_pusher->canSeeObject()) {
        changeState(PusherComponent::RANDOM_WALK);
        return;
    }

    //----- Check should push -----//
    if (!_pusher->canSeeGoal()) {
        changeState(PusherComponent::PUSH_OBJECT);
        return;
    }

    //----- Timeout -----//
    // If timer reached zero
    if (_pusher->timer >= PusherComponent::moveAroundObjectTimeout) {
        changeState(PusherComponent::RANDOM_WALK);
        return;
    }

    //----- Input -----//
    unsigned idxF = _pusher->clockwise ? 0 : 4;
    if (_pusher->objectDirection < 0) {
        // If robot moving "backward", front sensor is behind
        idxF = (idxF == 4) ? 0 : 4;
    }

    float irF = _ir[idxF]; // IR front
    float irS = 0.0f;
    if (_pusher->clockwise)
        irS = _ir[idxF + 2]; // IR right
    else
        irS = _ir[(idxF - 2 + _ir.size()) % _ir.size()]; // IR left

    //----- Force field -----//
    atta::vec2 objVec = dirToVec(_pusher->objectDirection);

    // Force to move around the object
    moveVec = atta::vec2(-objVec.y, objVec.x);
    if (!_pusher->clockwise)
        moveVec *= -1;

    // Force to keep distance from object
    if (irS > maxDist)
       moveVec += objVec;
    else if (irS < minDist)
       moveVec -= objVec;

    // Force to deviate from obstacles
    if (irF < minDist)
       moveVec = atta::vec2(0.0f, _pusher->clockwise ? -1.0f : 1.0f);

    //----- Output - move -----//
    move(moveVec);
}

void PusherScript::pushObject() {
    // Can see goal
    if (_pusher->canSeeGoal()) {
        changeState(PusherComponent::MOVE_AROUND_OBJECT);
        return;
    }

    // Timeout
    if (_pusher->timer >= PusherComponent::pushObjectTimeout) {
        changeState(PusherComponent::RANDOM_WALK);
        return;
    }

    // Push
    move(dirToVec(_pusher->objectDirection));
}

void PusherScript::beAGoal() {
    _entity.get<cmp::Material>()->set("goal");

    if (_pusher->canSeeGoal() || _pusher->timer >= PusherComponent::beAGoalTimeout || _pusher->objectDistance == 0.0f) {
        _entity.get<cmp::Material>()->set("pusher");
        changeState(PusherComponent::RANDOM_WALK);
    }
}

//---------- Low level functions ----------//
void PusherScript::changeState(PusherComponent::State state) {
    // LOG_DEBUG("Pusher" + std::to_string(_entity.getCloneId()), "Changed to state $0", int(state));
    _pusher->state = state;
    _pusher->timer = 0.0f;
}

void PusherScript::move(atta::vec2 direction) {
    constexpr float wheelD = 0.06f; // Wheel distance
    constexpr float wheelR = 0.01f; // Wheel radius
    constexpr float maxPwr = 50.0f; // Motor maximum power (max 0.5m/s)

    auto r = _entity.get<cmp::RigidBody2D>();

    // If break motors
    if (direction.x == 0.0f && direction.y == 0.0f) {
        r->setLinearVelocity(atta::vec2(0.0f));
        r->setAngularVelocity(0.0f);
        return;
    }

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
    float angle = _entity.get<cmp::Transform>()->orientation.get2DAngle();
    r->setLinearVelocity(atta::vec2(linVel * cos(angle), linVel * sin(angle)));
    r->setAngularVelocity(angVel);
}

atta::vec2 PusherScript::dirToVec(float dir) { return {std::cos(dir), std::sin(dir)}; }

float PusherScript::calcDirection(unsigned y, Color color) {
    // Create row of colors
    std::array<const uint8_t*, 4> images = {_cams[0]->getImage(), _cams[1]->getImage(), _cams[2]->getImage(), _cams[3]->getImage()};
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
    int start = -1;
    int end = -1;
    for (unsigned i = 0; i < row.size(); i++) {
        if (row[i] != color) {
            if (start != end)
                intervals.push_back({start + 1, end});
            start = i;
            end = i;
        } else
            end = i;
    }
    if (start != end)
        intervals.push_back({start + 1, end});

    // If could not find color, return NaN
    if (intervals.empty()) {
        LOG_WARN("PusherScript", "No interval found when calculating direction");
        return NAN;
    }

    // Merge intervals
    if (intervals.size() > 1 && intervals.front().first == 0 && intervals.back().second == row.size() - 1) {
        intervals.front().first = intervals.back().first;
        intervals.pop_back();
    }

    // Find largest interval
    unsigned idxLargest = -1;
    unsigned sizeLargest = 0;
    for (unsigned i = 0; i < intervals.size(); i++) {
        int start = intervals[i].first;
        int end = intervals[i].second;

        // Calculate size
        unsigned size = 0;
        if (start < end)
            size = end - start;
        else if (start == end)
            size = 1;
        else
            size = end + w - size;

        // Update largest
        if (size >= sizeLargest) {
            idxLargest = i;
            sizeLargest = size;
        }
    }

    // Calculate direction to largest interval
    auto interval = intervals[idxLargest];
    // Calculate mean pixel position
    int pixelPos = (interval.first + sizeLargest / 2) % (w * 4);
    // Convert to [0,1]
    float meanPos = pixelPos / float(w * 4);
    // Convert to [-2, 2] (and rotate so 0.0 is to the front)
    meanPos = (meanPos * 4 - 0.5);
    if (meanPos > 2)
        meanPos = -2 + (meanPos - 2);
    // Return direction
    return meanPos * M_PI * 0.5;
}

void PusherScript::processCameras() {
    PROFILE();

    // If it is not a new image, do not process
    if (_cams[0]->captureTime == _pusher->lastFrameTime)
        return;
    _pusher->lastFrameTime = _cams[0]->captureTime;

    // Initialize values as default
    _pusher->objectDirection = NAN;
    _pusher->objectDistance = NAN;
    _pusher->goalDirection = NAN;
    _pusher->goalDistance = NAN;

    // If there is no image available yet, do not continue
    if (_cams[0]->captureTime < 0.0f)
        return;

    // Process images
    std::array<const uint8_t*, 4> images = {_cams[0]->getImage(), _cams[1]->getImage(), _cams[2]->getImage(), _cams[3]->getImage()};
    const unsigned w = _cams[0]->width;
    const unsigned h = _cams[0]->height;
    for (int y = h * 0.8; y >= 0; y--) // Scan from bottom to top (ignore lower pixels where robot is visible)
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
