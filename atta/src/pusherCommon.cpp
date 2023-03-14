//--------------------------------------------------
// Box Pushing
// pusherCommon.cpp
// Date: 2023-02-08
//--------------------------------------------------
#include "pusherCommon.h"
#include <atta/component/components/rigidBody2D.h>
#include <atta/component/components/transform.h>

void PusherCommon::changeState(PusherComponent* pusher, PusherComponent::State state) {
    // LOG_DEBUG("Pusher" + std::to_string(_entity.getCloneId()), "Changed to state $0", int(state));
    pusher->state = state;
    pusher->timer = 0.0f;
}

void PusherCommon::move(cmp::Entity entity, atta::vec2 direction) {
    constexpr float wheelD = 0.06f; // Wheel distance
    constexpr float wheelR = 0.01f; // Wheel radius
    constexpr float maxPwr = 50.0f; // Motor maximum power (max 0.5m/s)

    auto r = entity.get<cmp::RigidBody2D>();

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
    float angle = entity.get<cmp::Transform>()->orientation.get2DAngle();
    r->setLinearVelocity(atta::vec2(linVel * cos(angle), linVel * sin(angle)));
    r->setAngularVelocity(angVel);
}

atta::vec2 PusherCommon::dirToVec(float dir) { return {std::cos(dir), std::sin(dir)}; }

float PusherCommon::distInDirection(const std::array<float, 8>& irs, float dir) {
    int idx = std::round((dir + M_PI) / (2 * M_PI) * 8 - 4);
    if (idx < 0)
        idx = 8 + idx;
    return irs[idx];
}

float calcDirection(std::array<cmp::CameraSensor*, 4> cams, unsigned y, Color color) {
    // Create row of colors
    std::array<const uint8_t*, 4> images = {cams[0]->getImage(), cams[1]->getImage(), cams[2]->getImage(), cams[3]->getImage()};
    const unsigned w = cams[0]->width;
    const unsigned h = cams[0]->height;
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

void PusherCommon::approachObject(cmp::Entity entity, PusherComponent* pusher, const std::array<float, 8>& irs) {
    const float minCamDist = 0.15f; // Minimum distance to the object to change state (using Camera)
    const float minIrDist = 0.1f;   // Minimum distance to the object to change state (using IR)
    const float minAngle = 0.15f;   // The front angle interval is [-minAngle, minAngle]

    if (pusher->canSeeObject() && pusher->canSeeGoal()) {
        float dir = pusher->objectDirection;

        // Move to object
        PusherCommon::move(entity, PusherCommon::dirToVec(dir));

        // Check if arrived
        bool isInFront = (dir < minAngle && dir > -minAngle) || (dir < (-M_PI + minAngle) || dir > (M_PI - minAngle));
        if (pusher->objectDistance < minCamDist && PusherCommon::distInDirection(irs, pusher->objectDirection) < minIrDist && isInFront) {
            if (pusher->canSeeGoal())
                PusherCommon::changeState(pusher, PusherComponent::MOVE_AROUND_OBJECT);
            else
                PusherCommon::changeState(pusher, PusherComponent::PUSH_OBJECT);
            return;
        }
    } else
        PusherCommon::changeState(pusher, PusherComponent::RANDOM_WALK);
}

void PusherCommon::pushObject(cmp::Entity entity, PusherComponent* pusher) {
    // Can see goal
    if (pusher->canSeeGoal()) {
        PusherCommon::changeState(pusher, PusherComponent::MOVE_AROUND_OBJECT);
        return;
    }

    // Can't see object anymore
    if (!pusher->canSeeObject()) {
        PusherCommon::changeState(pusher, PusherComponent::RANDOM_WALK);
        return;
    }

    // Timeout
    if (pusher->timer >= PusherComponent::pushObjectTimeout) {
        PusherCommon::changeState(pusher, PusherComponent::RANDOM_WALK);
        return;
    }

    // Push
    PusherCommon::move(entity, PusherCommon::dirToVec(pusher->objectDirection));
}

void PusherCommon::processCameras(PusherComponent* pusher, std::array<cmp::CameraSensor*, 4> cams) {
    PROFILE();

    // If it is not a new image, do not process
    if (cams[0]->captureTime == pusher->lastFrameTime)
        return;

    // Store data about last frame
    pusher->lastFrameTime = cams[0]->captureTime;
    pusher->couldSeeGoal = pusher->canSeeGoal();

    // Initialize values as default
    pusher->objectDirection = NAN;
    pusher->objectDistance = NAN;
    pusher->goalDirection = NAN;
    pusher->goalDistance = NAN;

    // If there is no image available yet, do not continue
    if (cams[0]->captureTime < 0.0f)
        return;

    // Process images
    std::array<const uint8_t*, 4> images = {cams[0]->getImage(), cams[1]->getImage(), cams[2]->getImage(), cams[3]->getImage()};
    const unsigned w = cams[0]->width;
    const unsigned h = cams[0]->height;
    for (int y = h * 0.8; y >= 0; y--) // Scan from bottom to top (ignore lower pixels where robot is visible)
        for (unsigned x = 0; x < w * 4; x++) {
            // Get pixel values
            const uint8_t* img = images[x / w];
            unsigned idx = (y * w + (x % w)) * 3;
            Color color = {img[idx + 0], img[idx + 1], img[idx + 2]};

            // Update distances
            if (color == objectColor)
                pusher->objectDistance = y / float(h);
            if (color == goalColor)
                pusher->goalDistance = y / float(h);

            // Update directions
            if (color == objectColor && std::isnan(pusher->objectDirection))
                pusher->objectDirection = calcDirection(cams, y, objectColor);
            if (color == goalColor && std::isnan(pusher->goalDirection))
                pusher->goalDirection = calcDirection(cams, y, goalColor);
        }
}

