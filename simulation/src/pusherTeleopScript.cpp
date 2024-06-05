//--------------------------------------------------
// Box Pushing
// pusherTeleopScript.h
// Date: 2022-10-31
//--------------------------------------------------
#include "pusherTeleopScript.h"
#include "common.h"
#include "pusherCommon.h"
#include <algorithm>
#include <atta/component/components/material.h>
#include <atta/component/components/relationship.h>
#include <atta/component/components/transform.h>
#include <atta/graphics/drawer.h>
#include <atta/utils/config.h>
#include <queue>

namespace gfx = atta::graphics;
namespace sns = atta::sensor;
namespace evt = atta::event;

void PusherTeleopScript::update(cmp::Entity entity, float dt) {
    PROFILE();
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
    for (int i = 0; i < 8; i++)
        _irs[i] = infrareds.getChild(i).get<cmp::InfraredSensor>()->measurement;

    _pusher = _entity.get<PusherComponent>();
    _pusher->timer += dt;

    PusherCommon::processCameras(_pusher, _cams);

    // Stop motors
    PusherCommon::move(_entity, atta::vec2(0.0f));

    if (!cmp::getFactory(pusherProto)->getClones().empty() && entity.getId() == cmp::getFactory(pusherProto)->getClones()[0].getId()) {
        // Teleoperated robot
        teleoperate();
    } else {
        // Chen et al.
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
        }
    }
}

//---------- Teleoperation ----------//
struct WallInfo {
    atta::vec2 pos;
    atta::vec2 size;
};
std::vector<WallInfo> teleopWalls;    ///< Map walls
std::vector<WallInfo> teleopGapWalls; ///< Map walls with gap

struct TeleopNode {
    atta::vec2 pos;
    std::vector<TeleopNode*> neighbors;
};
std::vector<TeleopNode> teleopNodes;
std::vector<atta::vec2> teleopShortestPath;

bool linesIntersect(const atta::vec2& a1, const atta::vec2& a2, const atta::vec2& b1, const atta::vec2& b2) {
    auto cross = [](const atta::vec2& v1, const atta::vec2& v2) { return v1.x * v2.y - v1.y * v2.x; };

    atta::vec2 d1 = a2 - a1;
    atta::vec2 d2 = b2 - b1;
    float denominator = cross(d1, d2);

    if (std::abs(denominator) < 1e-6)
        return false; // Parallel lines

    atta::vec2 d3 = b1 - a1;
    float t1 = cross(d3, d2) / denominator;
    float t2 = cross(d3, d1) / denominator;

    return t1 >= 0 && t1 <= 1 && t2 >= 0 && t2 <= 1;
}

bool doesEdgeIntersectWalls(const atta::vec2& p1, const atta::vec2& p2, const std::vector<WallInfo>& walls) {
    for (const auto& wall : walls) {
        std::array<atta::vec2, 4> corners = {
            wall.pos + 0.5f * atta::vec2{wall.size.x, wall.size.y}, wall.pos + 0.5f * atta::vec2{wall.size.x, -wall.size.y},
            wall.pos + 0.5f * atta::vec2{-wall.size.x, -wall.size.y}, wall.pos + 0.5f * atta::vec2{-wall.size.x, wall.size.y}};
        if (linesIntersect(p1, p2, corners[0], corners[1]) || linesIntersect(p1, p2, corners[1], corners[2]) ||
            linesIntersect(p1, p2, corners[2], corners[3]) || linesIntersect(p1, p2, corners[3], corners[0])) {
            return true;
        }
    }
    return false;
}

atta::vec2 findPositionAtDistance(const std::vector<atta::vec2>& path, float distance) {
    if (path.empty()) {
        LOG_DEBUG("PusherTeleopScript", "Path is empty");
        return {};
    }

    float accumulatedDistance = 0.0f;
    for (size_t i = 1; i < path.size(); ++i) {
        float segmentLength = length(path[i] - path[i - 1]);
        if (accumulatedDistance + segmentLength >= distance) {
            float overshoot = distance - accumulatedDistance;
            atta::vec2 direction = path[i] - path[i - 1];
            float segmentFraction = overshoot / segmentLength;
            return path[i - 1] + direction * segmentFraction;
        }
        accumulatedDistance += segmentLength;
    }

    // If the path is shorter than the required distance, return the last point.
    return path.back();
}

void PusherTeleopScript::teleoperate() {
    //----- Create walls -----//
    teleopWalls.clear();
    teleopGapWalls.clear();
    atta::vec3 objScale = object.get<cmp::Transform>()->scale;
    float gap = std::max(objScale.x, objScale.y) * 0.5;
    cmp::Relationship* obstR = obstacles.get<cmp::Relationship>();
    for (cmp::Entity obst : obstR->getChildren()) {
        cmp::Transform* t = obst.get<cmp::Transform>();
        teleopWalls.push_back({.pos = {t->position.x, t->position.y}, .size = {t->scale.x, t->scale.y}});
        teleopGapWalls.push_back({.pos = {t->position.x, t->position.y}, .size = {t->scale.x + 2 * gap, t->scale.y + 2 * gap}});
    }

    //----- Create nodes -----//
    teleopNodes.clear();
    for (const auto& w : teleopGapWalls) {
        teleopNodes.push_back({.pos = w.pos + 0.5f * atta::vec2(w.size.x, w.size.y)});
        teleopNodes.push_back({.pos = w.pos + 0.5f * atta::vec2(w.size.x, -w.size.y)});
        teleopNodes.push_back({.pos = w.pos + 0.5f * atta::vec2(-w.size.x, -w.size.y)});
        teleopNodes.push_back({.pos = w.pos + 0.5f * atta::vec2(-w.size.x, w.size.y)});
    }
    teleopNodes.push_back({.pos = atta::vec2(goal.get<cmp::Transform>()->position)});
    teleopNodes.push_back({.pos = atta::vec2(object.get<cmp::Transform>()->position)});
    for (int i = teleopNodes.size() - 1; i >= 0; i--) {
        if (std::abs(teleopNodes[i].pos.x) >= 1.5f || std::abs(teleopNodes[i].pos.y) >= 1.5f)
            teleopNodes.erase(teleopNodes.begin() + i);
    }
    //----- Create edges -----//
    for (size_t i = 0; i < teleopNodes.size(); i++)
        for (size_t j = 1; j < teleopNodes.size(); j++) {
            if (!doesEdgeIntersectWalls(teleopNodes[i].pos, teleopNodes[j].pos, teleopWalls)) {
                teleopNodes[i].neighbors.push_back(&teleopNodes[j]);
                teleopNodes[j].neighbors.push_back(&teleopNodes[i]);
            }
        }

    //----- Dijkstra's algorithm -----//
    auto dijkstra = [&](TeleopNode& startNode, TeleopNode& goalNode) {
        std::unordered_map<TeleopNode*, float> distances;
        std::unordered_map<TeleopNode*, TeleopNode*> previous;
        auto compare = [&](TeleopNode* lhs, TeleopNode* rhs) { return distances[lhs] > distances[rhs]; };
        std::priority_queue<TeleopNode*, std::vector<TeleopNode*>, decltype(compare)> pq(compare);

        for (auto& node : teleopNodes) {
            distances[&node] = std::numeric_limits<float>::infinity();
            previous[&node] = nullptr;
        }
        distances[&startNode] = 0.0f;
        pq.push(&startNode);

        while (!pq.empty()) {
            TeleopNode* currentNode = pq.top();
            pq.pop();

            if (currentNode == &goalNode)
                break;

            for (TeleopNode* neighbor : currentNode->neighbors) {
                float newDist = distances[currentNode] + length(neighbor->pos - currentNode->pos);
                if (newDist < distances[neighbor]) {
                    distances[neighbor] = newDist;
                    previous[neighbor] = currentNode;
                    pq.push(neighbor);
                }
            }
        }

        std::vector<atta::vec2> path;
        for (TeleopNode* at = &goalNode; at != nullptr; at = previous[at])
            path.push_back(at->pos);
        std::reverse(path.begin(), path.end());
        return path;
    };

    TeleopNode& startNode = *std::find_if(teleopNodes.begin(), teleopNodes.end(),
                                          [&](const TeleopNode& node) { return node.pos == atta::vec2(object.get<cmp::Transform>()->position); });

    TeleopNode& goalNode = *std::find_if(teleopNodes.begin(), teleopNodes.end(),
                                         [&](const TeleopNode& node) { return node.pos == atta::vec2(goal.get<cmp::Transform>()->position); });

    teleopShortestPath = dijkstra(startNode, goalNode);

    //----- Move robot -----//
    constexpr float OBJECT_DISTANCE = 0.5f;
    atta::vec2 teleopPos = findPositionAtDistance(teleopShortestPath, OBJECT_DISTANCE);
    cmp::Transform* t = _entity.get<cmp::Transform>();

    // If less than 100ms of simulation, start at right position
    if (atta::Config::getTime() == 0.0f)
        t->position = atta::vec3(teleopPos, t->position.z);

    atta::vec2 moveDir = teleopPos - atta::vec2(t->position);
    float angle = -t->orientation.get2DAngle();
    moveDir.normalize();
    moveDir = atta::vec2(std::cos(angle) * moveDir.x - std::sin(angle) * moveDir.y, std::sin(angle) * moveDir.x + std::cos(angle) * moveDir.y);
    PusherCommon::move(_entity, moveDir);
    _entity.get<cmp::Material>()->set("goal");

    //----- Graphical debugging -----//
    gfx::Drawer::clear("teleop");
    gfx::Drawer::Line line;
    line.c0 = line.c1 = atta::vec4(0.6f, 0.2f, 0.2f, 1.0f);
    // Plot all edges
    // for (const WallInfo& w : teleopGapWalls) {
    //    std::vector<atta::vec2> corners;
    //    corners.push_back(w.pos + 0.5f * atta::vec2(w.size.x, w.size.y));
    //    corners.push_back(w.pos + 0.5f * atta::vec2(w.size.x, -w.size.y));
    //    corners.push_back(w.pos + 0.5f * atta::vec2(-w.size.x, -w.size.y));
    //    corners.push_back(w.pos + 0.5f * atta::vec2(-w.size.x, w.size.y));
    //    for (size_t i = 0; i < 4; i++) {
    //        line.p0 = atta::vec3(corners[i], 0.2);
    //        line.p1 = atta::vec3(corners[(i + 1) % corners.size()], 0.2);
    //        gfx::Drawer::add(line, "teleop");
    //    }
    //}

    // Plot good edges
    line.c0 = line.c1 = atta::vec4(0.4f, 0.2f, 0.8f, 1.0f);
    for (const TeleopNode& n : teleopNodes) {
        for (const TeleopNode* neighbor : n.neighbors) {
            line.p0 = atta::vec3(n.pos, 0.3);
            line.p1 = atta::vec3(neighbor->pos, 0.3);
            gfx::Drawer::add(line, "teleop");
        }
    }

    // Plot shortest path
    line.c0 = line.c1 = atta::vec4(0.4f, 0.8f, 0.2f, 1.0f);
    for (size_t i = 1; i < teleopShortestPath.size(); i++) {
        line.p0 = atta::vec3(teleopShortestPath[i - 1], 0.4);
        line.p1 = atta::vec3(teleopShortestPath[i], 0.4);
        gfx::Drawer::add(line, "teleop");
    }
}

//---------- States ----------//
void PusherTeleopScript::randomWalk() { PusherCommon::randomWalk(_entity, _pusher, _dt, true); }

void PusherTeleopScript::approachObject() { PusherCommon::approachObject(_entity, _pusher, _irs, true); }

void PusherTeleopScript::moveAroundObject() { PusherCommon::moveAroundObject(_entity, _pusher, _irs, _dt, false); }

void PusherTeleopScript::pushObject() { PusherCommon::pushObject(_entity, _pusher); }
