//--------------------------------------------------
// Box Pushing
// projectScript.cpp
// Date: 2022-10-31
// By Breno Cunha Queiroz
//--------------------------------------------------
#include "projectScript.h"
#include "common.h"
#include "pusherComponent.h"

#include "imgui.h"
#include <atta/component/components/boxCollider2D.h>
#include <atta/component/components/material.h>
#include <atta/component/components/mesh.h>
#include <atta/component/components/name.h>
#include <atta/component/components/relationship.h>
#include <atta/component/components/rigidBody2D.h>
#include <atta/component/components/transform.h>
#include <atta/graphics/drawer.h>
#include <atta/sensor/interface.h>
#include <atta/utils/config.h>

namespace gfx = atta::graphics;
namespace cmp = atta::component;
namespace sns = atta::sensor;

struct WallInfo {
    atta::vec2 pos;
    atta::vec2 size;
};

struct MapInfo {
    atta::vec2 goalPos;
    atta::vec2 objectPos;
    std::vector<WallInfo> walls;
};

std::map<std::string, MapInfo> maps = {
    {
        "Reference",
        {
            .goalPos = {1.0f, 1.0f},
            .objectPos = {-1.0f, -1.0f},
            .walls = {},
        },
    },
    {
        "Middle",
        {
            .goalPos = {0.0f, 1.0f},
            .objectPos = {0.0f, -1.0f},
            .walls = {{.pos = {0.0f, 0.0f}, .size = {1.5f, 0.5f}}},
        },
    },
    {
        "Corner",
        {
            .goalPos = {-1.0f, 1.0f},
            .objectPos = {-1.0f, -1.0f},
            .walls = {{.pos = {-0.5f, 0.0f}, .size = {2.0f, 0.5f}}},
        },
    },
    {
        "2 Corners",
        {
            .goalPos = {1.0f, 1.0f},
            .objectPos = {-1.0f, -1.0f},
            .walls =
                {
                    {.pos = {-0.5f, -0.5f}, .size = {2.0f, 0.1f}},
                    {.pos = {0.5f, 0.5f}, .size = {2.0f, 0.1f}},
                },
        },
    },
};

void ProjectScript::onLoad() { selectMap("Reference"); }

void ProjectScript::onUnload() { resetMap(); }

void ProjectScript::onStart() { randomizePushers(); }

void ProjectScript::onAttaLoop() {
    drawerPusherLines() ;
}

void ProjectScript::onUIRender() {
    ImGui::SetNextWindowSize(ImVec2(310, 300), ImGuiCond_Once);
    ImGui::Begin("Project");
    {
        uiControl();
        ImGui::Separator();
        uiPusherInspector();
    }
    ImGui::End();
}

void ProjectScript::selectMap(std::string mapName) {
    resetMap();
    MapInfo map = maps[mapName];

    // Move goal/object
    cmp::Transform* ot = object.get<cmp::Transform>();
    cmp::Transform* gt = goal.get<cmp::Transform>();
    ot->position = atta::vec3(map.objectPos, ot->position.z);
    ot->orientation.set2DAngle(0.0f);
    gt->position = atta::vec3(map.goalPos, gt->position.z);
    gt->orientation.set2DAngle(0.0f);

    // Create obstacles
    cmp::Relationship* obstR = obstacles.get<cmp::Relationship>();
    for (WallInfo wi : map.walls) {
        cmp::Entity wall = cmp::createEntity();
        cmp::Transform* t = wall.add<cmp::Transform>();
        t->position = atta::vec3(wi.pos, 0.1f);
        t->scale = atta::vec3(wi.size, 0.2f);
        t->orientation.set2DAngle(0.0f);
        wall.add<cmp::Mesh>()->set("meshes/cube.obj");
        wall.add<cmp::Material>()->set("obstacle");
        wall.add<cmp::Name>()->set("Map wall");
        wall.add<cmp::RigidBody2D>()->type = cmp::RigidBody2D::Type::STATIC;
        wall.add<cmp::BoxCollider2D>();
        obstR->addChild(obstacles, wall);
    }

    _currentMap = mapName;

    if (atta::Config::getState() != atta::Config::State::IDLE)
        randomizePushers();
}

void ProjectScript::resetMap() {
    // Move to goal/obstacle to center
    cmp::Transform* ot = object.get<cmp::Transform>();
    cmp::Transform* gt = goal.get<cmp::Transform>();
    ot->position = atta::vec3(0.0f, 0.0f, ot->position.z);
    gt->position = atta::vec3(0.0f, 0.0f, gt->position.z);

    // Delete created obstacles
    cmp::Relationship* obstR = obstacles.get<cmp::Relationship>();
    std::vector<cmp::Entity> obst = obstR->getChildren();
    for (unsigned i = 4; i < obst.size(); i++)
        cmp::deleteEntity(obst[i]);
}

void ProjectScript::randomizePushers() {
    constexpr float worldSize = 2.9f;
    const float pusherRadius = pusherProto.get<cmp::Transform>()->scale.x;
    const float gap = pusherRadius;
    atta::vec2 goalPos = maps[_currentMap].goalPos;
    float goalRadius = goal.get<cmp::Transform>()->scale.x;
    atta::vec2 objectPos = maps[_currentMap].objectPos;
    atta::vec2 objectSize = object.get<cmp::Transform>()->scale;

    std::vector<WallInfo> walls = maps[_currentMap].walls;
    std::vector<atta::vec2> pusherPositions;
    // For each pusher
    for (cmp::Entity pusher : cmp::getFactory(pusherProto)->getClones()) {
        bool freePosition = false;
        atta::vec2 pos(0.0f);
        while (!freePosition) {
            freePosition = true;
            // Get random position
            float rx = rand() / float(RAND_MAX) * worldSize - worldSize * 0.5f;
            float ry = rand() / float(RAND_MAX) * worldSize - worldSize * 0.5f;
            pos = {rx, ry};

            // Check goal collision
            float dist = (goalPos - pos).length();
            if (dist < goalRadius + pusherRadius + gap) {
                freePosition = false;
                continue;
            }

            // Check object collision
            dist = (pos - objectPos).length();
            if (dist < objectSize.x * sqrt(2) * 0.5 + pusherRadius + gap) {
                freePosition = false;
                continue;
            }

            // Check wall collision
            for (WallInfo wall : walls) {
                float wallMinX = wall.pos.x - wall.size.x * 0.5;
                float wallMaxX = wall.pos.x + wall.size.x * 0.5;
                float wallMinY = wall.pos.y - wall.size.y * 0.5;
                float wallMaxY = wall.pos.y + wall.size.y * 0.5;
                if (pos.x + pusherRadius + gap >= wallMinX && pos.x - pusherRadius - gap <= wallMaxX && pos.y + pusherRadius + gap >= wallMinY &&
                    pos.y - pusherRadius - gap <= wallMaxY) {
                    freePosition = false;
                    continue;
                }
            }

            // Check robot collision
            for (atta::vec2 o : pusherPositions) {
                dist = (o - pos).length();
                if (dist < 2 * pusherRadius + gap) {
                    freePosition = false;
                    continue;
                }
            }
        }
        auto t = pusher.get<cmp::Transform>();
        t->position = atta::vec3(pos, t->position.z);
        t->orientation.set2DAngle(rand() / float(RAND_MAX) * M_PI * 2);
        pusherPositions.push_back(pos);
    }
}

#include "projectScriptUI.cpp"
