//--------------------------------------------------
// Box Pushing
// projectScript.cpp
// Date: 2022-10-31
// By Breno Cunha Queiroz
//--------------------------------------------------
#include "projectScript.h"
#include "imgui.h"

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

void ProjectScript::onStart() { _map = "reference"; }

void ProjectScript::onStop() {}

void ProjectScript::onUpdateBefore(float dt) {}

void ProjectScript::onUIRender() {
    //---------- Map combo ----------//
    {
        static std::vector<std::string> options = {"Reference", "Middle", "Corner", "2 Corners"};

        ImGui::SetNextItemWidth(40.0f);
        if (ImGui::BeginCombo((_currentMap + "##ComboMap").c_str(), _currentMap.c_str())) {
            for (int i = 0; i < options.size(); i++) {
                const bool selected = (options[i] == _currentMap);
                if (ImGui::Selectable(options[i].c_str(), selected)) {
                    destroyMap(_currentMap);
                    _currentMap = options[i];
                    createMap(_currentMap);
                }
                if (selected)
                    ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }
    }
}

void ProjectScript::createMap(std::string mapName) {
    MapInfo map = maps[mapName];
    object.getComponent<cmp::Transform>()->position = map.objectPos;
    goal.getComponent<cmp::Transform>()->position = map.goalPos;
}

void ProjectScript::destroyMap(std::string mapName) {}
