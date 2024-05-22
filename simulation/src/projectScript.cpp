//--------------------------------------------------
// Box Pushing
// projectScript.cpp
// Date: 2022-10-31
//--------------------------------------------------
#include "projectScript.h"
#include "common.h"
#include "pusherComponent.h"

#include "imgui.h"
#include <atta/component/components/boxCollider2D.h>
#include <atta/component/components/circleCollider2D.h>
#include <atta/component/components/material.h>
#include <atta/component/components/mesh.h>
#include <atta/component/components/name.h>
#include <atta/component/components/polygonCollider2D.h>
#include <atta/component/components/prototype.h>
#include <atta/component/components/relationship.h>
#include <atta/component/components/rigidBody2D.h>
#include <atta/component/components/rigidJoint.h>
#include <atta/component/components/script.h>
#include <atta/component/components/transform.h>
#include <atta/event/events/simulationStart.h>
#include <atta/event/events/simulationStop.h>
#include <atta/event/interface.h>
#include <atta/graphics/drawer.h>
#include <atta/sensor/interface.h>
#include <atta/utils/config.h>
#include <queue>

namespace gfx = atta::graphics;
namespace cmp = atta::component;
namespace sns = atta::sensor;
namespace evt = atta::event;

//---------- Maps ----------//
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
        "reference",
        {
            .goalPos = {1.0f, 1.0f},
            .objectPos = {-1.0f, -1.0f},
            .walls = {},
        },
    },
    {
        "middle",
        {
            .goalPos = {0.0f, 1.0f},
            .objectPos = {0.0f, -1.0f},
            .walls = {{.pos = {0.0f, 0.0f}, .size = {1.5f, 0.5f}}},
        },
    },
    {
        "corner",
        {
            .goalPos = {-1.0f, 1.0f},
            .objectPos = {-1.0f, -1.0f},
            .walls = {{.pos = {-0.5f, 0.0f}, .size = {2.0f, 0.5f}}},
        },
    },
    {
        "2-corners",
        {
            .goalPos = {1.0f, 1.0f},
            .objectPos = {-1.0f, -1.0f},
            .walls =
                {
                    {.pos = {-0.5f, -0.5f}, .size = {2.0f, 0.3f}},
                    {.pos = {0.5f, 0.5f}, .size = {2.0f, 0.3f}},
                },
        },
    },
};

//---------- Experiments ----------//
struct Experiment {
    int numRepetitions = 1;
    int numRobots = 20;
    float timeout = 60.0f;
    std::string map = "reference";
    std::string object = "circle";
    std::string initialPos = "random";
    std::string script = "PusherScript";
};

const float gTimeout = 20 * 60.0f; // Global timeout in seconds
std::vector<Experiment> experiments = {
    //---------- RANDOM ----------//
    //// Experiments on the obstacle-free map
    //{.numRepetitions = 50, .numRobots = 5, .timeout = gTimeout, .map = "reference", .object = "square", .initialPos="random", .script = "PusherPaperScript"},
    //{.numRepetitions = 50, .numRobots = 10, .timeout = gTimeout, .map = "reference", .object = "square", .initialPos="random", .script = "PusherPaperScript"},
    //{.numRepetitions = 50, .numRobots = 15, .timeout = gTimeout, .map = "reference", .object = "square", .initialPos="random", .script = "PusherPaperScript"},
    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "reference", .object = "square", .initialPos="random", .script = "PusherPaperScript"},
    //{.numRepetitions = 50, .numRobots = 30, .timeout = gTimeout, .map = "reference", .object = "square", .initialPos="random", .script = "PusherPaperScript"},

    //{.numRepetitions = 50, .numRobots = 5, .timeout = gTimeout, .map = "reference", .object = "square", .initialPos="random", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 10, .timeout = gTimeout, .map = "reference", .object = "square", .initialPos="random", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 15, .timeout = gTimeout, .map = "reference", .object = "square", .initialPos="random", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "reference", .object = "square", .initialPos="random", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 30, .timeout = gTimeout, .map = "reference", .object = "square", .initialPos="random", .script = "PusherScript"},

    //// Experiments on maps with obstacles
    //{.numRepetitions = 50, .numRobots = 5, .timeout = gTimeout, .map = "corner", .object = "square", .initialPos="random", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 10, .timeout = gTimeout, .map = "corner", .object = "square", .initialPos="random", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 15, .timeout = gTimeout, .map = "corner", .object = "square", .initialPos="random", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "corner", .object = "square", .initialPos="random", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 30, .timeout = gTimeout, .map = "corner", .object = "square", .initialPos="random", .script = "PusherScript"},

    //{.numRepetitions = 50, .numRobots = 5, .timeout = gTimeout, .map = "2-corners", .object = "square", .initialPos="random", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 10, .timeout = gTimeout, .map = "2-corners", .object = "square", .initialPos="random", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 15, .timeout = gTimeout, .map = "2-corners", .object = "square", .initialPos="random", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "2-corners", .object = "square", .initialPos="random", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 30, .timeout = gTimeout, .map = "2-corners", .object = "square", .initialPos="random", .script = "PusherScript"},

    //{.numRepetitions = 50, .numRobots = 5, .timeout = gTimeout, .map = "middle", .object = "square", .initialPos="random", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 10, .timeout = gTimeout, .map = "middle", .object = "square", .initialPos="random", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 15, .timeout = gTimeout, .map = "middle", .object = "square", .initialPos="random", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "middle", .object = "square", .initialPos="random", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 30, .timeout = gTimeout, .map = "middle", .object = "square", .initialPos="random", .script = "PusherScript"},

    //// Experiments with different shapes
    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "reference", .object = "circle", .initialPos="random", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "middle", .object = "circle", .initialPos="random", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "corner", .object = "circle", .initialPos="random", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "2-corners", .object = "circle", .initialPos="random", .script = "PusherScript"},

    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "reference", .object = "square", .initialPos="random", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "middle", .object = "square", .initialPos="random", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "corner", .object = "square", .initialPos="random", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "2-corners", .object = "square", .initialPos="random", .script = "PusherScript"},

    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "reference", .object = "rectangle", .initialPos="random", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "middle", .object = "rectangle", .initialPos="random", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "corner", .object = "rectangle", .initialPos="random", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "2-corners", .object = "rectangle", .initialPos="random", .script = "PusherScript"},

    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "reference", .object = "triangle", .initialPos="random", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "middle", .object = "triangle", .initialPos="random", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "corner", .object = "triangle", .initialPos="random", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "2-corners", .object = "triangle", .initialPos="random", .script = "PusherScript"},

    {.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "reference", .object = "plus", .initialPos="random", .script = "PusherScript"},
    {.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "middle", .object = "plus", .initialPos="random", .script = "PusherScript"},
    {.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "corner", .object = "plus", .initialPos="random", .script = "PusherScript"},
    {.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "2-corners", .object = "plus", .initialPos="random", .script = "PusherScript"},

    //---------- TOP ----------//
    //// Experiments on the obstacle-free map
    //{.numRepetitions = 50, .numRobots = 5, .timeout = gTimeout, .map = "reference", .object = "square", .initialPos="top", .script = "PusherPaperScript"},
    //{.numRepetitions = 50, .numRobots = 10, .timeout = gTimeout, .map = "reference", .object = "square", .initialPos="top", .script = "PusherPaperScript"},
    //{.numRepetitions = 50, .numRobots = 15, .timeout = gTimeout, .map = "reference", .object = "square", .initialPos="top", .script = "PusherPaperScript"},
    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "reference", .object = "square", .initialPos="top", .script = "PusherPaperScript"},
    //{.numRepetitions = 50, .numRobots = 30, .timeout = gTimeout, .map = "reference", .object = "square", .initialPos="top", .script = "PusherPaperScript"},

    //{.numRepetitions = 50, .numRobots = 5, .timeout = gTimeout, .map = "reference", .object = "square", .initialPos="top", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 10, .timeout = gTimeout, .map = "reference", .object = "square", .initialPos="top", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 15, .timeout = gTimeout, .map = "reference", .object = "square", .initialPos="top", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "reference", .object = "square", .initialPos="top", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 30, .timeout = gTimeout, .map = "reference", .object = "square", .initialPos="top", .script = "PusherScript"},

    //// Experiments on maps with obstacles
    //{.numRepetitions = 50, .numRobots = 5, .timeout = gTimeout, .map = "corner", .object = "square", .initialPos="top", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 10, .timeout = gTimeout, .map = "corner", .object = "square", .initialPos="top", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 15, .timeout = gTimeout, .map = "corner", .object = "square", .initialPos="top", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "corner", .object = "square", .initialPos="top", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 30, .timeout = gTimeout, .map = "corner", .object = "square", .initialPos="top", .script = "PusherScript"},

    //{.numRepetitions = 50, .numRobots = 5, .timeout = gTimeout, .map = "2-corners", .object = "square", .initialPos="top", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 10, .timeout = gTimeout, .map = "2-corners", .object = "square", .initialPos="top", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 15, .timeout = gTimeout, .map = "2-corners", .object = "square", .initialPos="top", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "2-corners", .object = "square", .initialPos="top", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 30, .timeout = gTimeout, .map = "2-corners", .object = "square", .initialPos="top", .script = "PusherScript"},

    //{.numRepetitions = 50, .numRobots = 5, .timeout = gTimeout, .map = "middle", .object = "square", .initialPos="top", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 10, .timeout = gTimeout, .map = "middle", .object = "square", .initialPos="top", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 15, .timeout = gTimeout, .map = "middle", .object = "square", .initialPos="top", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "middle", .object = "square", .initialPos="top", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 30, .timeout = gTimeout, .map = "middle", .object = "square", .initialPos="top", .script = "PusherScript"},

    //// Experiments with different shapes
    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "reference", .object = "circle", .initialPos="top", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "middle", .object = "circle", .initialPos="top", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "corner", .object = "circle", .initialPos="top", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "2-corners", .object = "circle", .initialPos="top", .script = "PusherScript"},

    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "reference", .object = "square", .initialPos="top", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "middle", .object = "square", .initialPos="top", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "corner", .object = "square", .initialPos="top", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "2-corners", .object = "square", .initialPos="top", .script = "PusherScript"},

    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "reference", .object = "rectangle", .initialPos="top", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "middle", .object = "rectangle", .initialPos="top", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "corner", .object = "rectangle", .initialPos="top", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "2-corners", .object = "rectangle", .initialPos="top", .script = "PusherScript"},

    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "reference", .object = "triangle", .initialPos="top", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "middle", .object = "triangle", .initialPos="top", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "corner", .object = "triangle", .initialPos="top", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "2-corners", .object = "triangle", .initialPos="top", .script = "PusherScript"},

    //---------- BOTTOM ----------//
    // Experiments on the obstacle-free map
    //{.numRepetitions = 50, .numRobots = 5, .timeout = gTimeout, .map = "reference", .object = "square", .initialPos="bottom", .script = "PusherPaperScript"},
    //{.numRepetitions = 50, .numRobots = 10, .timeout = gTimeout, .map = "reference", .object = "square", .initialPos="bottom", .script = "PusherPaperScript"},
    //{.numRepetitions = 50, .numRobots = 15, .timeout = gTimeout, .map = "reference", .object = "square", .initialPos="bottom", .script = "PusherPaperScript"},
    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "reference", .object = "square", .initialPos="bottom", .script = "PusherPaperScript"},
    //{.numRepetitions = 50, .numRobots = 30, .timeout = gTimeout, .map = "reference", .object = "square", .initialPos="bottom", .script = "PusherPaperScript"},

    //{.numRepetitions = 50, .numRobots = 5, .timeout = gTimeout, .map = "reference", .object = "square", .initialPos="bottom", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 10, .timeout = gTimeout, .map = "reference", .object = "square", .initialPos="bottom", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 15, .timeout = gTimeout, .map = "reference", .object = "square", .initialPos="bottom", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "reference", .object = "square", .initialPos="bottom", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 30, .timeout = gTimeout, .map = "reference", .object = "square", .initialPos="bottom", .script = "PusherScript"},

    //// Experiments on maps with obstacles
    //{.numRepetitions = 50, .numRobots = 5, .timeout = gTimeout, .map = "corner", .object = "square", .initialPos="bottom", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 10, .timeout = gTimeout, .map = "corner", .object = "square", .initialPos="bottom", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 15, .timeout = gTimeout, .map = "corner", .object = "square", .initialPos="bottom", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "corner", .object = "square", .initialPos="bottom", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 30, .timeout = gTimeout, .map = "corner", .object = "square", .initialPos="bottom", .script = "PusherScript"},

    //{.numRepetitions = 50, .numRobots = 5, .timeout = gTimeout, .map = "2-corners", .object = "square", .initialPos="bottom", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 10, .timeout = gTimeout, .map = "2-corners", .object = "square", .initialPos="bottom", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 15, .timeout = gTimeout, .map = "2-corners", .object = "square", .initialPos="bottom", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "2-corners", .object = "square", .initialPos="bottom", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 30, .timeout = gTimeout, .map = "2-corners", .object = "square", .initialPos="bottom", .script = "PusherScript"},

    //{.numRepetitions = 50, .numRobots = 5, .timeout = gTimeout, .map = "middle", .object = "square", .initialPos="bottom", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 10, .timeout = gTimeout, .map = "middle", .object = "square", .initialPos="bottom", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 15, .timeout = gTimeout, .map = "middle", .object = "square", .initialPos="bottom", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "middle", .object = "square", .initialPos="bottom", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 30, .timeout = gTimeout, .map = "middle", .object = "square", .initialPos="bottom", .script = "PusherScript"},

    //// Experiments with different shapes
    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "reference", .object = "circle", .initialPos="bottom", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "middle", .object = "circle", .initialPos="bottom", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "corner", .object = "circle", .initialPos="bottom", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "2-corners", .object = "circle", .initialPos="bottom", .script = "PusherScript"},

    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "reference", .object = "square", .initialPos="bottom", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "middle", .object = "square", .initialPos="bottom", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "corner", .object = "square", .initialPos="bottom", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "2-corners", .object = "square", .initialPos="bottom", .script = "PusherScript"},

    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "reference", .object = "rectangle", .initialPos="bottom", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "middle", .object = "rectangle", .initialPos="bottom", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "corner", .object = "rectangle", .initialPos="bottom", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "2-corners", .object = "rectangle", .initialPos="bottom", .script = "PusherScript"},

    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "reference", .object = "triangle", .initialPos="bottom", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "middle", .object = "triangle", .initialPos="bottom", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "corner", .object = "triangle", .initialPos="bottom", .script = "PusherScript"},
    //{.numRepetitions = 50, .numRobots = 20, .timeout = gTimeout, .map = "2-corners", .object = "triangle", .initialPos="bottom", .script = "PusherScript"},
};

//---------- Project Script ----------//
void ProjectScript::onLoad() {
    _currentExperiment = 0;
    _runExperiments = false;
    selectMap("reference");
    selectObject("circle");
    _currentInitialPos = "random";
}

void ProjectScript::onUnload() { resetMap(); }

void ProjectScript::onStart() {
    randomizePushers(_currentInitialPos);

    // Make sure all cameras from the same pusher are synchronized
    for (cmp::Entity pusher : cmp::getFactory(pusherProto)->getClones()) {
        cmp::Entity cams = pusher.getChild(0);
        auto cam0 = cams.getChild(0).get<cmp::CameraSensor>();
        auto cam1 = cams.getChild(1).get<cmp::CameraSensor>();
        auto cam2 = cams.getChild(2).get<cmp::CameraSensor>();
        auto cam3 = cams.getChild(3).get<cmp::CameraSensor>();
        cam3->captureTime = cam2->captureTime = cam1->captureTime = cam0->captureTime;
    }
}

void ProjectScript::onStop() { selectMap(_currentMap); }

std::vector<WallInfo> teleopWalls;
struct TeleopNode {
    atta::vec2 pos;
    std::vector<TeleopNode *> neighbors;
};
std::vector<TeleopNode> teleopNodes;
std::vector<atta::vec2> teleopShortestPath;

bool linesIntersect(const atta::vec2& a1, const atta::vec2& a2, const atta::vec2& b1, const atta::vec2& b2) {
    auto cross = [](const atta::vec2& v1, const atta::vec2& v2) {
        return v1.x * v2.y - v1.y * v2.x;
    };

    atta::vec2 d1 = a2 - a1;
    atta::vec2 d2 = b2 - b1;
    float denominator = cross(d1, d2);

    if (std::abs(denominator) < 1e-6) return false; // Parallel lines

    atta::vec2 d3 = b1 - a1;
    float t1 = cross(d3, d2) / denominator;
    float t2 = cross(d3, d1) / denominator;

    return t1 >= 0 && t1 <= 1 && t2 >= 0 && t2 <= 1;
}

bool doesEdgeIntersectWalls(const atta::vec2& p1, const atta::vec2& p2, const std::vector<WallInfo>& walls) {
    for (const auto& wall : walls) {
        std::array<atta::vec2, 4> corners = {
            wall.pos + 0.45f * atta::vec2{wall.size.x, wall.size.y},
            wall.pos + 0.45f * atta::vec2{wall.size.x, -wall.size.y},
            wall.pos + 0.45f * atta::vec2{-wall.size.x, -wall.size.y},
            wall.pos + 0.45f * atta::vec2{-wall.size.x, wall.size.y}
        };
        if (linesIntersect(p1, p2, corners[0], corners[1]) ||
            linesIntersect(p1, p2, corners[1], corners[2]) ||
            linesIntersect(p1, p2, corners[2], corners[3]) ||
            linesIntersect(p1, p2, corners[3], corners[0])) {
            return true;
        }
    }
    return false;
}

void ProjectScript::onAttaLoop() {
    if (atta::Config::getState() == atta::Config::State::RUNNING) {
        atta::vec2 objPos = atta::vec2(object.get<cmp::Transform>()->position);
        if (_objectPath.empty() || length(objPos - _objectPath.back()) >= 0.01)
            _objectPath.push_back(objPos);
    }

    drawerPusherLines();
    drawerPathLines();

    //----- Create walls -----//
    teleopWalls.clear();
    float gap = 0.1f;// 10cm
    for (const WallInfo w : maps[_currentMap].walls)
        teleopWalls.push_back({.pos = {w.pos.x, w.pos.y}, .size = {w.size.x + 2 * gap, w.size.y + 2 * gap}});
    //----- Create nodes -----//
    teleopNodes.clear();
    for(const auto& w : teleopWalls) {
        teleopNodes.push_back({.pos = w.pos + 0.5f * atta::vec2(w.size.x, w.size.y)});
        teleopNodes.push_back({.pos = w.pos + 0.5f * atta::vec2(w.size.x, -w.size.y)});
        teleopNodes.push_back({.pos = w.pos + 0.5f * atta::vec2(-w.size.x, -w.size.y)});
        teleopNodes.push_back({.pos = w.pos + 0.5f * atta::vec2(-w.size.x, w.size.y)});
    }
    teleopNodes.push_back({.pos = atta::vec2(goal.get<cmp::Transform>()->position)});
    teleopNodes.push_back({.pos = atta::vec2(object.get<cmp::Transform>()->position)});
    for (int i = teleopNodes.size() - 1; i >= 0 ; i--) {
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

            if (currentNode == &goalNode) break;

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

    TeleopNode& startNode = *std::find_if(teleopNodes.begin(), teleopNodes.end(), [&](const TeleopNode& node) {
        return node.pos == atta::vec2(object.get<cmp::Transform>()->position);
    });

    TeleopNode& goalNode = *std::find_if(teleopNodes.begin(), teleopNodes.end(), [&](const TeleopNode& node) {
        return node.pos == atta::vec2(goal.get<cmp::Transform>()->position);
    });

    teleopShortestPath = dijkstra(startNode, goalNode);
}

void ProjectScript::onUIRender() {
    ImGui::SetNextWindowSize(ImVec2(310, 300), ImGuiCond_Once);
    ImGui::Begin("Project");
    {
        uiControl();
        ImGui::Separator();
        uiExperiment();
        runExperiments();
        ImGui::Separator();
        uiPusherInspector();
    }
    ImGui::End();
}

void ProjectScript::selectMap(std::string mapName) {
    resetMap();
    _objectPath.clear();
    MapInfo map = maps[mapName];

    // Move goal/object
    cmp::Transform* ot = object.get<cmp::Transform>();
    ot->position = atta::vec3(map.objectPos, ot->position.z);
    ot->orientation.set2DAngle((rand() / float(RAND_MAX)) * 2 * M_PI);

    cmp::Transform* gt = goal.get<cmp::Transform>();
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
        auto rb = wall.add<cmp::RigidBody2D>();
        rb->type = cmp::RigidBody2D::Type::STATIC;
        rb->friction = 0.0f;
        wall.add<cmp::BoxCollider2D>();
        obstR->addChild(obstacles, wall);
    }

    _currentMap = mapName;

    if (atta::Config::getState() != atta::Config::State::IDLE)
        randomizePushers(_currentInitialPos);
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

void ProjectScript::selectObject(std::string objectName) {
    constexpr float objectMass = 5.0f;
    cmp::Transform oldT = *object.get<cmp::Transform>();
    cmp::RigidBody2D oldRB = *object.get<cmp::RigidBody2D>();
    cmp::deleteEntity(object);
    cmp::createEntity(object);
    *(object.add<cmp::Transform>()) = oldT;
    *(object.add<cmp::RigidBody2D>()) = oldRB;
    object.get<cmp::RigidBody2D>()->mass = objectMass;
    object.add<cmp::Name>()->set("Object");
    object.add<cmp::Material>()->set("object");

    if (objectName == "square" || objectName == "rectangle") {
        object.get<cmp::Transform>()->scale = objectName == "square" ? atta::vec3(0.4f, 0.4f, 0.2f) : atta::vec3(0.6f, 0.15f, 0.2f);
        object.add<cmp::Mesh>()->set("meshes/cube.obj");
        object.add<cmp::BoxCollider2D>();
    } else if (objectName == "circle") {
        object.get<cmp::Transform>()->scale = {0.4f, 0.4f, 0.2f};
        object.add<cmp::Mesh>()->set("meshes/cylinder.obj");
        object.add<cmp::CircleCollider2D>();
    } else if (objectName == "triangle") {
        object.get<cmp::Transform>()->scale = {0.5f, 0.5f, 0.2f};
        object.add<cmp::Mesh>()->set("triangle-object.obj");
        object.add<cmp::PolygonCollider2D>()->points = {{0.2, 0.2}, {-0.4, 0.2}, {0.2, -0.6}, {0.2, 0.2}};
    } else if (objectName == "plus") {
        object.get<cmp::Transform>()->scale = atta::vec3(0.4f, 0.4f, 0.2f);
        object.add<cmp::Mesh>()->set("plus-object.obj");
        object.add<cmp::PolygonCollider2D>()->points = {{-0.05, 0.5}, {-0.05, 0.05}, {-0.5, 0.05}, {-0.5, -0.05}, {-0.05, -0.05},
            {-0.05, -0.5}, {0.05, -0.5}, {0.05, -0.05}, {0.5, -0.05}, {0.5, 0.05}, {0.05, 0.05}, {0.05, 0.5}, {-0.05, 0.5}};
    }

    _currentObject = objectName;
}

void ProjectScript::selectScript(std::string scriptName) {
    pusherProto.get<cmp::Script>()->set(scriptName);
    _currentScript = scriptName;
}

void ProjectScript::randomizePushers(std::string initialPos) {
    constexpr float worldSize = 2.9f;
    const float pusherRadius = pusherProto.get<cmp::Transform>()->scale.x * 0.5f;
    const float gap = pusherRadius;
    atta::vec2 goalPos = maps[_currentMap].goalPos;
    float goalRadius = goal.get<cmp::Transform>()->scale.x * 0.5f;
    atta::vec2 objectPos = maps[_currentMap].objectPos;
    atta::vec2 objectSize = object.get<cmp::Transform>()->scale;

    std::vector<WallInfo> walls = maps[_currentMap].walls;
    std::vector<atta::vec2> pusherPositions;
    // For each pusher
    for (cmp::Entity pusher : cmp::getFactory(pusherProto)->getClones()) {
        bool freePosition = false;
        atta::vec2 pos(0.0f);
        size_t numTries = 10000;
        while (!freePosition && --numTries > 0) {
            freePosition = true;
            // Get random position (lower-left of the map)
            float rx = rand() / float(RAND_MAX) * worldSize - worldSize * 0.5f;
            float ry = 0.0f;
            if (initialPos == "random")
                ry = rand() / float(RAND_MAX) * worldSize - worldSize * 0.5f;
            else if (initialPos == "top")
                ry = rand() / float(RAND_MAX) * worldSize * 0.25f + worldSize * 0.25f;
            else if (initialPos == "bottom")
                ry = rand() / float(RAND_MAX) * worldSize * 0.25f - worldSize * 0.5f;
            else
                LOG_WARN("ProjectScript", "Unknown initial position option [w]$0", initialPos);
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
        if (numTries == 0)
            LOG_WARN("ProjectScript", "Failed to find free position to initialize pusher");
        auto t = pusher.get<cmp::Transform>();
        t->position = atta::vec3(pos, t->position.z);
        t->orientation.set2DAngle(rand() / float(RAND_MAX) * M_PI * 2);
        pusherPositions.push_back(pos);
    }
}

#include "projectScriptExperiments.cpp"
#include "projectScriptUI.cpp"
