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

void ProjectScript::onAttaLoop() {
    if (atta::Config::getState() == atta::Config::State::RUNNING) {
        atta::vec2 objPos = atta::vec2(object.get<cmp::Transform>()->position);
        if (_objectPath.empty() || length(objPos - _objectPath.back()) >= 0.01)
            _objectPath.push_back(objPos);
    }

    drawerPusherLines();
    drawerPathLines();
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
    if (_currentObject == "plus") {
        cmp::Transform* ot1 = objectPart1.get<cmp::Transform>();
        ot1->position = ot->position;
        ot1->orientation = ot->orientation;
    }/* else if (_currentObject == "H") {
        cmp::Transform* ot1 = objectPart1.get<cmp::Transform>();
        ot1->position = ot->position;
        ot1->orientation = ot->orientation;

        cmp::Transform* ot2 = objectPart2.get<cmp::Transform>();
        ot2->position = ot->position;
        ot2->orientation = ot->orientation;
    }*/

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
    cmp::deleteEntity(objectPart1);
    cmp::deleteEntity(objectPart2);
    cmp::createEntity(object);
    cmp::createEntity(objectPart1);
    cmp::createEntity(objectPart2);
    *(object.add<cmp::Transform>()) = oldT;
    *(object.add<cmp::RigidBody2D>()) = oldRB;
    object.get<cmp::RigidBody2D>()->mass = objectMass;
    object.add<cmp::Name>()->set("Object");
    objectPart1.add<cmp::Name>()->set("Object Part1");
    objectPart2.add<cmp::Name>()->set("Object Part2");
    object.add<cmp::Material>()->set("object");
    objectPart1.add<cmp::Material>()->set("object");
    objectPart2.add<cmp::Material>()->set("object");

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
        object.get<cmp::Transform>()->scale = atta::vec3(0.4f, 0.05f, 0.2f);
        object.add<cmp::Mesh>()->set("meshes/cube.obj");
        object.add<cmp::BoxCollider2D>();

        *(objectPart1.add<cmp::Transform>()) = oldT;
        *(objectPart1.add<cmp::RigidBody2D>()) = oldRB;
        objectPart1.get<cmp::Transform>()->scale = atta::vec3(0.05f, 0.4f, 0.2f);
        objectPart1.add<cmp::Mesh>()->set("meshes/cube.obj");
        objectPart1.add<cmp::BoxCollider2D>();
        cmp::RigidJoint* rj = objectPart1.add<cmp::RigidJoint>();
        rj->bodyA = object.getId();
        rj->bodyB = objectPart1.getId();

        // Divide mass across parts
        const float partMass = objectMass / 2.0f;
        object.get<cmp::RigidBody2D>()->mass = partMass;
        objectPart1.get<cmp::RigidBody2D>()->mass = partMass;
    } /*else if (objectName == "H") {
        object.get<cmp::Transform>()->scale = atta::vec3(0.3f, 0.05f, 0.2f);
        object.add<cmp::Mesh>()->set("meshes/cube.obj");
        object.add<cmp::BoxCollider2D>();

        // Object part1
        cmp::Transform* t1 = objectPart1.add<cmp::Transform>();
        *t1 = oldT;
        atta::vec3 posVec = {-0.18f, 0.0f, 0.0f};
        t1->orientation.rotateVector(posVec);
        t1->position += posVec;
        *(objectPart1.add<cmp::RigidBody2D>()) = oldRB;
        objectPart1.get<cmp::Transform>()->scale = atta::vec3(0.05f, 0.4f, 0.2f);
        objectPart1.add<cmp::Mesh>()->set("meshes/cube.obj");
        objectPart1.add<cmp::BoxCollider2D>();
        cmp::RigidJoint* rj1 = objectPart1.add<cmp::RigidJoint>();
        rj1->bodyA = object.getId();
        rj1->bodyB = objectPart1.getId();

        // Object part2
        cmp::Transform* t2 = objectPart2.add<cmp::Transform>();
        *t2 = oldT;
        posVec = {0.18f, 0.0f, 0.0f};
        t2->orientation.rotateVector(posVec);
        t2->position += posVec;
        *(objectPart2.add<cmp::RigidBody2D>()) = oldRB;
        objectPart2.get<cmp::Transform>()->scale = atta::vec3(0.05f, 0.4f, 0.2f);
        objectPart2.add<cmp::Mesh>()->set("meshes/cube.obj");
        objectPart2.add<cmp::BoxCollider2D>();
        cmp::RigidJoint* rj2 = objectPart2.add<cmp::RigidJoint>();
        rj2->bodyA = object.getId();
        rj2->bodyB = objectPart2.getId();

        // Divide mass across parts
        const float partMass = objectMass / 3.0f;
        object.get<cmp::RigidBody2D>()->mass = partMass;
        objectPart1.get<cmp::RigidBody2D>()->mass = partMass;
        objectPart2.get<cmp::RigidBody2D>()->mass = partMass;
    }*/

    _currentObject = objectName;
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
