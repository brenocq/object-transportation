//--------------------------------------------------
// Box Pushing
// pusherScript.h
// Date: 2022-10-31
//--------------------------------------------------
#ifndef PUSHER_SCRIPT_H
#define PUSHER_SCRIPT_H
#include "common.h"
#include "pusherComponent.h"
#include <atta/component/components/cameraSensor.h>
#include <atta/component/components/infraredSensor.h>
#include <atta/script/script.h>

namespace cmp = atta::component;
namespace scr = atta::script;

class PusherScript : public scr::Script {
  public:
    void update(cmp::Entity entity, float dt) override;

  private:
    // States
    void randomWalk();
    void approachObject();
    void moveAroundObject();
    void pushObject();
    void beAGoal();

    // Low level functions
    void changeState(PusherComponent::State state);
    void move(atta::vec2 direction);
    atta::vec2 dirToVec(float dir);

    float calcDirection(unsigned y, Color color);
    void processCameras();

    cmp::Entity _entity;
    float _dt;

    PusherComponent* _pusher;
    std::array<cmp::CameraSensor*, 4> _cams;
};

ATTA_REGISTER_SCRIPT(PusherScript)

#endif // PUSHER_SCRIPT_H
