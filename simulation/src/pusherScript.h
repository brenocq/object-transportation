//--------------------------------------------------
// Box Pushing
// pusherScript.h
// Date: 2022-10-31
//--------------------------------------------------
#ifndef PUSHER_SCRIPT_H
#define PUSHER_SCRIPT_H
#include "pusherComponent.h"
#include <atta/component/components/cameraSensor.h>
#include <atta/component/components/infraredSensor.h>
#include <atta/script/script.h>
#include <atta/script/interface.h>

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

    cmp::Entity _entity;
    float _dt;

    PusherComponent* _pusher;
    std::array<cmp::CameraSensor*, 4> _cams;
    std::array<float, 8> _irs;
};

ATTA_REGISTER_SCRIPT(PusherScript)

#endif // PUSHER_SCRIPT_H
