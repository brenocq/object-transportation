//--------------------------------------------------
// Box Pushing
// pusherTeleopScript.h
// Date: 2022-10-31
//--------------------------------------------------
#ifndef PUSHER_TELEOP_SCRIPT_H
#define PUSHER_TELEOP_SCRIPT_H
#include "pusherComponent.h"
#include <atta/component/components/cameraSensor.h>
#include <atta/component/components/infraredSensor.h>
#include <atta/script/interface.h>
#include <atta/script/script.h>

namespace cmp = atta::component;
namespace scr = atta::script;

class PusherTeleopScript : public scr::Script {
  public:
    void update(cmp::Entity entity, float dt) override;

  private:
    void teleoperate();

    // States
    void randomWalk();
    void approachObject();
    void moveAroundObject();
    void pushObject();

    cmp::Entity _entity;
    float _dt;

    PusherComponent* _pusher;
    std::array<cmp::CameraSensor*, 4> _cams;
    std::array<float, 8> _irs;
};

ATTA_REGISTER_SCRIPT(PusherTeleopScript)

#endif // PUSHER_TELEOP_SCRIPT_H
