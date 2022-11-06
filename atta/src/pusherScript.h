//--------------------------------------------------
// Box Pushing
// pusherScript.h
// Date: 2022-10-31
// By Breno Cunha Queiroz
//--------------------------------------------------
#ifndef PUSHER_SCRIPT_H
#define PUSHER_SCRIPT_H
#include <atta/script/script.h>
#include "pusherComponent.h"

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
    void processCamera();


    cmp::Entity _entity;
    float _dt;
};

ATTA_REGISTER_SCRIPT(PusherScript)

#endif // PUSHER_SCRIPT_H
