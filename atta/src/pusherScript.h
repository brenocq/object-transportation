//--------------------------------------------------
// Box Pushing
// pusherScript.h
// Date: 2022-10-31
// By Breno Cunha Queiroz
//--------------------------------------------------
#ifndef PUSHER_SCRIPT_H
#define PUSHER_SCRIPT_H
#include <atta/script/script.h>

namespace cmp = atta::component;
namespace scr = atta::script;

class PusherScript : public scr::Script {
  public:
    void update(cmp::Entity entity, float dt) override;
};

ATTA_REGISTER_SCRIPT(PusherScript)

#endif // PUSHER_SCRIPT_H
