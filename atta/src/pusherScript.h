//--------------------------------------------------
// Box Pushing
// pusherScript.h
// Date: 2022-10-31
// By Breno Cunha Queiroz
//--------------------------------------------------
#ifndef PUSHER_SCRIPT_H
#define PUSHER_SCRIPT_H
#include "pusherComponent.h"
#include <atta/component/components/camera.h>
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

    struct Color {
        uint8_t r;
        uint8_t g;
        uint8_t b;
        Color() = default;
        Color(uint8_t red, uint8_t green, uint8_t blue) : r(red), g(green), b(blue) {}
        bool operator==(const Color& o) const { return (o.r == r) && (o.g == g) && (o.b == b); }
        bool operator!=(const Color& o) const { return !(o == *this); }
    };

    float calcDirection(unsigned y, Color color);
    void processCameras();

    cmp::Entity _entity;
    float _dt;

    PusherComponent* _pusher;
    std::array<cmp::Camera*, 4> _cams;

    static const PusherScript::Color goalColor;
    static const PusherScript::Color objectColor;
};

ATTA_REGISTER_SCRIPT(PusherScript)

#endif // PUSHER_SCRIPT_H
