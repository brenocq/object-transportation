//--------------------------------------------------
// Box Pushing
// common.h
// Date: 2022-10-31
//--------------------------------------------------
#ifndef COMMON_H
#define COMMON_H
#include <atta/component/interface.h>

namespace cmp = atta::component;

cmp::Entity obstacles(1);
cmp::Entity pusherProto(7);
cmp::Entity object(8);
cmp::Entity goal(9);

struct Color {
    uint8_t r;
    uint8_t g;
    uint8_t b;
    Color() = default;
    Color(uint8_t red, uint8_t green, uint8_t blue) : r(red), g(green), b(blue) {}
    bool operator==(const Color& o) const { return (o.r == r) && (o.g == g) && (o.b == b); }
    bool operator!=(const Color& o) const { return !(o == *this); }
};
inline const Color goalColor(0, 255, 0);
inline const Color objectColor(255, 0, 0);
inline const Color pusherColor(0, 0, 255);

#endif // COMMON_H
