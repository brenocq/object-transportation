//--------------------------------------------------
// Box Pushing
// common.h
// Date: 2022-10-31
// By Breno Cunha Queiroz
//--------------------------------------------------
#ifndef COMMON_H
#define COMMON_H
#include <atta/component/interface.h>

namespace cmp = atta::component;

cmp::Entity obstacles(1);
cmp::Entity pusherProto(7);
cmp::Entity object(8);
cmp::Entity goal(9);

#endif // COMMON_H
