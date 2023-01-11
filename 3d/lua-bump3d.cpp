#include "bump3d.hpp"
#include <lua.hpp>

using namespace bump3d;

#define METANAME "_bump_world_3d"

extern "C" {
int LUAMOD_API luaopen_bump3d(lua_State *L)
{
    return 1;
}
}