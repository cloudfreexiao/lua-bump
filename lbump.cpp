#include "bump.hpp"
#include <cstddef>
#include <lua.hpp>

using namespace bump;

#define METANAME "BumpWorld"

struct BumpWorldWorld {
    World *world;
};

static const char *luaL_tostring(lua_State *L, int narg)
{
    lua_getglobal(L, "tostring");
    lua_pushvalue(L, narg);
    lua_call(L, 1, 1);
    const char *str = lua_tostring(L, -1);
    lua_pop(L, 1);
    return str;
}

static inline void lauxh_pushint2tblat(lua_State *L, const char *k,
                                       lua_Integer v, int at)
{
    if (at < 0) {
        at -= 2;
    }
    lua_pushstring(L, k);
    lua_pushinteger(L, v);
    lua_rawset(L, at);
}
#define lauxh_pushint2tbl(L, k, v) lauxh_pushint2tblat(L, k, v, -1)

static void assertNumber(lua_State *L, int narg, const char *name)
{
    lua_Integer d = lua_tointeger(L, narg);
    if (d == 0 && !lua_isnumber(L, narg)) /* avoid extra test when d is not 0 */
    {
        lua_pushfstring(L, "%s must be a number, but was %s (a %s)", name,
                        luaL_tostring(L, narg), lua_typename(L, narg));
        lua_error(L);
    }
}

static void assertIsPositiveNumber(lua_State *L, int narg, const char *name)
{
    lua_Integer d = lua_tointeger(L, narg);
    if (d <= 0) {
        lua_pushfstring(L, "%s must be a positive integer, but was %s (a %s)",
                        name, luaL_tostring(L, narg), lua_typename(L, narg));
        lua_error(L);
    }
}

static void assertIsRect(lua_State *L, int x, int y, int w, int h)
{
    assertNumber(L, x, "x");
    assertNumber(L, y, "y");
    assertIsPositiveNumber(L, w, "w");
    assertIsPositiveNumber(L, h, "h");
}

static int worldProject(lua_State *L)
{
    BumpWorldWorld *bump = (BumpWorldWorld *)lua_touserdata(L, 1);
    World *world         = bump->world;

    int item  = lua_tonumber(L, 2);
    double x  = luaL_checknumber(L, 3);
    double y  = luaL_checknumber(L, 4);
    double w  = luaL_checknumber(L, 5);
    double h  = luaL_checknumber(L, 6);
    double gx = luaL_checknumber(L, 7);
    double gy = luaL_checknumber(L, 8);
    ColFilter *filter = world->getFilterById(luaL_optinteger(L, 9, Slide));

    std::vector<Collision> items;
    world->project(item, x, y, w, h, gx, gy, filter, items);
    int n = 0;
    lua_newtable(L);
    for (std::vector<Collision>::iterator it = items.begin(); it != items.end();
         it++) {
        lua_newtable(L);
        lua_rawgeti(L, -2, (*it).item);
        lua_setfield(L, -2, "item");
        lua_rawgeti(L, -2, (*it).other);
        lua_setfield(L, -2, "other");
        lua_pushnumber(L, (*it).type);
        lua_setfield(L, -2, "type");
        lua_pushboolean(L, (*it).overlaps);
        lua_setfield(L, -2, "overlaps");
        lua_pushnumber(L, (*it).ti);
        lua_setfield(L, -2, "ti");

        lua_newtable(L);
        lua_pushnumber(L, (*it).move.x);
        lua_setfield(L, -2, "x");
        lua_pushnumber(L, (*it).move.y);
        lua_setfield(L, -2, "y");
        lua_setfield(L, -2, "move");
        lua_newtable(L);
        lua_pushnumber(L, (*it).normal.x);
        lua_setfield(L, -2, "x");
        lua_pushnumber(L, (*it).normal.y);
        lua_setfield(L, -2, "y");
        lua_setfield(L, -2, "normal");
        lua_newtable(L);
        lua_pushnumber(L, (*it).touch.x);
        lua_setfield(L, -2, "x");
        lua_pushnumber(L, (*it).touch.y);
        lua_setfield(L, -2, "y");
        lua_setfield(L, -2, "touch");

        lua_newtable(L);
        lua_pushnumber(L, (*it).itemRect.x);
        lua_setfield(L, -2, "x");
        lua_pushnumber(L, (*it).itemRect.y);
        lua_setfield(L, -2, "y");
        lua_pushnumber(L, (*it).itemRect.w);
        lua_setfield(L, -2, "w");
        lua_pushnumber(L, (*it).itemRect.h);
        lua_setfield(L, -2, "h");
        lua_setfield(L, -2, "itemRect");
        lua_newtable(L);
        lua_pushnumber(L, (*it).otherRect.x);
        lua_setfield(L, -2, "x");
        lua_pushnumber(L, (*it).otherRect.y);
        lua_setfield(L, -2, "y");
        lua_pushnumber(L, (*it).otherRect.w);
        lua_setfield(L, -2, "w");
        lua_pushnumber(L, (*it).otherRect.h);
        lua_setfield(L, -2, "h");
        lua_setfield(L, -2, "otherRect");

        lua_rawseti(L, -2, ++n);
    }
    lua_pushinteger(L, n);

    return 2;
}

static int worldCountCells(lua_State *L)
{
    BumpWorldWorld *bump = (BumpWorldWorld *)lua_touserdata(L, 1);
    World *world         = bump->world;
    lua_pushnumber(L, world->countCells());
    return 1;
}

static int worldCountItems(lua_State *L)
{
    BumpWorldWorld *bump = (BumpWorldWorld *)lua_touserdata(L, 1);
    World *world         = bump->world;
    lua_pushnumber(L, world->countItems());
    return 1;
}

static int worldHasItem(lua_State *L)
{
    BumpWorldWorld *bump = (BumpWorldWorld *)lua_touserdata(L, 1);
    World *world         = bump->world;
    int item             = lua_tonumber(L, 2);
    lua_pushboolean(L, world->hasItem(item));
    return 1;
}

static int worldGetRect(lua_State *L)
{
    BumpWorldWorld *bump = (BumpWorldWorld *)lua_touserdata(L, 1);
    World *world         = bump->world;
    int item             = lua_tonumber(L, 2);
    double x, y, w, h;
    world->getRect(item, x, y, w, h);
    lua_pushnumber(L, x);
    lua_pushnumber(L, y);
    lua_pushnumber(L, w);
    lua_pushnumber(L, h);
    return 4;
}

static int worldToWorld(lua_State *L)
{
    BumpWorldWorld *bump = (BumpWorldWorld *)lua_touserdata(L, 1);
    World *world         = bump->world;
    int cx               = luaL_checknumber(L, 2);
    int cy               = luaL_checknumber(L, 3);
    double x, y;
    world->toWorld(cx, cy, x, y);
    lua_pushnumber(L, x);
    lua_pushnumber(L, y);
    return 2;
}

static int worldToCell(lua_State *L)
{
    BumpWorldWorld *bump = (BumpWorldWorld *)lua_touserdata(L, 1);
    World *world         = bump->world;
    double x             = luaL_checknumber(L, 2);
    double y             = luaL_checknumber(L, 3);
    int cx, cy;
    world->toCell(x, y, cx, cy);
    lua_pushnumber(L, cx);
    lua_pushnumber(L, cy);
    return 2;
}

static int worldQueryRect(lua_State *L)
{
    BumpWorldWorld *bump = (BumpWorldWorld *)lua_touserdata(L, 1);
    World *world         = bump->world;
    double x             = luaL_checknumber(L, 2);
    double y             = luaL_checknumber(L, 3);
    double w             = luaL_checknumber(L, 4);
    double h             = luaL_checknumber(L, 5);

    ItemFilter *f = NULL;
    std::set<int> items;
    world->queryRect(x, y, w, h, f, items);
    lua_createtable(L, items.size(), 0);
    int n = 0;
    for (std::set<int>::iterator it = items.begin(); it != items.end(); it++) {
        lua_pushnumber(L, *it);
        lua_rawseti(L, -2, ++n);
    }
    lua_remove(L, -2);
    return 1;
}

static int worldQueryPoint(lua_State *L)
{
    BumpWorldWorld *bump = (BumpWorldWorld *)lua_touserdata(L, 1);
    World *world         = bump->world;

    double x      = luaL_checknumber(L, 2);
    double y      = luaL_checknumber(L, 3);
    ItemFilter *f = NULL;
    std::set<int> items;
    world->queryPoint(x, y, f, items);
    lua_createtable(L, items.size(), 0);
    int n = 0;
    for (std::set<int>::iterator it = items.begin(); it != items.end(); it++) {
        lua_pushnumber(L, *it);
        lua_rawseti(L, -2, ++n);
    }
    lua_remove(L, -2);
    return 1;
}

static int worldQuerySegment(lua_State *L)
{
    BumpWorldWorld *bump = (BumpWorldWorld *)lua_touserdata(L, 1);
    World *world         = bump->world;

    double x1     = luaL_checknumber(L, 2);
    double y1     = luaL_checknumber(L, 3);
    double x2     = luaL_checknumber(L, 4);
    double y2     = luaL_checknumber(L, 5);
    ItemFilter *f = NULL;
    std::set<int> items;
    world->querySegment(x1, y1, x2, y2, f, items);
    lua_createtable(L, items.size(), 0);
    int n = 0;
    for (std::set<int>::iterator it = items.begin(); it != items.end(); it++) {
        lua_pushnumber(L, *it);
        lua_rawseti(L, -2, ++n);
    }
    lua_remove(L, -2);
    return 1;
}

// static int worldQuerySegmentWithCoords(lua_State *L)
// {
//     BumpWorldWorld *bump = (BumpWorldWorld *)lua_touserdata(L, 1);
//     World *world         = bump->world;

//     double x1     = luaL_checknumber(L, 2);
//     double y1     = luaL_checknumber(L, 3);
//     double x2     = luaL_checknumber(L, 4);
//     double y2     = luaL_checknumber(L, 5);
//     ItemFilter *f = NULL;
//     std::vector<ItemInfo> items;
//     world->querySegmentWithCoords(x1, y1, x2, y2, f, items);
//     lua_createtable(L, items.size(), 0);
//     int n = 0;
//     for (std::vector<ItemInfo>::iterator it = items.begin(); it != items.end();
//          it++) {
//         lua_createtable(L, 0, 7);
//         lua_pushnumber(L, (*it).item);
//         lua_setfield(L, -2, "item");
//         lua_pushnumber(L, (*it).ti1);
//         lua_setfield(L, -2, "ti1");
//         lua_pushnumber(L, (*it).ti2);
//         lua_setfield(L, -2, "ti2");
//         lua_pushnumber(L, (*it).x1);
//         lua_setfield(L, -2, "x1");
//         lua_pushnumber(L, (*it).y1);
//         lua_setfield(L, -2, "y1");
//         lua_pushnumber(L, (*it).x2);
//         lua_setfield(L, -2, "x2");
//         lua_pushnumber(L, (*it).y2);
//         lua_setfield(L, -2, "y2");

//         lua_rawseti(L, -2, ++n);
//     }
//     lua_remove(L, -2);
//     return 1;
// }

static int worldAdd(lua_State *L)
{
    assertIsRect(L, 2, 3, 4, 5);

    BumpWorldWorld *bump = (BumpWorldWorld *)lua_touserdata(L, 1);
    World *world         = bump->world;

    double x = luaL_checknumber(L, 2);
    double y = luaL_checknumber(L, 3);
    double w = luaL_checknumber(L, 4);
    double h = luaL_checknumber(L, 5);

    int item = world->allocateId();
    world->add(item, x, y, w, h);

    lua_pushnumber(L, item);
    return 1;
}

static int worldRemove(lua_State *L)
{
    BumpWorldWorld *bump = (BumpWorldWorld *)lua_touserdata(L, 1);
    World *world         = bump->world;
    int item             = lua_tonumber(L, 2);
    world->remove(item);
    return 0;
}

static int worldClear(lua_State *L)
{
    BumpWorldWorld *bump = (BumpWorldWorld *)lua_touserdata(L, 1);
    World *world         = bump->world;
    world->clear();
    return 0;
}

static int worldUpdate(lua_State *L)
{
    assertIsRect(L, 3, 4, 5, 6);
    BumpWorldWorld *bump = (BumpWorldWorld *)lua_touserdata(L, 1);
    World *world         = bump->world;
    int item             = lua_tonumber(L, 2);
    double x             = luaL_checknumber(L, 3);
    double y             = luaL_checknumber(L, 4);
    double w             = luaL_checknumber(L, 5);
    double h             = luaL_checknumber(L, 6);
    world->update(item, x, y, w, h);
    return 0;
}

static int worldMove(lua_State *L)
{
    BumpWorldWorld *bump = (BumpWorldWorld *)lua_touserdata(L, 1);
    World *world         = bump->world;
    int item             = lua_tonumber(L, 2);
    double x             = luaL_checknumber(L, 3);
    double y             = luaL_checknumber(L, 4);
    ColFilter *filter    = world->getFilterById(luaL_optinteger(L, 5, Slide));

    double ax, ay;
    std::vector<Collision> items;
    world->move(item, x, y, filter, ax, ay, items);
    int n = 0;
    lua_createtable(L, items.size(), 0);
    for (std::vector<Collision>::iterator it = items.begin(); it != items.end();
         it++) {
        lua_createtable(L, 0, 10);
        lua_pushnumber(L, (*it).item);
        lua_setfield(L, -2, "item");
        lua_pushnumber(L, (*it).other);
        lua_setfield(L, -2, "other");
        lua_pushnumber(L, (*it).type);
        lua_setfield(L, -2, "type");
        lua_pushboolean(L, (*it).overlaps);
        lua_setfield(L, -2, "overlaps");
        lua_pushnumber(L, (*it).ti);
        lua_setfield(L, -2, "ti");

        lua_createtable(L, 0, 2);
        lua_pushnumber(L, (*it).move.x);
        lua_setfield(L, -2, "x");
        lua_pushnumber(L, (*it).move.y);
        lua_setfield(L, -2, "y");
        lua_setfield(L, -2, "move");
        lua_createtable(L, 0, 2);
        lua_pushnumber(L, (*it).normal.x);
        lua_setfield(L, -2, "x");
        lua_pushnumber(L, (*it).normal.y);
        lua_setfield(L, -2, "y");
        lua_setfield(L, -2, "normal");
        lua_createtable(L, 0, 2);
        lua_pushnumber(L, (*it).touch.x);
        lua_setfield(L, -2, "x");
        lua_pushnumber(L, (*it).touch.y);
        lua_setfield(L, -2, "y");
        lua_setfield(L, -2, "touch");

        if (((*it).type == Bounce) || ((*it).type == Slide)) {
            lua_createtable(L, 0, 2);
            lua_pushnumber(L, (*it).response.x);
            lua_setfield(L, -2, "x");
            lua_pushnumber(L, (*it).response.y);
            lua_setfield(L, -2, "y");
            lua_pushnumber(L, (*it).type);
            lua_setfield(L, -2, "type");
        }

        lua_createtable(L, 0, 4);
        lua_pushnumber(L, (*it).itemRect.x);
        lua_setfield(L, -2, "x");
        lua_pushnumber(L, (*it).itemRect.y);
        lua_setfield(L, -2, "y");
        lua_pushnumber(L, (*it).itemRect.w);
        lua_setfield(L, -2, "w");
        lua_pushnumber(L, (*it).itemRect.h);
        lua_setfield(L, -2, "h");
        lua_setfield(L, -2, "itemRect");
        lua_createtable(L, 0, 4);
        lua_pushnumber(L, (*it).otherRect.x);
        lua_setfield(L, -2, "x");
        lua_pushnumber(L, (*it).otherRect.y);
        lua_setfield(L, -2, "y");
        lua_pushnumber(L, (*it).otherRect.w);
        lua_setfield(L, -2, "w");
        lua_pushnumber(L, (*it).otherRect.h);
        lua_setfield(L, -2, "h");
        lua_setfield(L, -2, "otherRect");

        lua_rawseti(L, -2, ++n);
    }
    lua_pushnumber(L, ax);
    lua_pushnumber(L, ay);
    lua_pushvalue(L, -3);
    lua_remove(L, -4);
    lua_pushinteger(L, n);

    return 4;
}

static int worldCellSize(lua_State *L)
{
    BumpWorldWorld *bump = (BumpWorldWorld *)lua_touserdata(L, 1);
    World *world         = bump->world;
    lua_pushnumber(L, world->cellSize);
    return 1;
}

static int bumpWorldRelease(lua_State *L)
{
    BumpWorldWorld *bump = (BumpWorldWorld *)lua_touserdata(L, 1);
    if (NULL == bump) {
        return luaL_argerror(L, 1, "invalid lua-bump pointer");
    }

    World *world = bump->world;

    Response *responseTouch = world->getResponseById(Touch);
    delete responseTouch;

    Response *responseCross = world->getResponseById(Cross);
    delete responseCross;

    Response *responseSlide = world->getResponseById(Slide);
    delete responseSlide;

    Response *responseBounce = world->getResponseById(Bounce);
    delete responseBounce;

    world->responses.erase(Touch);
    world->responses.erase(Cross);
    world->responses.erase(Bounce);
    world->responses.erase(Slide);


    ColFilter *filterTouch = world->getFilterById(Touch);
    delete filterTouch;

    ColFilter *filterCross = world->getFilterById(Cross);
    delete filterCross;

    ColFilter *filterSlide = world->getFilterById(Slide);
    delete filterSlide;

    ColFilter *filterBounce = world->getFilterById(Bounce);
    delete filterBounce;

    world->filters.erase(Touch);
    world->filters.erase(Cross);
    world->filters.erase(Bounce);
    world->filters.erase(Slide);

    world->clear();

    delete world;
    bump->world = NULL;
    return 0;
}

static int bumpNewWorld(lua_State *L)
{
    if (!lua_isnoneornil(L, 1)) {
        assertIsPositiveNumber(L, 1, "cellSize");
    }
    int cs = luaL_optinteger(L, 1, 64);

    BumpWorldWorld *bump =
        (BumpWorldWorld *)lua_newuserdatauv(L, sizeof(World), 0);
    World *world = new World(cs);
    bump->world  = world;

    CrossFilter *filterCross   = new CrossFilter();
    TouchFilter *filterTouch   = new TouchFilter();
    SlideFilter *filterSlide   = new SlideFilter();
    BounceFilter *filterBounce = new BounceFilter();

    world->addFilter(Touch, filterTouch);
    world->addFilter(Cross, filterCross);
    world->addFilter(Slide, filterSlide);
    world->addFilter(Bounce, filterBounce);

    CrossResponse *responseCross   = new CrossResponse();
    TouchResponse *responseTouch   = new TouchResponse();
    SlideResponse *responseSlide   = new SlideResponse();
    BounceResponse *responseBounce = new BounceResponse();

    world->addResponse(Touch, responseTouch);
    world->addResponse(Cross, responseCross);
    world->addResponse(Slide, responseSlide);
    world->addResponse(Bounce, responseBounce);

    if (luaL_newmetatable(L, METANAME)) // mt
    {
        luaL_Reg l[] = {
            {"project",                worldProject               },
            {"countCells",             worldCountCells            },
            {"hasItem",                worldHasItem               },
            {"countItems",             worldCountItems            },
            {"getRect",                worldGetRect               },
            {"toWorld",                worldToWorld               },
            {"toCell",                 worldToCell                },
            {"queryRect",              worldQueryRect             },
            {"queryPoint",             worldQueryPoint            },
            {"querySegment",           worldQuerySegment          },
            // {"querySegmentWithCoords", worldQuerySegmentWithCoords},
            {"add",                    worldAdd                   },
            {"remove",                 worldRemove                },
            {"update",                 worldUpdate                },
            {"move",                   worldMove                  },
            {"cellSize",               worldCellSize              },
            {"clear",                  worldClear                 },
            {NULL,                     NULL                       }
        };
        luaL_newlib(L, l);              //{}
        lua_setfield(L, -2, "__index"); // mt[__index] = {}
        lua_pushcfunction(L, bumpWorldRelease);
        lua_setfield(L, -2, "__gc"); // mt[__gc] = bumpWorldRelease
    }
    lua_setmetatable(L, -2); // set userdata metatable
    return 1;
}

static int rectGetNearestCorner(lua_State *L)
{
    double x1 = luaL_checknumber(L, 1);
    double y1 = luaL_checknumber(L, 2);
    double w1 = luaL_checknumber(L, 3);
    double h1 = luaL_checknumber(L, 4);
    double px = luaL_checknumber(L, 5);
    double py = luaL_checknumber(L, 6);
    double nx, ny;
    rect_getNearestCorner(x1, y1, w1, h1, px, py, nx, ny);
    lua_pushnumber(L, nx);
    lua_pushnumber(L, ny);
    return 2;
}

static int rectGetSegmentIntersectionIndices(lua_State *L)
{
    double x1  = luaL_checknumber(L, 1);
    double y1  = luaL_checknumber(L, 2);
    double w1  = luaL_checknumber(L, 3);
    double h1  = luaL_checknumber(L, 4);
    double x2  = luaL_checknumber(L, 5);
    double y2  = luaL_checknumber(L, 6);
    double w2  = luaL_checknumber(L, 7);
    double h2  = luaL_checknumber(L, 8);
    double ti1 = 0, ti2 = 1, nx1, ny1, nx2, ny2;
    rect_getSegmentIntersectionIndices(x1, y1, w1, h1, x2, y2, w2, h2, ti1, ti2,
                                       nx1, ny1, nx2, ny2);
    lua_pushnumber(L, ti1);
    lua_pushnumber(L, ti2);
    lua_pushnumber(L, nx1);
    lua_pushnumber(L, ny1);
    lua_pushnumber(L, nx2);
    lua_pushnumber(L, ny2);
    return 6;
}

static int rectGetDiff(lua_State *L)
{
    double x1 = luaL_checknumber(L, 1);
    double y1 = luaL_checknumber(L, 2);
    double w1 = luaL_checknumber(L, 3);
    double h1 = luaL_checknumber(L, 4);
    double x2 = luaL_checknumber(L, 5);
    double y2 = luaL_checknumber(L, 6);
    double w2 = luaL_checknumber(L, 7);
    double h2 = luaL_checknumber(L, 8);

    double x, y, w, h;

    rect_getDiff(x1, y1, w1, h1, x2, y2, w2, h2, x, y, w, h);

    lua_pushnumber(L, x);
    lua_pushnumber(L, y);
    lua_pushnumber(L, w);
    lua_pushnumber(L, h);
    return 4;
}

static int rectContainsPoint(lua_State *L)
{
    double x1 = luaL_checknumber(L, 1);
    double y1 = luaL_checknumber(L, 2);
    double w1 = luaL_checknumber(L, 3);
    double h1 = luaL_checknumber(L, 4);
    double x2 = luaL_checknumber(L, 5);
    double y2 = luaL_checknumber(L, 6);
    lua_pushboolean(L, rect_containsPoint(x1, y1, w1, h1, x2, y2));
    return 1;
}

static int rectContainsRect(lua_State *L)
{
    double x1 = luaL_checknumber(L, 1);
    double y1 = luaL_checknumber(L, 2);
    double w1 = luaL_checknumber(L, 3);
    double h1 = luaL_checknumber(L, 4);
    double x2 = luaL_checknumber(L, 5);
    double y2 = luaL_checknumber(L, 6);
    double w2 = luaL_checknumber(L, 7);
    double h2 = luaL_checknumber(L, 8);
    lua_pushboolean(L, rect_containsRect(x1, y1, w1, h1, x2, y2, w2, h2));
    return 1;
}

static int rectIsIntersecting(lua_State *L)
{
    double x1 = luaL_checknumber(L, 1);
    double y1 = luaL_checknumber(L, 2);
    double w1 = luaL_checknumber(L, 3);
    double h1 = luaL_checknumber(L, 4);
    double x2 = luaL_checknumber(L, 5);
    double y2 = luaL_checknumber(L, 6);
    double w2 = luaL_checknumber(L, 7);
    double h2 = luaL_checknumber(L, 8);
    lua_pushboolean(L, rect_isIntersecting(x1, y1, w1, h1, x2, y2, w2, h2));
    return 1;
}

static int rectGetSquareDistance(lua_State *L)
{
    double x1 = luaL_checknumber(L, 1);
    double y1 = luaL_checknumber(L, 2);
    double w1 = luaL_checknumber(L, 3);
    double h1 = luaL_checknumber(L, 4);
    double x2 = luaL_checknumber(L, 5);
    double y2 = luaL_checknumber(L, 6);
    double w2 = luaL_checknumber(L, 7);
    double h2 = luaL_checknumber(L, 8);
    lua_pushnumber(L, rect_getSquareDistance(x1, y1, w1, h1, x2, y2, w2, h2));
    return 1;
}

static int rectDetectCollision(lua_State *L)
{
    double x1 = luaL_checknumber(L, 1);
    double y1 = luaL_checknumber(L, 2);
    double w1 = luaL_checknumber(L, 3);
    double h1 = luaL_checknumber(L, 4);
    double x2 = luaL_checknumber(L, 5);
    double y2 = luaL_checknumber(L, 6);
    double w2 = luaL_checknumber(L, 7);
    double h2 = luaL_checknumber(L, 8);
    double gx = luaL_checknumber(L, 9);
    double gy = luaL_checknumber(L, 10);
    Collision col;
    if (rect_detectCollision(x1, y1, w1, h1, x2, y2, w2, h2, gx, gy, col)) {
        lua_newtable(L);
        lua_pushboolean(L, col.overlaps);
        lua_setfield(L, -2, "overlaps");
        lua_pushnumber(L, col.ti);
        lua_setfield(L, -2, "ti");

        lua_newtable(L);
        lua_pushnumber(L, col.move.x);
        lua_setfield(L, -2, "x");
        lua_pushnumber(L, col.move.y);
        lua_setfield(L, -2, "y");
        lua_setfield(L, -2, "move");
        lua_newtable(L);
        lua_pushnumber(L, col.normal.x);
        lua_setfield(L, -2, "x");
        lua_pushnumber(L, col.normal.y);
        lua_setfield(L, -2, "y");
        lua_setfield(L, -2, "normal");
        lua_newtable(L);
        lua_pushnumber(L, col.touch.x);
        lua_setfield(L, -2, "x");
        lua_pushnumber(L, col.touch.y);
        lua_setfield(L, -2, "y");
        lua_setfield(L, -2, "touch");

        lua_newtable(L);
        lua_pushnumber(L, col.itemRect.x);
        lua_setfield(L, -2, "x");
        lua_pushnumber(L, col.itemRect.y);
        lua_setfield(L, -2, "y");
        lua_pushnumber(L, col.itemRect.w);
        lua_setfield(L, -2, "w");
        lua_pushnumber(L, col.itemRect.h);
        lua_setfield(L, -2, "h");
        lua_setfield(L, -2, "itemRect");
        lua_newtable(L);
        lua_pushnumber(L, col.otherRect.x);
        lua_setfield(L, -2, "x");
        lua_pushnumber(L, col.otherRect.y);
        lua_setfield(L, -2, "y");
        lua_pushnumber(L, col.otherRect.w);
        lua_setfield(L, -2, "w");
        lua_pushnumber(L, col.otherRect.h);
        lua_setfield(L, -2, "h");
        lua_setfield(L, -2, "otherRect");
        return 1;
    }
    return 0;
}

extern "C" {
int LUAMOD_API luaopen_bump(lua_State *L)
{
    const luaL_Reg bumpFuncs[] = {
        {"newWorld", bumpNewWorld},
        {NULL,       NULL        },
    };

    luaL_newlib(L, bumpFuncs);

    const luaL_Reg rectFuncs[] = {
        {"getNearestCorner",              rectGetNearestCorner             },
        {"getSegmentIntersectionIndices", rectGetSegmentIntersectionIndices},
        {"getDiff",                       rectGetDiff                      },
        {"containsPoint",                 rectContainsPoint                },
        {"containsRect",                  rectContainsRect                 },
        {"isIntersecting",                rectIsIntersecting               },
        {"getSquareDistance",             rectGetSquareDistance            },
        {"detectCollision",               rectDetectCollision              },
        {NULL,                            NULL                             },
    };

    luaL_newlib(L, rectFuncs);
    lua_setfield(L, -2, "rect");

    lauxh_pushint2tbl(L, "touch", Touch);
    lauxh_pushint2tbl(L, "cross", Cross);
    lauxh_pushint2tbl(L, "slide", Slide);
    lauxh_pushint2tbl(L, "bounce", Bounce);

    return 1;
}
}
