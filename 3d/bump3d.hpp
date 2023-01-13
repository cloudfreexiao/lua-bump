#pragma once

#include <algorithm>
#include <limits.h>
#include <map>
#include <math.h>
#include <set>
#include <vector>

namespace bump3d
{
#define UNUSED(x) (void)(x)
#define MATH_HUGE HUGE_VAL

#define Touch  1
#define Cross  2
#define Slide  3
#define Bounce 4

#define DELTA   1e-10 // -- floating-point margin of error
#define iabs(a) ((a >= 0) ? a : -a)

static double sign(double x)
{
    return (x > 0) ? 1 : ((x == 0) ? 0 : -1);
}

static double nearest(double x, double a, double b)
{
    return fabs(a - x) < fabs(b - x) ? a : b;
}

/*
------------------------------------------
-- Cube functions
------------------------------------------
*/

static void cube_getNearestCorner(double x, double y, double z, double w,
                                  double h, double d, double px, double py,
                                  double pz, double &nx, double &ny, double &nz)
{
    nx = nearest(px, x, x + w);
    ny = nearest(py, y, y + h);
    nz = nearest(pz, z, z + d);
}

// -- This is a generalized implementation of the liang-barsky algorithm, which
// also returns
// -- the normals of the sides where the segment intersects.
// -- Returns nil if the segment never touches the cube
// -- Notice that normals are only guaranteed to be accurate when initially ti1,
// ti2 == -math.huge, math.huge
static bool cube_getSegmentIntersectionIndices(
    double x, double y, double z, double w, double h, double d, double x1,
    double y1, double z1, double x2, double y2, double z2, double &ti1,
    double &ti2, double &nx1, double &ny1, double &nz1, double &nx2,
    double &ny2, double &nz2)
{
    double dx = x2 - x1;
    double dy = y2 - y1;
    double dz = z2 - z1;
    double nx, ny, nz;
    double p, q, r;

    nx1 = ny1 = nz1 = nx2 = ny2 = nz2 = 0;

    for (int side = 1; side <= 6; side++) {
        switch (side) {
        case 1:
            nx = -1;
            ny = 0;
            nz = 0;
            p  = -dx;
            q  = x1 - x;
            break; // left
        case 2:
            nx = 1;
            ny = 0;
            nz = 0;
            p  = dx;
            q  = x + w - x1;
            break; //-- right
        case 3:
            nx = 0;
            ny = -1;
            nz = 0;
            p  = -dy;
            q  = y1 - y;
            break; //-- top
        case 4:
            nx = 0;
            ny = 1;
            nz = 0;
            p  = dy;
            q  = y + h - y1;
            break; //-- bottom
        case 5:
            nx = 0;
            ny = 0;
            nz = -1;
            p  = -dz;
            q  = z1 - z;
            break; // -- front
        case 6:
            nx = 0;
            ny = 0;
            nz = 1;
            p  = dz;
            q  = z + d - z1;
            break; // -- back
        }
    }

    if (p == 0) {
        if (q <= 0) {
            return false;
        }
    } else {
        r = q / p;
        if (p < 0) {
            if (r > ti2) {
                return false;
            }
            if (r > ti1) {
                ti1 = r;
                nx1 = nx;
                ny1 = ny;
                nz1 = nz;
            }
        } else { //-- p > 0
            if (r < ti1) {
                return false;
            }
            if (r < ti2) {
                ti2 = r;
                nx2 = nx;
                ny2 = ny;
                nz2 = nz;
            }
        }
    }
    return true;
}

// -- Calculates the minkowsky difference between 2 cubes, which is another cube
static void cube_getDiff(double x1, double y1, double z1, double w1, double h1,
                         double d1, double x2, double y2, double z2, double w2,
                         double h2, double d2, double &rx, double &ry,
                         double &rz, double &rw, double &rh, double &rd)
{
    rx = x2 - x1 - w1;
    ry = y2 - y1 - h1;
    rz = z2 - z1 - d1;
    rw = w1 + w2;
    rh = h1 + h2;
    rd = d1 + d2;
}

static bool cube_containsPoint(double x, double y, double z, double w, double h,
                               double d, double px, double py, double pz)
{
    return ((px - x) > DELTA) && ((py - y) > DELTA) && ((pz - z) > DELTA) &&
           ((x + w - px) > DELTA) && ((y + h - py) > DELTA) &&
           ((z + d - pz) > DELTA);
}

static bool cube_isIntersecting(double x1, double y1, double z1, double w1,
                                double h1, double d1, double x2, double y2,
                                double z2, double w2, double h2, double d2)
{
    return (x1 < x2 + w2) && (x2 < x1 + w1) && (y1 < y2 + h2) &&
           (y2 < y1 + h1) && (z1 < z2 + d2) && (z2 < z1 + d1);
}

static double cube_getCubeDistance(double x1, double y1, double z1, double w1,
                                   double h1, double d1, double x2, double y2,
                                   double z2, double w2, double h2, double d2)
{
    double dx = x1 - x2 + (w1 - w2) / 2;
    double dy = y1 - y2 + (h1 - h2) / 2;
    double dz = z1 - z2 + (d1 - d2) / 2;
    return (dx * dx) + (dy * dy) + (dz * dz);
}

struct Point {
    double x, y, z;
};

struct Cube {
    double x, y, z, w, h, d;
};

struct Collision {
    int item;
    bool overlaps;
    double ti;
    double distance;
    Point move;
    Point normal;
    Point touch;
    // int other;
    // int type;
    // Point response;
};

static bool cube_detectCollision(double x1, double y1, double z1, double w1,
                                 double h1, double d1, double x2, double y2,
                                 double z2, double w2, double h2, double d2,
                                 double goalX, double goalY, double goalZ,
                                 Collision &col)
{
    double dx = goalX - x1, dy = goalY - y1, dz = goalZ - z1;

    double x, y, z, w, h, d;
    cube_getDiff(x1, y1, z1, w1, h1, d1, x2, y2, z2, w2, h2, d2, x, y, z, w, h,
                 d);

    bool overlaps = false;
    double ti;
    bool cf   = false;
    double nx = 0, ny = 0, nz = 0;

    if (cube_containsPoint(x, y, z, w, h, d, 0, 0, d)) {
        // -- item was intersecting other
        double px, py, pz;
        cube_getNearestCorner(x, y, z, w, h, d, 0, 0, 0, px, py, pz);
        // -- Volume of intersection:
        double wi = (w1 < fabs(px)) ? w1 : fabs(px);
        double hi = (h1 < fabs(py)) ? h1 : fabs(py);
        double di = (d1 < fabs(pz)) ? h1 : fabs(pz);
        ti = wi * hi * di * -1; // -- ti is the negative volume of intersection
        overlaps = true;
        cf       = true;
    } else {
        double ti1 = -MATH_HUGE, ti2 = MATH_HUGE;
        double nx1, ny1, nz1, nx2, ny2, nz2;
        if (cube_getSegmentIntersectionIndices(x, y, z, w, h, d, 0, 0, 0, dx,
                                               dy, dz, ti1, ti2, nx1, ny1, nz1,
                                               nx2, ny2, nz2)) {
            //-- item tunnels into other
            if ((ti1 < 1) && (fabs(ti1 - ti2) >= DELTA) &&
                ((0 < (ti1 + DELTA)) || ((0 == ti1) && (ti2 > 0)))) {
                //-- special case for rect going
                // through another rect's corner
                ti       = ti1;
                nx       = nx1;
                ny       = ny1;
                nz       = nz1;
                overlaps = false;
                cf       = true;
            }
        }
    }

    if (!cf) {
        return false;
    }

    double tx, ty, tz;
    if (overlaps) {
        if ((dx == 0) && (dy == 0) && (dz == 0)) {
            //  -- intersecting and not moving - use minimum displacement vector
            double px, py, pz;
            cube_getNearestCorner(x, y, z, w, h, d, 0, 0, 0, px, py, pz);
            if (fabs(px) <= fabs(py) && fabs(px) <= fabs(pz)) {
                // -- X axis has minimum displacement
                py = 0;
                pz = 0;
            } else if (fabs(py) <= fabs(pz)) {
                // -- Y axis has minimum displacement
                px = 0;
                pz = 0;
            } else {
                //         -- Z axis has minimum displacement
                px = 0;
                py = 0;
            }

            nx = sign(px);
            ny = sign(py);
            nz = sign(pz);

            tx = x1 + px;
            ty = y1 + py;
            tz = z1 + pz;
        } else {
            // -- intersecting and moving - move in the opposite direction
            double ti1 = -MATH_HUGE;
            double ti2 = 1;
            double nx1, ny1, nz1, nx2, ny2, nz2;
            if (!cube_getSegmentIntersectionIndices(x, y, z, w, h, d, 0, 0, 0,
                                                    dx, dy, dz, ti1, ti2, nx1,
                                                    ny1, nz1, nx2, ny2, nz2)) {
                return false;
            }
            tx = x1 + dx * ti1;
            ty = y1 + dy * ti1;
            tz = z1 + dz * ti1;
        }
    } else {
        // -- tunnel
        tx = x1 + dx * ti;
        ty = y1 + dy * ti;
        tz = z1 + dz * ti;
    }

    col.overlaps = overlaps;
    col.ti       = ti;
    col.move.x   = dx;
    col.move.y   = dy;
    col.move.z   = dz;

    col.normal.x = nx;
    col.normal.y = ny;
    col.normal.z = nz;

    col.touch.x = tx;
    col.touch.y = ty;
    col.touch.z = tz;

    col.distance =
        cube_getCubeDistance(x1, y1, z1, w1, h1, d1, x2, y2, z2, w2, h2, d2);

    return true;
}

//------------------------------------------
//-- Grid functions
//------------------------------------------

static void grid_toWorld(int cellSize, int cx, int cy, int cz, double &wx,
                         double &wy, double &wz)
{
    wx = (cx - 1) * cellSize;
    wy = (cy - 1) * cellSize;
    wz = (cz - 1) * cellSize;
}

static void grid_toCell(int cellSize, double x, double y, double z, int &cx,
                        int &cy, int &cz)
{
    cx = floor(x / cellSize) + 1;
    cy = floor(y / cellSize) + 1;
    cz = floor(z / cellSize) + 1;
}

// -- grid_traverse* functions are based on "A Fast Voxel Traversal Algorithm
// for Ray Tracing",
// -- by John Amanides and Andrew Woo -
// http://www.cse.yorku.ca/~amana/research/grid.pdf
// -- It has been modified to include both cells when the ray "touches a grid
// corner",
// -- and with a different exit condition
static int grid_traverse_initStep(int cellSize, int ct, double t1, double t2,
                                  double &rx, double &ry)
{
    double v = t2 - t1;
    if (v > 0) {
        rx = cellSize / v;
        ry = ((ct + v) * cellSize - t1) / v;
        return 1;
    }
    if (v < 0) {
        rx = -cellSize / v;
        ry = ((ct + v - 1) * cellSize - t1) / v;
        return -1;
    }
    rx = HUGE_VAL;
    ry = HUGE_VAL;
    return 0;
}

typedef void (*pointFunc)(void *data, int, int, int);

static void grid_traverse(int cellSize, double x1, double y1, double z1,
                          double x2, double y2, double z2, pointFunc f,
                          void *data)
{
    int cx1, cy1, cz1, cx2, cy2, cz2;
    double dx, tx, dy, ty, dz, tz;

    grid_toCell(cellSize, x1, y1, z1, cx1, cy1, cz1);
    grid_toCell(cellSize, x2, y2, z2, cx2, cy2, cz2);

    int stepX = grid_traverse_initStep(cellSize, cx1, x1, x2, dx, tx);
    int stepY = grid_traverse_initStep(cellSize, cy1, y1, y2, dy, ty);
    int stepZ = grid_traverse_initStep(cellSize, cy1, z1, z2, dz, tz);

    int cx = cx1;
    int cy = cy1;
    int cz = cz1;

    f(data, cx, cy, cz);

    //-- The default implementation had an infinite loop problem when
    //-- approaching the last cell in some occassions. We finish iterating
    //-- when we are *next* to the last cell
    while ((iabs(cx - cx2) + iabs(cy - cy2) + abs(cz - cz2)) > 1) {
        if (tx < ty && tx < tz) { //  -- tx is smallest
            tx += dx;
            cx += stepX;
            f(data, cx, cy, cz);
        } else if (ty < tz) { //  -- ty is smallest
            // -- Addition: include both cells when going through corners
            if (tx == ty) {
                f(data, cx + stepX, cy, cz);
            }
            ty = ty + dy;
            cy = cy + stepY;
            f(data, cx, cy, cz);
        } else {
            //  -- tz is smallest
            // -- Addition: include both cells when going through corners
            if (tx == tz) {
                f(data, cx + stepX, cy, cz);
            }
            if (ty == tz) {
                f(data, cx, cy + stepY, cz);
            }
            tz = tz + dz;
            cz = cz + stepZ;
            f(data, cx, cy, cz);
        }
    }

    //-- If we have not arrived to the last cell, use it
    if ((cx != cx2) || (cy != cy2) || (cz != cz2)) {
        f(data, cx2, cy2, cz2);
    }
}

static void grid_toCellCube(int cellSize, double x, double y, double z,
                            double w, double h, double d, int &cx, int &cy,
                            int &cz, int &cw, int &ch, int &cd)
{
    grid_toCell(cellSize, x, y, z, cx, cy, cz);
    int cx2 = ceil((x + w) / cellSize);
    int cy2 = ceil((y + h) / cellSize);
    int cz2 = ceil((z + d) / cellSize);
    cw      = cx2 - cx + 1;
    ch      = cy2 - cy + 1;
    cd      = cz2 - cz + 1;
}

struct World;

/*------------------------------------------
-- ColFilter
------------------------------------------*/

struct ColFilter {
    virtual int Filter(int item, int other) = 0;
    virtual ~ColFilter(){};
};

struct VisitedFilter : ColFilter {
    std::set<int> visited;
    ColFilter *filter;
    int Filter(int item, int other)
    {
        if (visited.find(other) != visited.end()) {
            return 0;
        }
        return filter->Filter(item, other);
    }
};

/*------------------------------------------
-- Responses
------------------------------------------*/

struct Response {
    virtual void ComputeResponse(World *world, Collision &col, double x,
                                 double y, double z, double w, double h,
                                 double d, double goalX, double goalY,
                                 double goalZ, ColFilter *filter,
                                 double &actualX, double &actualY,
                                 double &actualZ,
                                 std::vector<Collision> &cols) = 0;
    virtual ~Response(){};
};

/*------------------------------------------
-- World
------------------------------------------*/

struct Cell {
    std::set<int> items;
    int x, y, z;
    Cell() : x(0), y(0), z(0) {}
};

struct ItemInfo {
    int item;
    double ti1, ti2, weight;
    double x1, y1, z1, x2, y2, z2;
};

struct ItemFilter {
    virtual bool Filter(int item) = 0;
    virtual ~ItemFilter(){};
};

struct World {
    int cellSize;
    int itemId;
    std::map<int, Response *> responses;
    std::map<int, ColFilter *> filters;

    std::map<int, Cube> cubes;
    std::map<int, std::map<int, std::map<int, Cell>>> cells;

    World(int cs)
    {
        cellSize = cs;
        itemId   = 0;
    }

    static bool sortByWeight(ItemInfo a, ItemInfo b)
    {
        return a.weight < b.weight;
    }

    static bool sortByTiAndDistance(Collision a, Collision b)
    {
        if (a.ti == b.ti) {
            return a.distance < b.distance;
        }
        return a.ti < b.ti;
    }

    void addItemToCell(int item, int cx, int cy, int cz)
    {
        cells[cz][cy][cx].items.insert(item);
    }

    bool removeItemFromCell(int item, int cx, int cy, int cz)
    {
        std::map<int, std::map<int, std::map<int, Cell>>>::iterator plane =
            cells.find(cz);
        if (plane == cells.end()) {
            return false;
        }
        std::map<int, std::map<int, Cell>>::iterator row =
            plane->second.find(cy);
        if (row == plane->second.end()) {
            return false;
        }
        std::map<int, Cell>::iterator cell = row->second.find(cx);
        if (cell == row->second.end()) {
            return false;
        }
        if (cell->second.items.find(item) == cell->second.items.end()) {
            return false;
        }
        cell->second.items.erase(item);
        return true;
    }

   void getDictItemsInCellCube(int cx, int cy, int cz, int cw,
                                            int ch, int cd, std::set<int> &items_dict)
    {
    }
};

} // namespace bump3d