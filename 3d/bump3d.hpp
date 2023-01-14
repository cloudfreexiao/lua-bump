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
    int other;
    int type;
    Point move;
    Point normal;
    Point touch;
    Point response;
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
    std::map<int, std::map<int, std::map<int, Cell> > > cells;

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
        std::map<int, std::map<int, std::map<int, Cell> > >::iterator plane =
            cells.find(cz);
        if (plane == cells.end()) {
            return false;
        }
        std::map<int, std::map<int, Cell> >::iterator row =
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

    void getDictItemsInCellCube(int cx, int cy, int cz, int cw, int ch, int cd,
                                std::set<int> &items_dict)
    {
        for (int z = cz; z < cz + cd; z++) {
            std::map<int, std::map<int, std::map<int, Cell> > >::iterator plane =
                cells.find(z);
            if (plane == cells.end()) {
                continue;
            }
            for (int y = cy; y < cy + ch; y++) {
                std::map<int, std::map<int, Cell> >::iterator row =
                    plane->second.find(y);
                if (row == plane->second.end()) {
                    continue;
                }
                for (int x = cx; x < cx + cw; x++) {
                    std::map<int, Cell>::iterator cell = row->second.find(x);
                    if (cell == row->second.end()) {
                        continue;
                    }
                    if (cell->second.items.size() > 0) {
                        for (std::set<int>::iterator it =
                                 cell->second.items.begin();
                             it != cell->second.items.end(); it++) {
                            items_dict.insert(*it);
                        }
                    }
                }
            }
        }
    }

    struct _CellTraversal {
        World *world;
        std::set<Cell *> cells;
    };

    static void cellsTraversal_(void *ctx, int cx, int cy, int cz)
    {
        struct _CellTraversal *ct = (struct _CellTraversal *)ctx;

        std::map<int, std::map<int, std::map<int, Cell> > >::iterator plane =
            ct->world->cells.find(cz);
        if (plane == ct->world->cells.end()) {
            return;
        }
        std::map<int, std::map<int, Cell> >::iterator row =
            plane->second.find(cy);
        if (row == plane->second.end()) {
            return;
        }
        std::map<int, Cell>::iterator cell = row->second.find(cx);
        if (cell == row->second.end())
            return;
        ct->cells.insert(&cell->second);
    }
    std::set<Cell *> getCellsTouchedBySegment(double x1, double y1, double z1,
                                              double x2, double y2, double z2)
    {
        struct _CellTraversal ct;
        ct.world = this;
        grid_traverse(cellSize, x1, y1, z1, x2, y2, z2, cellsTraversal_, &ct);
        return ct.cells;
    }

    void getInfoAboutItemsTouchedBySegment(double x1, double y1, double z1,
                                           double x2, double y2, double z2,
                                           ItemFilter *filter,
                                           std::vector<ItemInfo> &itemInfo)
    {
        std::set<Cell *> cells =
            getCellsTouchedBySegment(x1, y1, z1, x2, y2, z2);

        std::set<int> visited;

        for (std::set<Cell *>::iterator it = cells.begin(); it != cells.end();
             it++) {
            Cell *cell = (*it);
            for (std::set<int>::iterator i = cell->items.begin();
                 i != cell->items.end(); i++) {
                if (visited.find(*i) == visited.end()) {
                    visited.insert(*i);
                    if ((!filter) || filter->Filter(*i)) {
                        Cube c = cubes[*i];
                        double nx1, ny1, nz1, nx2, ny2, nz2;
                        double ti1 = 0;
                        double ti2 = 1;

                        cube_getSegmentIntersectionIndices(
                            c.x, c.y, c.z, c.w, c.h, c.d, x1, y1, z1, x2, y2,
                            z2, ti1, ti2, nx1, ny1, nz1, nx2, ny2, nz2);
                        if (ti1 && (((0 < ti1) && (ti1 < 1)) ||
                                    ((0 < ti2) && (ti2 < 1)))) {
                            // -- the sorting is according to the t of an
                            // infinite line, not the segment
                            double tii0 = -MATH_HUGE;
                            double tii1 = MATH_HUGE;

                            cube_getSegmentIntersectionIndices(
                                c.x, c.y, c.z, c.w, c.h, c.d, x1, y1, z1, x2,
                                y2, z2, tii0, tii1, nx1, ny1, nz1, nx2, ny2,
                                nz2);

                            ItemInfo ii;
                            ii.item   = *i;
                            ii.ti1    = ti1;
                            ii.ti2    = ti2;
                            ii.weight = tii0 < tii1 ? tii0 : tii1;
                            itemInfo.push_back(ii);
                        }
                    }
                }
            }
        }

        std::sort(itemInfo.begin(), itemInfo.end(), sortByWeight);
    }

    Response *getResponseById(int id)
    {
        return responses[id];
    }

    void addResponse(int id, Response *response)
    {
        responses[id] = response;
    }

    void addFilter(int id, ColFilter *filter)
    {
        filters[id] = filter;
    }

    ColFilter *getFilterById(int id)
    {
        return filters[id];
    }

    void getCube(int item, double &x, double &y, double &z, double &w,
                 double &h, double &d)
    {
        Cube c = cubes[item];
        x      = c.x;
        y      = c.y;
        z      = c.z;
        w      = c.w;
        h      = c.h;
        d      = c.d;
    }

    void project(int item, double x, double y, double z, double w, double h,
                 double d, double goalX, double goalY, double goalZ,
                 ColFilter *filter, std::vector<Collision> &collisions)
    {
        std::set<int> visited;
        if (item) {
            visited.insert(item);
        }

        // -- This could probably be done with less cells using a
        //           polygon raster over the cells instead of a
        //   -- bounding cube of the whole movement. Conditional to building a
        //   queryPolygon method

        double tx1 = (goalX < x) ? goalX : x;
        double ty1 = (goalY < y) ? goalY : y;
        double tz1 = (goalY < z) ? goalY : z;

        double tx2 = ((goalX + w) > (x + w)) ? goalX + w : x + w;
        double ty2 = ((goalY + h) > (y + h)) ? goalY + h : y + h;
        double tz2 = ((goalY + d) > (y + d)) ? goalY + d : y + d;

        double tw = tx2 - tx1;
        double th = ty2 - ty1;
        double td = tz2 - tz1;

        int cx, cy, cz, cw, ch, cd;
        grid_toCellCube(cellSize, tx1, ty1, tz1, tw, th, td, cx, cy, cz, cw, ch,
                        cd);

        std::set<int> dictItemsInCellRect;
        getDictItemsInCellCube(cx, cy, cz, cw, ch, cd, dictItemsInCellRect);

        for (std::set<int>::iterator it = dictItemsInCellRect.begin();
             it != dictItemsInCellRect.end(); it++) {
            int other = *it;
            if (visited.find(other) == visited.end()) {
                visited.insert(other);
                int responseId = filter->Filter(item, other);
                if (responseId > 0) {
                    double ox, oy, oz, ow, oh, od;
                    getCube(other, ox, oy, oz, ow, oh, od);
                    Collision col;
                    if (cube_detectCollision(x, y, z, w, h, d, ox, oy, oz, ow,
                                             oh, od, goalX, goalY, goalZ,
                                             col)) {
                        col.other = other;
                        col.item  = item;
                        col.type  = responseId;
                        collisions.push_back(col);
                    }
                }
            }
        }

        std::sort(collisions.begin(), collisions.end(), sortByTiAndDistance);
    }

    void projectMove(int item, double x, double y, double z, double w, double h,
                     double d, double goalX, double goalY, double goalZ,
                     ColFilter *filter, double &actualX, double &actualY,
                     double &actualZ, std::vector<Collision> &cols)
    {
        VisitedFilter vf;
        vf.visited.insert(item);
        vf.filter = filter;

        std::vector<Collision> projected_cols;
        project(item, x, y, z, w, h, d, goalX, goalY, goalZ, &vf,
                projected_cols);

        while (projected_cols.size() > 0) {
            Collision col = projected_cols[0];
            vf.visited.insert(col.other);
            Response *response = getResponseById(col.type);

            projected_cols.clear();
            response->ComputeResponse(this, col, x, y, z, w, h, d, goalX, goalY,
                                      goalZ, &vf, goalX, goalY, goalZ,
                                      projected_cols);

            cols.push_back(col);
        }

        actualX = goalX;
        actualY = goalY;
        actualZ = goalZ;
    }

    bool hasItem(int item)
    {
        return cubes.find(item) != cubes.end();
    }

    void toWorld(int cx, int cy, int cz, double &x, double &y, double &z)
    {
        grid_toWorld(cellSize, cx, cy, cz, x, y, z);
    }

    void toCell(double x, double y, double z, int &cx, int &cy, int &cz)
    {
        grid_toCell(cellSize, x, y, z, cx, cy, cz);
    }

    //--- Query methods

    void queryCube(double x, double y, double z, double w, double h, double d,
                   ItemFilter *filter, std::set<int> &dictItemsInCellCube)
    {
        int cx, cy, cz, cw, ch, cd;
        grid_toCellCube(cellSize, x, y, z, w, h, d, cx, cy, cz, cw, ch, cd);
        getDictItemsInCellCube(cx, cy, cz, cw, ch, cd, dictItemsInCellCube);
        for (std::set<int>::iterator it = dictItemsInCellCube.begin();
             it != dictItemsInCellCube.end();) {
            bool drop = (filter && !filter->Filter(*it));
            if (!drop) {
                Cube cube = cubes[*it];
                drop |= !cube_isIntersecting(x, y, z, w, h, d, cube.x, cube.y,
                                             cube.z, cube.w, cube.h, cube.d);
            }
            if (drop) {
                dictItemsInCellCube.erase(it++);
            } else {
                ++it;
            }
        }
    }

    void queryPoint(double x, double y, double z, ItemFilter *filter,
                    std::set<int> &dictItemsInCellCube)
    {
        int cx, cy, cz;
        toCell(x, y, z, cx, cy, cz);
        getDictItemsInCellCube(cx, cy, cz, 1, 1, 1, dictItemsInCellCube);
        for (std::set<int>::iterator it = dictItemsInCellCube.begin();
             it != dictItemsInCellCube.end();) {
            Cube cube = cubes[*it];

            if ((filter && !filter->Filter(*it)) ||
                !cube_containsPoint(cube.x, cube.y, cube.z, cube.w, cube.h,
                                    cube.d, x, y, z)) {
                dictItemsInCellCube.erase(it++);
            } else {
                ++it;
            }
        }
    }

    void querySegment(double x1, double y1, double z1, double x2, double y2,
                      double z2, ItemFilter *filter, std::set<int> &items)
    {
        std::vector<ItemInfo> itemInfo;
        getInfoAboutItemsTouchedBySegment(x1, y1, z1, x2, y2, z2, filter,
                                          itemInfo);
        for (std::vector<ItemInfo>::iterator it = itemInfo.begin();
             it != itemInfo.end(); it++) {
            items.insert((*it).item);
        }
    }

    void querySegmentWithCoords(double x1, double y1, double z1, double x2,
                                double y2, double z2, ItemFilter *filter,
                                std::vector<ItemInfo> &itemInfo2)
    {
        std::vector<ItemInfo> itemInfo;
        getInfoAboutItemsTouchedBySegment(x1, y1, z1, x2, y2, z2, filter,
                                          itemInfo);
        double dx = x2 - x1, dy = y2 - y1, dz = z2 - z1;
        for (std::vector<ItemInfo>::iterator it = itemInfo.begin();
             it != itemInfo.end(); it++) {
            ItemInfo i = *it;
            i.x1       = x1 + dx * i.ti1;
            i.y1       = y1 + dy * i.ti1;
            i.z1       = z1 + dz * i.ti1;
            i.x2       = x1 + dx * i.ti2;
            i.y2       = y1 + dy * i.ti2;
            i.z2       = z2 + dz * i.ti2;
            itemInfo2.push_back(i);
        }
    }

    //--- Main methods
    int allocateId()
    {
        if (itemId >= INT_MAX) {
            itemId = 0;
        }

        int nid = (++itemId);

        while (hasItem(nid)) {
            nid++;
        }

        itemId = nid;
        return nid;
    }

    void add(int item, double x, double y, double z, double w, double h,
             double d)
    {
        Cube cube;
        cube.x      = x;
        cube.y      = y;
        cube.z      = z;
        cube.w      = w;
        cube.h      = h;
        cube.d      = d;
        cubes[item] = cube;

        int cl, ct, cs, cw, ch, cd;

        grid_toCellCube(cellSize, x, y, z, w, h, d, cl, ct, cs, cw, ch, cd);

        for (int cz = cs; cz < cs + cd; cz++) {
            for (int cy = ct; cy < ct + ch; cy++) {
                for (int cx = cl; cx < cl + cw; cx++) {
                    addItemToCell(item, cx, cy, cz);
                }
            }
        }
    }

    void remove(int item)
    {
        Cube cube = cubes[item];

        int cl, ct, cs, cw, ch, cd;
        grid_toCellCube(cellSize, cube.x, cube.y, cube.z, cube.w, cube.h,
                        cube.d, cl, ct, cs, cw, ch, cd);
        for (int cz = cs; cz < cs + cd; cz++) {
            for (int cy = ct; cy < ct + ch; cy++) {
                for (int cx = cl; cx < cl + cw; cx++) {
                    removeItemFromCell(item, cx, cy, cz);
                }
            }
        }

        cubes.erase(item);
    }

    void clear()
    {
        itemId = 0;
        cubes.clear();
        cells.clear();
    }

    void update(int item, double x2, double y2, double z2, double w2, double h2,
                double d2)
    {
        Cube cube = cubes[item];

        if (w2 <= 0) {
            w2 = cube.w;
        }
        if (h2 <= 0) {
            h2 = cube.h;
        }
        if (d2 <= 0) {
            d2 = cube.d;
        }
        if ((cube.x == x2) && (cube.y == y2) && (cube.w == w2) &&
            (cube.h == h2) && (cube.d == d2)) {
            return;
        }

        int cl1, ct1, cs1, cw1, ch1, cd1;
        grid_toCellCube(cellSize, cube.x, cube.y, cube.z, cube.w, cube.h,
                        cube.d, cl1, ct1, cs1, cw1, ch1, cd1);
        int cl2, ct2, cs2, cw2, ch2, cd2;
        grid_toCellCube(cellSize, x2, y2, z2, w2, h2, d2, cl2, ct2, cs2, cw2,
                        ch2, cd2);

        if ((cl1 != cl2) || (ct1 != ct2) || (cs1 != cs2) || (cw1 != cw2) ||
            (ch1 != ch2) || (cd1 != cd2)) {
            int cr1 = cl1 + cw1 - 1, cb1 = ct1 + ch1 - 1;
            int cr2 = cl2 + cw2 - 1, cb2 = ct2 + ch2 - 1;
            int css1 = cs1 + cd1 - 1, css2 = cs2 + cd2 - 1;

            bool cyOut, czOut;

            for (int cz = cs1; cz <= css1; cz++) {
                czOut = cz < cs2 || cz > css2;

                for (int cy = ct1; cy <= cb1; cy++) {
                    cyOut = (cy < ct2) || (cy > cb2);

                    for (int cx = cl1; cx <= cr1; cx++) {
                        if (czOut || cyOut || (cx < cl2) || (cx > cr2)) {
                            removeItemFromCell(item, cx, cy, cz);
                        }
                    }
                }
            }

            for (int cz = cs2; cz <= css2; cz++) {
                czOut = cz < cs1 || cz > css1;

                for (int cy = ct2; cy <= cb2; cy++) {
                    cyOut = (cy < ct1) || (cy > cb1);

                    for (int cx = cl2; cx <= cr2; cx++) {
                        if (czOut || cyOut || (cx < cl1) || (cx > cr1)) {
                            addItemToCell(item, cx, cy, cz);
                        }
                    }
                }
            }

            Cube c;
            c.x         = x2;
            c.y         = y2;
            c.z         = z2;
            c.w         = w2;
            c.h         = h2;
            c.d         = d2;
            cubes[item] = c;
        }
    }

    void check(int item, double goalX, double goalY, double goalZ,
               ColFilter *filter, double &actualX, double &actualY,
               double &actualZ, std::vector<Collision> &cols)
    {
        Cube cube = cubes[item];
        projectMove(item, cube.x, cube.y, cube.z, cube.w, cube.h, cube.d, goalX,
                    goalY, goalZ, filter, actualX, actualY, actualZ, cols);
    }

    void move(int item, double goalX, double goalY, double goalZ,
              ColFilter *filter, double &actualX, double &actualY,
              double &actualZ, std::vector<Collision> &cols)
    {
        check(item, goalX, goalY, goalZ, filter, actualX, actualY, actualZ,
              cols);
        update(item, actualX, actualY, actualZ, -1, -1, -1);
    }
};

} // namespace bump3d