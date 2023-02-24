#pragma once

#include <algorithm>
#include <limits.h>
#include <map>
#include <math.h>
#include <set>
#include <vector>

namespace bump2d
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
-- Rectangle functions
------------------------------------------
 */

static void rect_getNearestCorner(double x, double y, double w, double h,
                                  double px, double py, double &nx, double &ny)
{
    nx = nearest(px, x, x + w);
    ny = nearest(py, y, y + h);
}

/*-- This is a generalized implementation of the liang-barsky algorithm, which
 also returns
 -- the normals of the sides where the segment intersects.
 -- Returns nil if the segment never touches the rect
 -- Notice that normals are only guaranteed to be accurate when initially ti1,
 ti2 == -math.huge, math.huge
 */
static bool rect_getSegmentIntersectionIndices(double x, double y, double w,
                                               double h, double x1, double y1,
                                               double x2, double y2,
                                               double &ti1, double &ti2,
                                               double &nx1, double &ny1,
                                               double &nx2, double &ny2)
{
    double dx = x2 - x1;
    double dy = y2 - y1;
    double nx, ny;
    double p, q, r;
    nx1 = nx2 = ny1 = ny2 = 0;

    for (int side = 1; side <= 4; side++) {
        switch (side) {
        case 1:
            nx = -1;
            ny = 0;
            p  = -dx;
            q  = x1 - x;
            break; //-- left
        case 2:
            nx = 1;
            ny = 0;
            p  = dx;
            q  = x + w - x1;
            break; //-- right
        case 3:
            nx = 0;
            ny = -1;
            p  = -dy;
            q  = y1 - y;
            break; //-- top
        case 4:
            nx = 0;
            ny = 1;
            p  = dy;
            q  = y + h - y1;
            break; //-- bottom
        }
        if (p == 0) {
            if (q <= 0)
                return false;
        } else {
            r = q / p;
            if (p < 0) {
                if (r > ti2)
                    return false;
                if (r > ti1) {
                    ti1 = r;
                    nx1 = nx;
                    ny1 = ny;
                }
            } else { //-- p > 0
                if (r < ti1)
                    return false;
                if (r < ti2) {
                    ti2 = r;
                    nx2 = nx;
                    ny2 = ny;
                }
            }
        }
    }
    return true;
}

//-- Calculates the minkowsky difference between 2 rects, which is another rect
static void rect_getDiff(double x1, double y1, double w1, double h1, double x2,
                         double y2, double w2, double h2, double &rx,
                         double &ry, double &rw, double &rh)
{
    rx = x2 - x1 - w1;
    ry = y2 - y1 - h1;
    rw = w1 + w2;
    rh = h1 + h2;
}

static bool rect_containsPoint(double x, double y, double w, double h,
                               double px, double py)
{
    return ((px - x) > DELTA) && ((py - y) > DELTA) && ((x + w - px) > DELTA) &&
           ((y + h - py) > DELTA);
}

static bool rect_containsRect(double x1, double y1, double w1, double h1,
                              double x2, double y2, double w2, double h2)
{
    return x1 <= x2 && y1 >= y2 && x1 + w1 <= x2 + w2 && y1 + h1 <= y2 + h2;
}

static bool rect_isIntersecting(double x1, double y1, double w1, double h1,
                                double x2, double y2, double w2, double h2)
{
    return (x1 < (x2 + w2)) && (x2 < (x1 + w1)) && (y1 < (y2 + h2)) &&
           (y2 < (y1 + h1));
}

static double rect_getSquareDistance(double x1, double y1, double w1, double h1,
                                     double x2, double y2, double w2, double h2)
{
    double dx = x1 - x2 + (w1 - w2) / 2;
    double dy = y1 - y2 + (h1 - h2) / 2;
    return dx * dx + dy * dy;
}

struct Point {
    double x, y;
};
struct Rect {
    double x, y, w, h;
};
struct Collision {
    bool overlaps;
    int item;
    int other;
    int type;
    double ti;
    Point move;
    Point normal;
    Point touch;
    Point response;
    Rect itemRect;
    Rect otherRect;
};

static bool rect_detectCollision(double x1, double y1, double w1, double h1,
                                 double x2, double y2, double w2, double h2,
                                 double goalX, double goalY, Collision &col)
{
    double dx = goalX - x1, dy = goalY - y1;
    double x, y, w, h;
    rect_getDiff(x1, y1, w1, h1, x2, y2, w2, h2, x, y, w, h);

    bool overlaps = false;
    double ti;
    bool cf   = false;
    double nx = 0, ny = 0;

    if (rect_containsPoint(x, y, w, h, 0, 0)) { //-- item was intersecting other
        double px, py;
        rect_getNearestCorner(x, y, w, h, 0, 0, px, py);
        double wi = (w1 < fabs(px)) ? w1 : fabs(px);
        double hi = (h1 < fabs(py)) ? h1 : fabs(py);
        ti        = -wi * hi; //-- ti is the negative area of intersection
        overlaps  = true;
        cf        = true;
    } else {
        double ti1 = -MATH_HUGE, ti2 = MATH_HUGE;
        double nx1, ny1, nx2, ny2;
        if (rect_getSegmentIntersectionIndices(x, y, w, h, 0, 0, dx, dy, ti1,
                                               ti2, nx1, ny1, nx2, ny2)) {
            //-- item tunnels into other
            if ((ti1 < 1) &&
                (fabs(ti1 - ti2) >= DELTA) //-- special case for rect going
                                           // through another rect's corner
                && ((0 < (ti1 + DELTA)) || ((0 == ti1) && (ti2 > 0)))) {
                ti       = ti1;
                nx       = nx1;
                ny       = ny1;
                overlaps = false;
                cf       = true;
            }
        }
    }

    if (!cf)
        return false;

    double tx, ty;

    if (overlaps) {
        if ((dx == 0) && (dy == 0)) {
            //-- intersecting and not moving - use minimum displacement vector
            double px, py;
            rect_getNearestCorner(x, y, w, h, 0, 0, px, py);
            if (fabs(px) < fabs(py))
                py = 0;
            else
                px = 0;
            nx = sign(px);
            ny = sign(py);
            tx = x1 + px;
            ty = y1 + py;
        } else {
            //-- intersecting and moving - move in the opposite direction
            double ti1 = -MATH_HUGE;
            double ti2 = 1;
            double nx1, ny1, nx2, ny2;
            if (!rect_getSegmentIntersectionIndices(
                    x, y, w, h, 0, 0, dx, dy, ti1, ti2, nx1, ny1, nx2, ny2))
                return false;
            tx = x1 + dx * ti1;
            ty = y1 + dy * ti1;
        }
    } else //-- tunnel
    {
        tx = x1 + dx * ti;
        ty = y1 + dy * ti;
    }

    col.overlaps    = overlaps;
    col.ti          = ti;
    col.move.x      = dx;
    col.move.y      = dy;
    col.normal.x    = nx;
    col.normal.y    = ny;
    col.touch.x     = tx;
    col.touch.y     = ty;
    col.itemRect.x  = x1;
    col.itemRect.y  = y1;
    col.itemRect.w  = w1;
    col.itemRect.h  = h1;
    col.otherRect.x = x2;
    col.otherRect.y = y2;
    col.otherRect.w = w2;
    col.otherRect.h = h2;
    return true;
}

//------------------------------------------
//-- Grid functions
//------------------------------------------

static void grid_toWorld(int cellSize, int cx, int cy, double &wx, double &wy)
{
    wx = (cx - 1) * cellSize;
    wy = (cy - 1) * cellSize;
}

static void grid_toCell(int cellSize, double x, double y, int &cx, int &cy)
{
    cx = floor(x / cellSize) + 1;
    cy = floor(y / cellSize) + 1;
}

/*-- grid_traverse* functions are based on "A Fast Voxel Traversal Algorithm for
 Ray Tracing",
 -- by John Amanides and Andrew Woo -
 http://www.cse.yorku.ca/~amana/research/grid.pdf
 -- It has been modified to include both cells when the ray "touches a grid
 corner",
 -- and with a different exit condition*/

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

typedef void (*pointFunc)(void *data, int, int);

static void grid_traverse(int cellSize, double x1, double y1, double x2,
                          double y2, pointFunc f, void *data)
{
    int cx1, cy1, cx2, cy2;
    double dx, tx, dy, ty;
    grid_toCell(cellSize, x1, y1, cx1, cy1);
    grid_toCell(cellSize, x2, y2, cx2, cy2);
    int stepX = grid_traverse_initStep(cellSize, cx1, x1, x2, dx, tx);
    int stepY = grid_traverse_initStep(cellSize, cy1, y1, y2, dy, ty);
    int cx    = cx1;
    int cy    = cy1;

    f(data, cx, cy);

    //-- The default implementation had an infinite loop problem when
    //-- approaching the last cell in some occassions. We finish iterating
    //-- when we are *next* to the last cell
    while ((iabs(cx - cx2) + iabs(cy - cy2)) > 1) {
        if (tx < ty) {
            tx += dx;
            cx += stepX;
            f(data, cx, cy);
        } else {
            //-- Addition: include both cells when going through corners
            if (tx == ty)
                f(data, cx + stepX, cy);
            ty += dy;
            cy += stepY;
            f(data, cx, cy);
        }
    }

    //-- If we have not arrived to the last cell, use it
    if ((cx != cx2) || (cy != cy2))
        f(data, cx2, cy2);
}

static void grid_toCellRect(int cellSize, double x, double y, double w,
                            double h, int &cx, int &cy, int &cw, int &ch)
{
    grid_toCell(cellSize, x, y, cx, cy);
    int cr = ceil((x + w) / cellSize);
    int cb = ceil((y + h) / cellSize);
    cw     = cr - cx + 1;
    ch     = cb - cy + 1;
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
                                 double y, double w, double h, double goalX,
                                 double goalY, ColFilter *filter,
                                 double &actualX, double &actualY,
                                 std::vector<Collision> &cols) = 0;
    virtual ~Response(){};
};

/*------------------------------------------
-- World
------------------------------------------*/

struct Cell {
    std::set<int> items;
    int x, y;
    Cell() : x(0), y(0) {}
};

struct ItemInfo {
    int item;
    double ti1, ti2, weight;
    double x1, y1, x2, y2;
};

struct ItemFilter {
    virtual bool Filter(int item) = 0;
    virtual ~ItemFilter(){};
};

struct CrossResponse;
struct TouchResponse;
struct SlideResponse;
struct BounceResponse;

struct SlideFilter : ColFilter {
    int Filter(int item, int other)
    {
        UNUSED(item);
        UNUSED(other);
        return Slide;
    };
};

struct TouchFilter : ColFilter {
    int Filter(int item, int other)
    {
        UNUSED(item);
        UNUSED(other);
        return Touch;
    };
};

struct CrossFilter : ColFilter {
    int Filter(int item, int other)
    {
        UNUSED(item);
        UNUSED(other);
        return Cross;
    };
};

struct BounceFilter : ColFilter {
    int Filter(int item, int other)
    {
        UNUSED(item);
        UNUSED(other);
        return Bounce;
    };
};

struct TouchResponse : Response {
    void ComputeResponse(World *world, Collision &col, double x, double y,
                         double w, double h, double goalX, double goalY,
                         ColFilter *filter, double &actualX, double &actualY,
                         std::vector<Collision> &cols);
};

struct CrossResponse : Response {
    void ComputeResponse(World *world, Collision &col, double x, double y,
                         double w, double h, double goalX, double goalY,
                         ColFilter *filter, double &actualX, double &actualY,
                         std::vector<Collision> &cols);
};

struct SlideResponse : Response {
    void ComputeResponse(World *world, Collision &col, double x, double y,
                         double w, double h, double goalX, double goalY,
                         ColFilter *filter, double &actualX, double &actualY,
                         std::vector<Collision> &cols);
};

struct BounceResponse : Response {
    void ComputeResponse(World *world, Collision &col, double x, double y,
                         double w, double h, double goalX, double goalY,
                         ColFilter *filter, double &actualX, double &actualY,
                         std::vector<Collision> &cols);
};

struct World {
    int cellSize;
    int itemId;
    std::map<int, Response *> responses;
    std::map<int, ColFilter *> filters;
    std::map<int, Rect> rects;
    std::map<int, std::map<int, Cell> > rows;

    void initialize (int cellSize)
    {
        this->cellSize = cellSize;
        this->itemId = 0;

        CrossFilter *filterCross   = new CrossFilter();
        TouchFilter *filterTouch   = new TouchFilter();
        SlideFilter *filterSlide   = new SlideFilter();
        BounceFilter *filterBounce = new BounceFilter();

        this->addFilter(Touch, filterTouch);
        this->addFilter(Cross, filterCross);
        this->addFilter(Slide, filterSlide);
        this->addFilter(Bounce, filterBounce);

        CrossResponse *responseCross   = new CrossResponse();
        TouchResponse *responseTouch   = new TouchResponse();
        SlideResponse *responseSlide   = new SlideResponse();
        BounceResponse *responseBounce = new BounceResponse();

        this->addResponse(Touch, responseTouch);
        this->addResponse(Cross, responseCross);
        this->addResponse(Slide, responseSlide);
        this->addResponse(Bounce, responseBounce);
    }

    void release()
    {
        Response *responseTouch = this->getResponseById(Touch);
        if(responseTouch) {
            delete responseTouch;
        }

        Response *responseCross = this->getResponseById(Cross);
        if(responseCross) {
            delete responseCross;
        }

        Response *responseSlide = this->getResponseById(Slide);
        if(responseSlide) {
            delete responseSlide;
        }

        Response *responseBounce = this->getResponseById(Bounce);
        if(responseBounce) {
            delete responseBounce;
        }

        this->responses.erase(Touch);
        this->responses.erase(Cross);
        this->responses.erase(Bounce);
        this->responses.erase(Slide);

        ColFilter *filterTouch = this->getFilterById(Touch);
        delete filterTouch;

        ColFilter *filterCross = this->getFilterById(Cross);
        delete filterCross;

        ColFilter *filterSlide = this->getFilterById(Slide);
        delete filterSlide;

        ColFilter *filterBounce = this->getFilterById(Bounce);
        delete filterBounce;

        this->filters.erase(Touch);
        this->filters.erase(Cross);
        this->filters.erase(Bounce);
        this->filters.erase(Slide);

        this->clear();
    }

    //-- Private functions and methods
    static bool sortByWeight(ItemInfo a, ItemInfo b)
    {
        return a.weight < b.weight;
    }

    static bool sortByTiAndDistance(Collision a, Collision b)
    {
        if (a.ti == b.ti) {
            double ad = rect_getSquareDistance(
                a.itemRect.x, a.itemRect.y, a.itemRect.w, a.itemRect.h,
                a.otherRect.x, a.otherRect.y, a.otherRect.w, a.otherRect.h);
            double bd = rect_getSquareDistance(
                a.itemRect.x, a.itemRect.y, a.itemRect.w, a.itemRect.h,
                b.otherRect.x, b.otherRect.y, b.otherRect.w, b.otherRect.h);
            return ad < bd;
        }
        return a.ti < b.ti;
    }

    void addItemToCell(int item, int cx, int cy)
    {
        rows[cy][cx].items.insert(item);
    }

    bool removeItemFromCell(int item, int cx, int cy)
    {
        std::map<int, std::map<int, Cell> >::iterator row = rows.find(cy);
        if (row == rows.end())
            return false;
        std::map<int, Cell>::iterator cell = row->second.find(cx);
        if (cell == row->second.end())
            return false;
        if (cell->second.items.find(item) == cell->second.items.end())
            return false;
        cell->second.items.erase(item);
        return true;
    }

    void getDictItemsInCellRect(int cl, int ct, int cw, int ch,
                                std::set<int> &items_dict)
    {
        for (int cy = ct; cy < ct + ch; cy++) {
            std::map<int, std::map<int, Cell> >::iterator row = rows.find(cy);
            if (row == rows.end())
                continue;
            for (int cx = cl; cx < cl + cw; cx++) {
                std::map<int, Cell>::iterator cell = row->second.find(cx);
                if (cell == row->second.end())
                    continue;
                if (cell->second.items.size() > 0) {
                    for (std::set<int>::iterator it =
                             cell->second.items.begin();
                         it != cell->second.items.end(); it++)
                        items_dict.insert(*it);
                }
            }
        }
    }
    struct _CellTraversal {
        World *world;
        std::set<Cell *> cells;
    };
    static void cellsTraversal_(void *ctx, int cx, int cy)
    {
        struct _CellTraversal *ct = (struct _CellTraversal *)ctx;
        std::map<int, std::map<int, Cell> >::iterator row =
            ct->world->rows.find(cy);
        if (row == ct->world->rows.end())
            return;
        std::map<int, Cell>::iterator cell = row->second.find(cx);
        if (cell == row->second.end())
            return;
        ct->cells.insert(&cell->second);
    }
    
    std::set<Cell *> getCellsTouchedBySegment(double x1, double y1, double x2,
                                              double y2)
    {
        struct _CellTraversal ct;
        ct.world = this;
        grid_traverse(cellSize, x1, y1, x2, y2, cellsTraversal_, &ct);
        return ct.cells;
    }

    void getInfoAboutItemsTouchedBySegment(double x1, double y1, double x2,
                                           double y2, ItemFilter *filter,
                                           std::vector<ItemInfo> &itemInfo)
    {
        std::set<Cell *> cells = getCellsTouchedBySegment(x1, y1, x2, y2);
        std::set<int> visited;

        for (std::set<Cell *>::iterator it = cells.begin(); it != cells.end();
             it++) {
            Cell *cell = (*it);
            for (std::set<int>::iterator i = cell->items.begin();
                 i != cell->items.end(); i++) {
                if (visited.find(*i) == visited.end()) {
                    visited.insert(*i);
                    if ((!filter) || filter->Filter(*i)) {
                        Rect r = rects[*i];
                        double nx1, ny1, nx2, ny2;
                        double ti1 = 0;
                        double ti2 = 1;
                        rect_getSegmentIntersectionIndices(
                            r.x, r.y, r.w, r.h, x1, y1, x2, y2, ti1, ti2, nx1,
                            ny1, nx2, ny2);
                        if (ti1 && (((0 < ti1) && (ti1 < 1)) ||
                                    ((0 < ti2) && (ti2 < 1)))) {
                            //-- the sorting is according to the t of an
                            // infinite line, not the segment
                            double tii0 = -MATH_HUGE;
                            double tii1 = MATH_HUGE;
                            rect_getSegmentIntersectionIndices(
                                r.x, r.y, r.w, r.h, x1, y1, x2, y2, tii0, tii1,
                                nx1, ny1, nx2, ny2);
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

    ColFilter * getFilterById(int id) 
    {
        return filters[id];
    }

    void project(int item, double x, double y, double w, double h, double goalX,
                 double goalY, ColFilter *filter,
                 std::vector<Collision> &collisions)
    {
        std::set<int> visited;
        if (item) {
            visited.insert(item);
        }

        //-- This could probably be done with less cells using a polygon raster
        // over the cells instead of a
        //-- bounding rect of the whole movement. Conditional to building a
        // queryPolygon method
        double tl = (goalX < x) ? goalX : x;
        double tt = (goalY < y) ? goalY : y;
        double tr = ((goalX + w) > (x + w)) ? goalX + w : x + w;
        double tb = ((goalY + h) > (y + h)) ? goalY + h : y + h;
        double tw = tr - tl;
        double th = tb - tt;

        int cl, ct, cw, ch;
        grid_toCellRect(cellSize, tl, tt, tw, th, cl, ct, cw, ch);

        std::set<int> dictItemsInCellRect;
        getDictItemsInCellRect(cl, ct, cw, ch, dictItemsInCellRect);

        for (std::set<int>::iterator it = dictItemsInCellRect.begin();
             it != dictItemsInCellRect.end(); it++) {
            int other = *it;
            if (visited.find(other) == visited.end()) {
                visited.insert(other);
                int responseId = filter->Filter(item, other);
                if (responseId > 0) {
                    double ox, oy, ow, oh;
                    getRect(other, ox, oy, ow, oh);
                    Collision col;
                    if (rect_detectCollision(x, y, w, h, ox, oy, ow, oh, goalX,
                                             goalY, col)) {
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

    int countCells()
    {
        int count = 0;
        for (std::map<int, std::map<int, Cell> >::iterator row = rows.begin();
             row != rows.end(); row++)
            count += row->second.size();
        return count;
    }

    bool hasItem(int item)
    {
        return rects.find(item) != rects.end();
    }

    std::set<int> getItems()
    {
        std::set<int> items;
        for (std::map<int, Rect>::iterator r = rects.begin(); r != rects.end();
             r++)
            items.insert(r->first);
        return items;
    }

    int countItems()
    {
        return rects.size();
    }

    void getRect(int item, double &x, double &y, double &w, double &h)
    {
        Rect r = rects[item];
        x      = r.x;
        y      = r.y;
        w      = r.w;
        h      = r.h;
    }

    void toWorld(int cx, int cy, double &x, double &y)
    {
        grid_toWorld(cellSize, cx, cy, x, y);
    }
    void toCell(double x, double y, int &cx, int &cy)
    {
        grid_toCell(cellSize, x, y, cx, cy);
    }

    //--- Query methods

    void queryRect(double x, double y, double w, double h, ItemFilter *filter,
                   std::set<int> &dictItemsInCellRect)
    {
        int cl, ct, cw, ch;
        grid_toCellRect(cellSize, x, y, w, h, cl, ct, cw, ch);
        getDictItemsInCellRect(cl, ct, cw, ch, dictItemsInCellRect);
        for (std::set<int>::iterator it = dictItemsInCellRect.begin();
             it != dictItemsInCellRect.end();) {
            bool drop = (filter && !filter->Filter(*it));
            if (!drop) {
                Rect rect = rects[*it];
                drop |= !rect_isIntersecting(x, y, w, h, rect.x, rect.y, rect.w,
                                             rect.h);
            }
            if (drop)
                dictItemsInCellRect.erase(it++);
            else
                ++it;
        }
    }

    void queryPoint(double x, double y, ItemFilter *filter,
                    std::set<int> &dictItemsInCellRect)
    {
        int cx, cy;
        toCell(x, y, cx, cy);
        getDictItemsInCellRect(cx, cy, 1, 1, dictItemsInCellRect);
        for (std::set<int>::iterator it = dictItemsInCellRect.begin();
             it != dictItemsInCellRect.end();) {
            Rect rect = rects[*it];
            if ((filter && !filter->Filter(*it)) ||
                !rect_containsPoint(rect.x, rect.y, rect.w, rect.h, x, y))
                dictItemsInCellRect.erase(it++);
            else
                ++it;
        }
    }

    void querySegment(double x1, double y1, double x2, double y2,
                      ItemFilter *filter, std::set<int> &items)
    {
        std::vector<ItemInfo> itemInfo;
        getInfoAboutItemsTouchedBySegment(x1, y1, x2, y2, filter, itemInfo);
        for (std::vector<ItemInfo>::iterator it = itemInfo.begin();
             it != itemInfo.end(); it++)
            items.insert((*it).item);
    }

    void querySegmentWithCoords(double x1, double y1, double x2, double y2,
                                ItemFilter *filter,
                                std::vector<ItemInfo> &itemInfo2)
    {
        std::vector<ItemInfo> itemInfo;
        getInfoAboutItemsTouchedBySegment(x1, y1, x2, y2, filter, itemInfo);
        double dx = x2 - x1, dy = y2 - y1;
        for (std::vector<ItemInfo>::iterator it = itemInfo.begin();
             it != itemInfo.end(); it++) {
            ItemInfo i = *it;
            i.x1       = x1 + dx * i.ti1;
            i.y1       = y1 + dy * i.ti1;
            i.x2       = x1 + dx * i.ti2;
            i.y2       = y1 + dy * i.ti2;
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

    void add(int item, double x, double y, double w, double h)
    {
        Rect r;
        r.x         = x;
        r.y         = y;
        r.w         = w;
        r.h         = h;
        rects[item] = r;

        int cl, ct, cw, ch;
        grid_toCellRect(cellSize, x, y, w, h, cl, ct, cw, ch);
        for (int cy = ct; cy < ct + ch; cy++) {
            for (int cx = cl; cx < cl + cw; cx++) {
                addItemToCell(item, cx, cy);
            }
        }
    }

    void remove(int item)
    {
        Rect r = rects[item];
        int cl, ct, cw, ch;
        grid_toCellRect(cellSize, r.x, r.y, r.w, r.h, cl, ct, cw, ch);
        for (int cy = ct; cy < ct + ch; cy++) {
            for (int cx = cl; cx < cl + cw; cx++) {
                removeItemFromCell(item, cx, cy);
            }
        }
        rects.erase(item);
    }

    void clear() {
        itemId = 0;
        rects.clear();
        rows.clear();
    }

    void update(int item, double x2, double y2, double w2, double h2)
    {
        Rect r = rects[item];
        if (w2 <= 0)
            w2 = r.w;
        if (h2 <= 0)
            h2 = r.h;

        if ((r.x != x2) || (r.y != y2) || (r.w != w2) || (r.h != h2)) {
            int cl1, ct1, cw1, ch1;
            grid_toCellRect(cellSize, r.x, r.y, r.w, r.h, cl1, ct1, cw1, ch1);
            int cl2, ct2, cw2, ch2;
            grid_toCellRect(cellSize, x2, y2, w2, h2, cl2, ct2, cw2, ch2);

            if ((cl1 != cl2) || (ct1 != ct2) || (cw1 != cw2) || (ch1 != ch2)) {
                int cr1 = cl1 + cw1 - 1, cb1 = ct1 + ch1 - 1;
                int cr2 = cl2 + cw2 - 1, cb2 = ct2 + ch2 - 1;
                bool cyOut;

                for (int cy = ct1; cy <= cb1; cy++) {
                    cyOut = (cy < ct2) || (cy > cb2);
                    for (int cx = cl1; cx <= cr1; cx++) {
                        if (cyOut || (cx < cl2) || (cx > cr2))
                            removeItemFromCell(item, cx, cy);
                    }
                }

                for (int cy = ct2; cy <= cb2; cy++) {
                    cyOut = (cy < ct1) || (cy > cb1);
                    for (int cx = cl2; cx <= cr2; cx++) {
                        if (cyOut || (cx < cl1) || (cx > cr1))
                            addItemToCell(item, cx, cy);
                    }
                }
            }

            Rect r;
            r.x         = x2;
            r.y         = y2;
            r.w         = w2;
            r.h         = h2;
            rects[item] = r;
        }
    }

    void move(int item, double goalX, double goalY, ColFilter *filter,
              double &actualX, double &actualY, std::vector<Collision> &cols)
    {
        check(item, goalX, goalY, filter, actualX, actualY, cols);
        update(item, actualX, actualY, -1, -1);
    }

    void check(int item, double goalX, double goalY, ColFilter *filter,
               double &actualX, double &actualY, std::vector<Collision> &cols)
    {
        VisitedFilter vf;
        vf.visited.insert(item);
        vf.filter = filter;

        Rect r = rects[item];

        std::vector<Collision> projected_cols;
        project(item, r.x, r.y, r.w, r.h, goalX, goalY, &vf, projected_cols);

        while (projected_cols.size() > 0) {
            Collision col = projected_cols[0];
            vf.visited.insert(col.other);
            Response *response = getResponseById(col.type);

            projected_cols.clear();
            response->ComputeResponse(this, col, r.x, r.y, r.w, r.h, goalX,
                                      goalY, &vf, goalX, goalY, projected_cols);
            cols.push_back(col);
        }

        actualX = goalX;
        actualY = goalY;
    }
};

void TouchResponse::ComputeResponse(World *world, Collision &col, double x, double y,
                         double w, double h, double goalX, double goalY,
                         ColFilter *filter, double &actualX, double &actualY,
                         std::vector<Collision> &cols)
    {
        UNUSED(world);
        UNUSED(x);
        UNUSED(y);
        UNUSED(w);
        UNUSED(h);
        UNUSED(filter);
        UNUSED(goalX);
        UNUSED(goalY);
        UNUSED(cols);
        actualX = col.touch.x;
        actualY = col.touch.y;
    }

void CrossResponse :: ComputeResponse (World *world, Collision &col, double x, double y,
                         double w, double h, double goalX, double goalY,
                         ColFilter *filter, double &actualX, double &actualY,
                         std::vector<Collision> &cols)
    {
        world->project(col.item, x, y, w, h, goalX, goalY, filter, cols);
        actualX = goalX;
        actualY = goalY;
    }

void SlideResponse :: ComputeResponse (World *world, Collision &col, double x, double y,
                         double w, double h, double goalX, double goalY,
                         ColFilter *filter, double &actualX, double &actualY,
                         std::vector<Collision> &cols)
    {
        double sx = col.touch.x;
        double sy = col.touch.y;

        if ((col.move.x != 0) || (col.move.y != 0)) {
            if (col.normal.x == 0)
                sx = goalX;
            else
                sy = goalY;
        }

        col.response.x = sx;
        col.response.y = sy;

        x     = col.touch.x;
        y     = col.touch.y;
        goalX = sx;
        goalY = sy;
        world->project(col.item, x, y, w, h, goalX, goalY, filter, cols);
        actualX = goalX;
        actualY = goalY;
    }

void BounceResponse ::ComputeResponse (World *world, Collision &col, double x, double y,
                         double w, double h, double goalX, double goalY,
                         ColFilter *filter, double &actualX, double &actualY,
                         std::vector<Collision> &cols)
    {
        double tx = col.touch.x;
        double ty = col.touch.y;
        double bx = tx;
        double by = ty;

        if ((col.move.x != 0) || (col.move.y != 0)) {
            double bnx = goalX - tx, bny = goalY - ty;
            if (col.normal.x == 0)
                bny = -bny;
            else
                bnx = -bnx;
            bx = tx + bnx;
            by = ty + bny;
        }

        col.response.x = bx;
        col.response.y = by;
        x              = tx;
        y              = ty;
        goalX          = bx;
        goalY          = by;
        world->project(col.item, x, y, w, h, goalX, goalY, filter, cols);
        actualX = goalX;
        actualY = goalY;
    }

} // namespace bump
