local bump = require('bump2d')
local test = require('u-test')


local detect = bump.rect.detectCollision
local responses = {}

responses.touch = function(world, col, x, y, w, h, goalX, goalY, filter)
    return col.touch.x, col.touch.y, {}, 0
end

responses.cross = function(world, col, x, y, w, h, goalX, goalY, filter)
    local cols, len = world:project(col.item, x, y, w, h, goalX, goalY, filter)
    return goalX, goalY, cols, len
end

responses.slide = function(world, col, x, y, w, h, goalX, goalY, filter)
    goalX = goalX or x
    goalY = goalY or y

    local tch, move = col.touch, col.move
    if move.x ~= 0 or move.y ~= 0 then
        if col.normal.x ~= 0 then
            goalX = tch.x
        else
            goalY = tch.y
        end
    end

    col.slide = {
        x = goalX,
        y = goalY,
    }

    x, y = tch.x, tch.y
    local cols, len = world:project(col.item, x, y, w, h, goalX, goalY, filter)
    return goalX, goalY, cols, len
end

responses.bounce = function(world, col, x, y, w, h, goalX, goalY, filter)
    goalX = goalX or x
    goalY = goalY or y

    local tch, move = col.touch, col.move
    local tx, ty = tch.x, tch.y

    local bx, by = tx, ty

    if move.x ~= 0 or move.y ~= 0 then
        local bnx, bny = goalX - tx, goalY - ty
        if col.normal.x == 0 then
            bny = -bny
        else
            bnx = -bnx
        end
        bx, by = tx + bnx, ty + bny
    end

    col.bounce = {
        x = bx,
        y = by,
    }
    x, y = tch.x, tch.y
    goalX, goalY = bx, by

    local cols, len = world:project(col.item, x, y, w, h, goalX, goalY, filter)
    return goalX, goalY, cols, len
end

---------------------------------------------------------------
local world = bump.newWorld()

local touch = function(x, y, w, h, ox, oy, ow, oh, goalX, goalY)
    goalX = goalX or x
    goalY = goalY or y
    local col = detect(x, y, w, h, ox, oy, ow, oh, goalX, goalY)
    responses.touch(world, col, x, y, h, goalX, goalY)
    return {col.touch.x, col.touch.y, col.normal.x, col.normal.y}
end

local slide = function(x, y, w, h, ox, oy, ow, oh, goalX, goalY)
    local col = detect(x, y, w, h, ox, oy, ow, oh, goalX, goalY)
    -- print("slides col", inspect(col))
    responses.slide(world, col, x, y, w, h, goalX, goalY)
    return {col.touch.x, col.touch.y, col.normal.x, col.normal.y, col.slide.x, col.slide.y}
end

local bounce = function(x, y, w, h, ox, oy, ow, oh, goalX, goalY)
    local col = detect(x, y, w, h, ox, oy, ow, oh, goalX, goalY)
    responses.bounce(world, col, x, y, w, h, goalX, goalY)
    return {col.touch.x, col.touch.y, col.normal.x, col.normal.y, col.bounce.x, col.bounce.y}
end

test['returns the left,top coordinates of the minimum displacement on static items'] = function()
    --                                          -2-1 0 1 2 3 4 5 6 7 8 9 10
    --      -2 -1 0 1 2 3 4 5 6 7 8 9           -2 · ┌–––┐ · ┌–––┐ · ┌–––┐ ·
    --      -1  ┌–––┐ · ┌–––┐ · ┌–––┐           -1 · │0-1│ · │0-1│ · │0-1│ ·
    --       0  │ ┌–––––––––––––––┐ │ 1  2  3    0 · └–┌–––––––––––––––┐–┘ ·
    --       1  └–│–┘ · └–––┘ · └–│–┘            1 · · │ · · · · · · · │ · ·
    --       2  · │ · · · · · · · │ ·            2 · · │ · · · · · · · │ · ·
    --       3  ┌–│–┐ · ┌–––┐ · ┌–│–┐            3 ┌–––│ · · · · · · · │–––┐
    --       4  │ │ │ · │ · │ · │ │ │ 4  5  6    4 -1 0│ · · · · · · · │1 0│
    --       5  └–│–┘ · └–––┘ · └–│–┘            5 └–––│ · · · · · · · │–––┘
    --       6  · │ · · · · · · · │ ·            6 · · │ · · · · · · · │ · ·
    --       7  ┌–│–┐ · ┌–––┐ · ┌–│–┐            7 · · │ · · · · · · · │ · ·
    --       8  │ └–––––––––––––––┘ │ 7  8  9    8 · ┌–└–––––––––––––––┘–┐ ·
    --       9  └–––┘ · └–––┘ · └–––┘            9 · │0 1│ · ╎0 1╎ · │0 1│ ·
    --      10                                  10 · └–––┘ · └╌╌╌┘ · └–––┘ ·

    test.is_table(touch(-1, -1, 2, 2, 0, 0, 8, 8), {-1, -2, 0, -1}) -- 1
    test.is_table(touch(3, -1, 2, 2, 0, 0, 8, 8), {3, -2, 0, -1}) -- 2
    test.is_table(touch(7, -1, 2, 2, 0, 0, 8, 8), {7, -2, 0, -1}) -- 3

    test.is_table(touch(-1, 3, 2, 2, 0, 0, 8, 8), {-2, 3, -1, 0}) -- 4
    test.is_table(touch(3, 3, 2, 2, 0, 0, 8, 8), {3, 8, 0, 1}) -- 5
    test.is_table(touch(7, 3, 2, 2, 0, 0, 8, 8), {8, 3, 1, 0}) -- 6

    test.is_table(touch(-1, 7, 2, 2, 0, 0, 8, 8), {-1, 8, 0, 1}) -- 7
    test.is_table(touch(3, 7, 2, 2, 0, 0, 8, 8), {3, 8, 0, 1}) -- 8
    test.is_table(touch(7, 7, 2, 2, 0, 0, 8, 8), {7, 8, 0, 1}) -- 9
end

test['returns the left,top coordinates of the overlaps with the movement line, opposite direction'] = function()
    test.is_table(touch(3, 3, 2, 2, 0, 0, 8, 8, 4, 3), {-2, 3, -1, 0})
    test.is_table(touch(3, 3, 2, 2, 0, 0, 8, 8, 2, 3), {8, 3, 1, 0})
    test.is_table(touch(3, 3, 2, 2, 0, 0, 8, 8, 2, 3), {8, 3, 1, 0})
    test.is_table(touch(3, 3, 2, 2, 0, 0, 8, 8, 3, 4), {3, -2, 0, -1})
    test.is_table(touch(3, 3, 2, 2, 0, 0, 8, 8, 3, 2), {3, 8, 0, 1})
end

test['returns the coordinates of the item when it starts touching the other, and the normal'] = function()
    test.is_table(touch(-3, 3, 2, 2, 0, 0, 8, 8, 3, 3), {-2, 3, -1, 0})
    test.is_table(touch(9, 3, 2, 2, 0, 0, 8, 8, 3, 3), {8, 3, 1, 0})
    test.is_table(touch(3, -3, 2, 2, 0, 0, 8, 8, 3, 3), {3, -2, 0, -1})
    test.is_table(touch(3, 9, 2, 2, 0, 0, 8, 8, 3, 3), {3, 8, 0, 1})
end

test['slides on overlaps'] = function()
    local id = world:add(3, 3, 2, 2)
    test.is_table(slide(3, 3, 2, 2, 0, 0, 8, 8, 4, 5), {0.5, -2, 0, -1, 4, -2})

    test.is_table(slide(3, 3, 2, 2, 0, 0, 8, 8, 5, 4), {-2, 0.5, -1, 0, -2, 4})
    test.is_table(slide(3, 3, 2, 2, 0, 0, 8, 8, 2, 1), {5.5, 8, 0, 1, 2, 8})
    test.is_table(slide(3, 3, 2, 2, 0, 0, 8, 8, 1, 2), {8, 5.5, 1, 0, 8, 2})

    world:remove(id)
end

test['slides over tunnels'] = function()
    local id = world:add(10, 10, 2, 2)

    test.is_table(slide(10, 10, 2, 2, 0, 0, 8, 8, 1, 4), {7, 8, 0, 1, 1, 8})
    test.is_table(slide(10, 10, 2, 2, 0, 0, 8, 8, 4, 1), {8, 7, 1, 0, 8, 1})

    -- perfect corner case:
    test.is_table(slide(10, 10, 2, 2, 0, 0, 8, 8, 1, 1), {8, 8, 1, 0, 8, 1})

    world:remove(id)
end

test['bounces on overlaps'] = function()
    local id = world:add(3, 3, 2, 2)

    test.is_table(bounce(3, 3, 2, 2, 0, 0, 8, 8, 4, 5), {0.5, -2, 0, -1, 4, -9})
    test.is_table(bounce(3, 3, 2, 2, 0, 0, 8, 8, 5, 4), {-2, 0.5, -1, 0, -9, 4})
    test.is_table(bounce(3, 3, 2, 2, 0, 0, 8, 8, 2, 1), {5.5, 8, 0, 1, 2, 15})
    test.is_table(bounce(3, 3, 2, 2, 0, 0, 8, 8, 1, 2), {8, 5.5, 1, 0, 15, 2})

    world:remove(id)
end

test['bounces over tunnels'] = function()
    local id = world:add(10, 10, 2, 2)

    test.is_table(bounce(10, 10, 2, 2, 0, 0, 8, 8, 1, 4), {7, 8, 0, 1, 1, 12})
    test.is_table(bounce(10, 10, 2, 2, 0, 0, 8, 8, 4, 1), {8, 7, 1, 0, 12, 1})

    -- perfect corner case:
    test.is_table(bounce(10, 10, 2, 2, 0, 0, 8, 8, 1, 1), {8, 8, 1, 0, 15, 1})

    world:remove(id)
end

world = nil
