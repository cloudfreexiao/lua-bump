local bump = require('bump2d')
local test = require('u-test')

local Touch = bump.touch
local Cross = bump.cross
local Slide = bump.slide
local Bounce = bump.bounce

local world = bump.newWorld()

local collect = function(t, field_name)
    local res = {}
    for i, v in ipairs(t) do
        res[i] = v[field_name]
    end
    return res
end

local sorted = function(array)
    table.sort(array)
    return array
end

test['creates as many cells as needed to hold the item'] = function()
    world:add(0, 0, 10, 10) -- adss one cell
    test.equal(world:countCells(), 1)

    world:add(100, 100, 10, 10) -- adds a separate single cell
    test.equal(world:countCells(), 2)

    world:add(0, 0, 100, 10) -- occupies 2 cells, but just adds one (the other is already added)
    test.equal(world:countCells(), 3)

    world:add(0, 0, 100, 10) -- occupies 2 cells, but just adds one (the other is already added)
    test.equal(world:countCells(), 3)

    world:add(300, 300, 64, 64) -- adds 8 new cells
    test.equal(world:countCells(), 7)

    world:clear()
end

test['updates the object'] = function()
    local id = world:add(0, 0, 10, 10)
    world:update(id, 40, 40, 20, 20)
    test.is_table({world:getRect(id)}, {40, 40, 20, 20})
    world:clear()
end

test['queryRect returns nothing when the world is empty'] = function()
    test.is_table(world:queryRect(0, 0, 1, 1), {})
    world:clear()
end

test['queryRect when the world has items'] = function()
    local a = world:add(10, 0, 10, 10)
    local b = world:add(70, 0, 10, 10)
    local c = world:add(50, 0, 10, 10)
    local d = world:add(90, 0, 10, 10)

    test.is_table(sorted(world:queryRect(55, 5, 20, 20)), {b, c})
    test.is_table(sorted(world:queryRect(0, 5, 100, 20)), {a, b, c, d})

    world:clear()
end

test['queryPoint returns the items inside/partially inside the given rect'] = function()
    local a = world:add(10, 0, 10, 10)
    local b = world:add(15, 0, 10, 10)
    local c = world:add(20, 0, 10, 10)

    test.is_table(sorted(world:queryPoint( 4,5)), {})
    test.is_table(sorted(world:queryPoint(14,5)), {a})
    test.is_table(sorted(world:queryPoint(16,5)), {a,b})
    test.is_table(sorted(world:queryPoint(21,5)), {b,c})
    test.is_table(sorted(world:queryPoint(26,5)), {c})
    test.is_table(sorted(world:queryPoint(31,5)), {})

    world:clear()
end

test['querySegment returns the items touched by the segment, sorted by touch order'] = function()
    local a = world:add(5, 0, 5, 10)
    local b = world:add(15, 0, 5, 10)
    local c = world:add(25, 0, 5, 10)

    test.is_table(world:querySegment(0,5, 11,5),  {a})
    test.is_table(world:querySegment(0,5, 17,5),  {a,b})
    test.is_table(world:querySegment(0,5, 30,5),  {a,b,c})
    test.is_table(world:querySegment(17,5, 26,5), {b,c})
    test.is_table(world:querySegment(22,5, 26,5), {c})

    test.is_table(world:querySegment(11,5, 0,5),  {a})
    test.is_table(world:querySegment(17,5, 0,5),  {b,a})
    test.is_table(world:querySegment(30,5, 0,5),  {c,b,a})
    test.is_table(world:querySegment(26,5, 17,5), {c,b})
    test.is_table(world:querySegment(26,5, 22,5), {c})

    world:clear()
end

test['hasItem returns wether the world has an item'] = function()
    test.is_false(world:hasItem(1))
    world:add(0,0,1,1)
    test.is_true(world:hasItem(1))
    world:clear()
end

test['countItems'] = function()
    world:add(1, 1, 1, 1)
    world:add(2, 2, 2, 2)
    local count = world:countItems()
    test.equal(2, count)

    world:clear()
end

test['move when there are no collisions'] = function()
    local item = world:add(0, 0, 1, 1)
    test.is_table({1, 1, {}, 0}, {world:move(item, 1, 1)})

    world:clear()
end

test['move when touching returns a collision with the first item it touches'] = function()
    local a = world:add(0, 0, 1, 1)
    local b = world:add(0, 2, 1, 1)
    world:add(0, 3, 1, 1)

    local x, y, cols, len = world:move(a, 0, 5, Touch)
    test.is_table({x, y}, {0, 1})
    test.equal(1, len)
    test.is_table(collect(cols, 'other'), {b})
    test.is_table(collect(cols, 'type'), {Touch})
    test.is_table({0, 1, 1, 1}, {world:getRect(a)})

    world:clear()
end

test['move when crossing returns a collision with every item it crosses'] = function()
    local a = world:add(0, 0, 1, 1)
    local b = world:add(0, 2, 1, 1)
    local c = world:add(0, 3, 1, 1)
    local x, y, cols, len = world:move(a, 0, 5, Cross)
    test.is_table({x, y}, {0, 5})
    test.equal(2, len)
    test.is_table(collect(cols, 'other'), {b, c})
    test.is_table(collect(cols, 'type'), {Cross, Cross})
    test.is_table({0, 5, 1, 1}, {world:getRect(a)})

    world:clear()
end

test['move when sliding slides with every element'] = function()
    local a  = world:add(0,0,1,1)
    world:add(0,2,1,2)
    local c = world:add(2,1,1,1)
    local x,y,cols,len = world:move(a, 5,5, Slide)
    test.is_table({x,y}, {1,5})
    test.equal(1, len)
    test.is_table(collect(cols, 'other'), {c})
    test.is_table(collect(cols, 'type'),  {Slide})
    test.is_table({1,5,1,1}, {world:getRect(a)})

    world:clear()
end

test['move when bouncing bounces on each element'] = function()
    local a = world:add(0, 0, 1, 1)
    local b = world:add(0, 2, 1, 2)
    local x, y, cols, len = world:move(a, 0, 5, Bounce)
    test.is_table({x, y}, {0, -3})
    test.equal(1, len)
    test.is_table(collect(cols, 'other'), {b})
    test.is_table(collect(cols, 'type'), {Bounce})
    test.is_table({0, -3, 1, 1}, {world:getRect(a)})

    world:clear()
end

world = nil
