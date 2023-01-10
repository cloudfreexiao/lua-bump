local bump = require('bump')
local test = require('u-test')

local detect = bump.rect.detectCollision

test['when itemRect does not intersect otherRect'] = function()
    local c = detect(0, 0, 1, 1, 5, 5, 1, 1, 0, 0)
    test.is_nil(c)
end

test['returns overlaps, normal, move, ti, diff, itemRect, otherRect'] = function()
    local c = detect(0, 0, 7, 6, 5, 5, 1, 1, 0, 0)

    test.is_true(c.overlaps)
    test.equal(c.ti, -2)
    test.is_table(c.move, {
        x = 0,
        y = 0,
    })
    test.is_table(c.itemRect, {
        x = 0,
        y = 0,
        w = 7,
        h = 6,
    })
    test.is_table(c.otherRect, {
        x = 5,
        y = 5,
        w = 1,
        h = 1,
    })
    test.is_table(c.normal, {
        x = 0,
        y = -1,
    })
end

test['when itemRect does not intersect otherRect'] = function()
    local c = detect(0, 0, 1, 1, 5, 5, 1, 1, 0, 1)
    test.is_nil(c)
end

test['detects collisions from the left'] = function()
    local c = detect(1, 1, 1, 1, 5, 0, 1, 1, 6, 0)
    test.equal(c.ti, 0.6)
    test.is_table(c.normal, {
        x = -1,
        y = 0,
    })
end

test['detects collisions from the right'] = function()
    local c = detect(6, 0, 1, 1, 1, 0, 1, 1, 1, 1)
    test.is_false(c.overlaps)
    test.equal(c.ti, 0.8)
    test.is_table(c.normal, {
        x = 1,
        y = 0,
    })
end

test['detects collisions from the top'] = function()
    local c = detect(0, 0, 1, 1, 0, 4, 1, 1, 0, 5)
    test.is_false(c.overlaps)
    test.equal(c.ti, 0.6)
    test.is_table(c.normal, {
        x = 0,
        y = -1,
    })
end

test['detects collisions from the bottom'] = function()
    local c = detect(0, 4, 1, 1, 0, 0, 1, 1, 0, -1)
    test.is_false(c.overlaps)
    test.equal(c.ti, 0.6)
    test.is_table(c.normal, {
        x = 0,
        y = 1,
    })
end

test['does not get caught by nasty corner cases'] = function()
    test.is_nil(detect(0, 16, 16, 16, 16, 0, 16, 16, -1, 15))
end
