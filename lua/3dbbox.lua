---@diagnostic disable: lowercase-global

BBox3d = {
    drawSettings = {
        bottomPlane = true,
        topPlane = false,
        full = false,
        corners = false,
        bottomCenter = true,
        topCenter = false,
        cornersColor = { { 255, 255, 255 }, { 255, 0, 0 }, { 0, 255, 0 }, { 0, 0, 255 }, { 255, 255, 0 }, { 255, 0, 255 }, { 0, 255, 255 }, { 0, 127, 127 } },
        bottomCenterColor = { 255, 255, 255 },
        topCenterColor = { 255, 0, 0 },
        bottomPlaneColorAndTickness = { 255, 255, 0, 1 },
        topPlaneColorAndTickness = { 0, 255, 255, 1 },
        verticalLinesColorAndTickness = { 255, 255, 255, 1 }
    },
    algorithmSettings = {
        pickBottomPlane = false
    }
}

BBox3d.ConvertToBBox3dData = BBox3d.Convert4PointsToBBox3dData

function BBox3d.Convert4PointsToBBox3dData(feat,w,h)
    return BBox3d.Convert6PointsToBBox3dData(BBox3d.Convert4PointsTo6Points(feat),w,h)
end

-- converts 8 values in 4 points to 12 values (see function below)
-- 4 points defined: center_pos, offset_1, offset_2, offset_3
function BBox3d.Convert4PointsTo6Points(feat)
    superpos_x = feat[1]
    superpos_y = feat[2]
    longest_x = feat[3]
    longest_y = feat[4]
    medium_x = feat[5]
    medium_y = feat[6]
    shortest_x = feat[7]
    shortest_y = feat[8]

    p1x = superpos_x-longest_x
    p1y = superpos_y-longest_y
    p2x = superpos_x+longest_x
    p2y = superpos_y+longest_y
    p3x = superpos_x-medium_x
    p3y = superpos_y-medium_y
    p4x = superpos_x+medium_x
    p4y = superpos_y+medium_y
    p5x = superpos_x-shortest_x
    p5y = superpos_y-shortest_y
    p6x = superpos_x+shortest_x
    p6y = superpos_y+shortest_y

    return {p1x,p1y,p2x,p2y,p3x,p3y,p4x,p4y,p5x,p5y,p6x,p6y}
end

function BBox3d.BiggestSide(points,w,h) 
    --calculate bounds of points
    local minx = 999
    local maxx = -1
    local miny = 999
    local maxy = -1
    for i=1, #points, 1 do
        if points[i][1] < minx then
            minx = points[i][1]
        end
        if points[i][1] > maxx then
            maxx = points[i][1]
        end
        if points[i][2] < miny then
            miny = points[i][2]
        end
        if points[i][2] > maxy then
            maxy = points[i][2]
        end
    end
    local width = (maxx-minx)*w
    local height = (maxy-miny)*h
    if width > height then
        return width
    else
        return height
    end
end

-- converts 12 values in a 8 points box + 2 center points on the bases
-- coords: 12 values in "crop" 2D BBox coordinates [0, 1]
-- return: { points = { {x, y}, ... }, centerPoints = { bottom = {x, y}, top = {x, y} } }
function BBox3d.Convert6PointsToBBox3dData(coords,w,h)
    -- front/back points
    local lengths = { { coords[1], coords[2] }, { coords[3], coords[4] } }
    -- left/right points
    local widths = { { coords[5], coords[6] }, { coords[7], coords[8] } }
    -- top/bottom points
    local heights = { { coords[9], coords[10] }, { coords[11], coords[12] } }

    -- calculate the center of the 3D BBox by the mean of all values
    local superposition = { 0.0, 0.0 }
    for i = 1, #coords, 2 do
        superposition[1] = superposition[1] + coords[i]
        superposition[2] = superposition[2] + coords[i + 1]
    end

    superposition[1] = superposition[1] / 6
    superposition[2] = superposition[2] / 6

    local offset1x = superposition[1] - widths[1][1] + superposition[1] - heights[1][1]
    local offset1y = superposition[2] - widths[1][2] + superposition[2] - heights[1][2]
    local offset2x = superposition[1] - widths[2][1] + superposition[1] - heights[1][1]
    local offset2y = superposition[2] - widths[2][2] + superposition[2] - heights[1][2]
    local p1x = lengths[1][1] + offset1x
    local p1y = lengths[1][2] + offset1y
    local p2x = lengths[1][1] + offset2x
    local p2y = lengths[1][2] + offset2y
    local p3x = lengths[2][1] + offset1x
    local p3y = lengths[2][2] + offset1y
    local p4x = lengths[2][1] + offset2x
    local p4y = lengths[2][2] + offset2y

    local offset3x = superposition[1] - widths[1][1] + superposition[1] - heights[2][1]
    local offset3y = superposition[2] - widths[1][2] + superposition[2] - heights[2][2]
    local offset4x = superposition[1] - widths[2][1] + superposition[1] - heights[2][1]
    local offset4y = superposition[2] - widths[2][2] + superposition[2] - heights[2][2]
    local p5x = lengths[1][1] + offset3x
    local p5y = lengths[1][2] + offset3y
    local p6x = lengths[1][1] + offset4x
    local p6y = lengths[1][2] + offset4y
    local p7x = lengths[2][1] + offset3x
    local p7y = lengths[2][2] + offset3y
    local p8x = lengths[2][1] + offset4x
    local p8y = lengths[2][2] + offset4y

    -- change the plane order
    if BBox3d.algorithmSettings.pickBottomPlane then
        local points = {{p1x,p1y}, {p2x,p2y}, {p3x,p3y}, {p4x,p4y}, {p5x,p5y}, {p6x,p6y}, {p7x,p7y}, {p8x,p8y}}

        --use 3 possible ordering options
        local o1 = {1,2,3,4,5,6,7,8}--normal
        local o2 = {1,2,5,6,3,4,7,8}--fronts
        local o3 = {1,3,5,7,2,4,6,8}--sides
        local options = {o1,o2,o3}

        -- discard the smallest plane based on its biggest side
        -- discard the smallest plane based on its biggest side
        local smallestSide = 999
        local smallestOption = -1
        for i=1, #options, 1 do
            local planePoints = {}
            for j=1, 4, 1 do
                table.insert(planePoints,points[options[i][j]])
            end
            local side = BBox3d.BiggestSide(planePoints,w,h)
            if side < smallestSide then
                smallestOption = i
                smallestSide = side
            end
        end
        -- table.remove(options,smallestOption)

        -- find lowest plane center
        local lowest_center = 999
        local chosen_option = -1
        for i=1, #options, 1 do
            if i ~= smallestOption then
                local centerBottomY = (points[options[i][1]][2] + points[options[i][2]][2] + points[options[i][3]][2] + points[options[i][4]][2]) / 4
                local centerTopY = (points[options[i][5]][2] + points[options[i][6]][2] + points[options[i][7]][2] + points[options[i][8]][2]) / 4
                if centerBottomY<lowest_center then
                    chosen_option = i
                    lowest_center=centerBottomY
            end
            if centerTopY<lowest_center then
                elseif centerTopY<lowest_center then
                    chosen_option = i
                    lowest_center=centerTopY
                end
            end
        end

        --reorder points based on chosen plane order
        local sorted = {}
        for i=1,#options[chosen_option],1 do
            table.insert(sorted,points[options[chosen_option][i]])
        end
        p1x = sorted[1][1]
        p1y = sorted[1][2]
        p2x = sorted[2][1]
        p2y = sorted[2][2]
        p3x = sorted[3][1]
        p3y = sorted[3][2]
        p4x = sorted[4][1]
        p4y = sorted[4][2]
        p5x = sorted[5][1]
        p5y = sorted[5][2]
        p6x = sorted[6][1]
        p6y = sorted[6][2]
        p7x = sorted[7][1]
        p7y = sorted[7][2]
        p8x = sorted[8][1]
        p8y = sorted[8][2]
        --end change plane order
    end

    local centerBottom = { (p1x + p2x + p3x + p4x) / 4, (p1y + p2y + p3y + p4y) / 4 }
    local centerTop = { (p5x + p6x + p7x + p8x) / 4, (p5y + p6y + p7y + p8y) / 4 }

    -- it's not guarantee that the first set of points are of the bottom plane
    if (centerTop[2] > centerBottom[2]) then
        centerBottom, centerTop = centerTop, centerBottom
        p1x, p5x = p5x, p1x
        p2x, p6x = p6x, p2x
        p3x, p7x = p7x, p3x
        p4x, p8x = p8x, p4x
        p1y, p5y = p5y, p1y
        p2y, p6y = p6y, p2y
        p3y, p7y = p7y, p3y
        p4y, p8y = p8y, p4y
    end

    local centerPoints = {
        bottom = centerBottom,
        top = centerTop,
    }

    return { -- 3/4 and 7/8 are swapped
        points = { { p1x, p1y }, { p2x, p2y }, { p4x, p4y }, { p3x, p3y }, { p5x, p5y }, { p6x, p6y }, { p8x, p8y }, { p7x, p7y } },
        centerPoints = centerPoints
    }
end

function BBox3d.GetBottomPlane(box3dData)
    local points = box3dData.points
    return { points[1], points[2], points[3], points[4] }
end

-- convert 3d bboxes to screen space coords
---@param bboxData table
---@param cropInfo Rect
---@param clip boolean
---@return BBox3d
function BBox3d.ConvertBBox3dDataToScreenSpace(bboxData, cropInfo, clip)
    local points = {}
    for i = 1, 8 do
        points[i] = {
            bboxData.points[i][1] * cropInfo.width + cropInfo.x,
            bboxData.points[i][2] * cropInfo.height + cropInfo.y
        }
    end

    local c1 = bboxData.centerPoints.bottom
    local c2 = bboxData.centerPoints.top
    local centerPoints = {
        bottom = { c1[1] * cropInfo.width + cropInfo.x, c1[2] * cropInfo.height + cropInfo.y },
        top    = { c2[1] * cropInfo.width + cropInfo.x, c2[2] * cropInfo.height + cropInfo.y }
    }

    if clip == true then
        for i = 1, 8 do
            points[i][1] = math.max(0, math.min(1.0, points[i][1]))
            points[i][2] = math.max(0, math.min(1.0, points[i][2]))
        end
        centerPoints.bottom[1] = math.max(0, math.min(1.0, centerPoints.bottom[1]))
        centerPoints.bottom[2] = math.max(0, math.min(1.0, centerPoints.bottom[2]))
        centerPoints.top[1] = math.max(0, math.min(1.0, centerPoints.top[1]))
        centerPoints.top[2] = math.max(0, math.min(1.0, centerPoints.top[2]))
    end

    return {
        points = points,
        centerPoints = centerPoints
    }
end

---@class CenterPoints
---@field bottom number[]
---@field top number[]

---@class BBox3d
---@field points table<number, number>[]
---@field centerPoints CenterPoints

function BBox3d.draw(box3d, outputimg)

    local settings = BBox3d.drawSettings;

    if settings.bottomPlane == false and
        settings.topPlane == false and
        settings.full == false and
        settings.corners == false and
        settings.bottomCenter == false
    then
        return
    end

    -- draw the center of the bottom plane
    if settings.bottomCenter then
        outputimg:drawCircle(box3d.centerPoints.bottom, 4, settings.bottomCenterColor, -1)
    end

    -- draw the center of the top plane
    if settings.topCenter then
        outputimg:drawCircle(box3d.centerPoints.top, 4, settings.topCenterColor, -1)
    end

    local points = box3d.points

    -- draw the 8 corners
    if settings.corners then
        for i = 1, 8 do
            outputimg:drawCircle(points[i], 3, settings.cornersColor[i], -1)
        end
    end

    -- draw the bottom plane
    if settings.bottomPlane or settings.full then
        local color = table.move(settings.bottomPlaneColorAndTickness, 1, 3, 1, {})
        local tickness = settings.bottomPlaneColorAndTickness[4]
        for i = 1, 4 do
            outputimg:drawLine(points[i], points[(i % 4) + 1], color, tickness)
        end
    end

    -- draw the top plane
    if settings.topPlane or settings.full then
        local color = table.move(settings.topPlaneColorAndTickness, 1, 3, 1, {})
        local tickness = settings.topPlaneColorAndTickness[4]
        for i = 1, 4 do
            outputimg:drawLine(points[i + 4], points[(i % 4) + 5], color, tickness)
        end
    end

    -- draw the vertical lines connecting top and bottom planes
    if settings.full then
        local color = table.move(settings.verticalLinesColorAndTickness, 1, 3, 1, {})
        local tickness = settings.verticalLinesColorAndTickness[4]
        for i = 1, 4 do
            outputimg:drawLine(points[i], points[i + 4], color, tickness)
        end
    end
end

---Checks if 3d bounding boxes are enabled in the instance
---@param instance rt_instance
---@return boolean
function BBox3d.isEnabled(instance)
    return instance:getConfigValue("Global/Bbox3d/enabled") == true
end

---Enable/Disable 3d bounding boxes
---@param instance rt_instance
function BBox3d.setEnabled(instance,enabled)
    instance:setConfigValue("Global/Bbox3d/enabled",enabled)
end

---Gets bottom plane center from a 3d bounding box, or nil if the bbox is empty
---@param bbox3d table
---@return number[]
function BBox3d.getBottomPlaneCenter(bbox3d)
    
    if bbox3d == nil or next(bbox3d) == nil then
        api.logging.LogError("Trying to get bottom plane center of an empty bbox3d")
        return nil
    end

    return bbox3d.centerPoints.bottom
end

function BBox3d.hasBottomPlaneCenter(bbox3d)
    
    if bbox3d == nil or next(bbox3d) == nil then
        return false
    end

    return true
end