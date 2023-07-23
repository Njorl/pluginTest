
GeomUtils = {}

---Calculate dot product of two vectors
---@param a Vec2
---@param b Vec2
---@return number
function GeomUtils.Dot(a, b)
    return (a.x * b.x) + (a.y * b.y)
end

---Normalize a vector
---@param vec Vec2
---@return Vec2
function GeomUtils.Norm(vec)
    local mag = math.sqrt(vec.x^2 + vec.y^2)
    return {x = vec.x / mag, y = vec.y / mag}
end

---Move a rectangle by a delta
---@param rect Rect
---@param delta Vec2
---@return Rect
function GeomUtils.MoveRect(rect,delta)
    local retRect = CopyRect(rect)
    retRect.x = retRect.x + delta.x
    retRect.y = retRect.y + delta.y
    return retRect
end

function ClosetPointOnSegment(v,a,b)

    local ab = {b[1] - a[1],b[2] - a[2]}
    local av = {v[1] - a[1],v[2] - a[2]}
    local  i = GeomUtils.Dot(av,ab) /(ab[1]^2+ab[2]^2)
     i = math.max(math.min(i,1),0) --aka clamp(i,0,1)
     return {a[1] + ab[1]*i, a[2] + ab[2]*i} 
end


---Calculate the IoU of two rectangles
---@param box1 Rect
---@param box2 Rect
---@return number
function GeomUtils.Iou(box1, box2)

    local iou = 0.0
    local x0 = math.max(box1.x, box2.x)
    local x1 = math.min(box1.x + box1.width, box2.x + box2.width)
    if x0 < x1 then

        local y0 = math.max(box1.y, box2.y)
        local y1 = math.min(box1.y + box1.height, box2.y + box2.height)
        if y0 < y1 then

            local box1Area = box1.width * box1.height
            local box2Area = box2.width * box2.height

            local intersectionArea = (x1-x0) * (y1-y0)
            local unionArea = (box1Area + box2Area) - intersectionArea
            
            iou = intersectionArea / unionArea
        end
    end

    return iou

end

---Calculates the containment factor of box2 inside box1 (intersection over box1)
---@param box1 any
---@param box2 any
function BoxContainment(box1,box2)
    
        local x0 = math.max(box1.x, box2.x)
        local x1 = math.min(box1.x + box1.width, box2.x + box2.width)
        if x0 < x1 then
    
            local y0 = math.max(box1.y, box2.y)
            local y1 = math.min(box1.y + box1.height, box2.y + box2.height)
            if y0 < y1 then
    
                local box1Area = box1.width * box1.height
    
                local intersectionArea = (x1-x0) * (y1-y0)
                
                return intersectionArea / box1Area
            end
        end
    
        return 0.0

end


function boundingBox(rectangles)
    local min_x = rectangles[1].x
    local min_y = rectangles[1].y
    local max_x = min_x + rectangles[1].width
    local max_y = min_y + rectangles[1].height

    for _, rectangle in ipairs(rectangles) do
        min_x = math.min(min_x,rectangle.x)
        min_y = math.min(min_y,rectangle.y)
        max_x = math.max(max_x,rectangle.x + rectangle.width)
        max_y = math.max(max_y,rectangle.y + rectangle.height)
    end

    return {x = min_x, y = min_y, width = max_x - min_x, height = max_y - min_y}
end

function distance(pos1, pos2)
    return math.sqrt((pos2.x-pos1.x)^2 + (pos2.y-pos1.y)^2)
end


function GetShapeBoundingBox(zone_shape)
    local min_x = zone_shape[1][1]
    local min_y = zone_shape[1][2]
    local max_x = zone_shape[1][1]
    local max_y = zone_shape[1][2]

    for _, pos in ipairs(zone_shape) do
        min_x = math.min(min_x,pos[1])
        min_y = math.min(min_y,pos[2])
        max_x = math.max(max_x,pos[1])
        max_y = math.max(max_y,pos[2])
    end

    --return {left = math.floor(min_x), top = math.floor(min_y), bottom = math.ceil(max_y), right = math.ceil(max_x), x = math.floor(min_x), y = math.floor(min_y), width = math.ceil(max_x - min_x), height = math.ceil(max_y - min_y) }
    return {left = min_x, top = min_y, bottom = max_y, right = max_x, x = min_x, y = min_y, width = max_x - min_x, height = max_y - min_y }
end

---Get a bounding box for a list of tracks
---@param tracks Track[]
function GetTracksBoundingBox(tracks)

    local minX = math.huge
    local minY = math.huge
    local maxX = -math.huge
    local maxY = -math.huge

    for _, track in ipairs(tracks) do
        local trackBBox = track.bbox
        if trackBBox then
            minX = math.min(minX, trackBBox.x)
            minY = math.min(minY, trackBBox.y)
            maxX = math.max(maxX, trackBBox.x + trackBBox.width)
            maxY = math.max(maxY, trackBBox.y + trackBBox.height)
        end
    end
    return {left = minX, top = minY, bottom = maxY, right = maxX, x = minX, y = minY, width = maxX - minX, height = maxY - minY }

end


--- Get a union of two bounding boxes
---@param rect1 Rect
---@param rect2 Rect
---@return Rect
function GeomUtils.RectUnion(rect1,rect2)

    local boxUnionMaxCoords = {x1 = math.min(rect1.x,rect2.x),y1 = math.min(rect1.y,rect2.y),x2 = math.max(rect1.x+rect1.width,rect2.x+rect2.width),y2 = math.max(rect1.y+rect1.height,rect2.y+rect2.height)}

    local boxUnion = {x = boxUnionMaxCoords.x1, y = boxUnionMaxCoords.y1, width = boxUnionMaxCoords.x2-boxUnionMaxCoords.x1, height = boxUnionMaxCoords.y2-boxUnionMaxCoords.y1}

    return boxUnion
end

function IsPointInPolygon(x, y, poly)
    local x1, y1, x2, y2
    local len = #poly
    x2, y2 = poly[len - 1][1], poly[len][2]
    local wn = 0
    for idx = 1, len do
      x1, y1 = x2, y2
      x2, y2 = poly[idx][1], poly[idx][2]
  
      if y1 > y then
        if (y2 <= y) and (x1 - x) * (y2 - y) < (x2 - x) * (y1 - y) then
          wn = wn + 1
        end
      else
        if (y2 > y) and (x1 - x) * (y2 - y) > (x2 - x) * (y1 - y) then
          wn = wn - 1
        end
      end
    end
    return wn % 2 ~= 0 -- even/odd rule
  end

  
---Returns an adjusted box for inference purposes, by ensuring it stays within the image boundaries
---@param box table Box to be adjusted
function AdjustBoxToFitImageBoundaries(box, minSize)

    local imgSize = {1.0, 1.0}
    
    local xc = box.x + box.width*0.5
    local yc = box.y + box.height*0.5
    local w = math.min(math.max(box.width, minSize), imgSize[1])
    local h = math.min(math.max(box.height, minSize), imgSize[2])

    local rightTruncation = math.max(0,(xc + w*0.5) - imgSize[1])
    local leftTruncation = -math.min(0,(xc - w*0.5))
    local bottomTruncation = math.max(0,(yc + h*0.5) - imgSize[2])
    local topTruncation = -math.min(0,(yc - h*0.5))

    xc = xc - rightTruncation + leftTruncation
    yc = yc - bottomTruncation + topTruncation
    
    local r = { x = xc - w * 0.5, y = yc - h * 0.5, width = w, height = h }
    return r
end

---comment
---@param box Rect
function TrimBoxToFitImageBoundaries(box)

    local xMin = math.max(0, box.x)
    local yMin = math.max(0, box.y)
    local xMax = math.min(1, box.x + box.width)
    local yMax = math.min(1, box.y + box.height)

    box.x = xMin
    box.y = yMin
    box.width = xMax - xMin
    box.height = yMax - yMin

    return box
end


function TurnBoxSquare(boxNormalizedCoordinates,imageSize,absPadding,absMinSize)
    
        local box = {x = boxNormalizedCoordinates.x * imageSize[1], y = boxNormalizedCoordinates.y * imageSize[2], width = boxNormalizedCoordinates.width * imageSize[1], height = boxNormalizedCoordinates.height * imageSize[2]}
    
        local xc = box.x + box.width*0.5
        local yc = box.y + box.height*0.5
        local w = box.width + absPadding
        local h = box.height + absPadding
        
        w = math.max(w, absMinSize)
        h = math.max(h, absMinSize)
    
        if w > h then
            h = w
        else
            w = h
        end
    
        local r = { x = xc - w * 0.5, y = yc - h * 0.5, width = w, height = h }
        
        --normalize box
        r.x = r.x / imageSize[1]
        r.y = r.y / imageSize[2]
        r.width = r.width / imageSize[1]
        r.height = r.height / imageSize[2]

        return r
end

function PadBoxAbs(boxNormalizedCoordinates,imageSize,absPadding)
    
    local box = {x = boxNormalizedCoordinates.x * imageSize[1], y = boxNormalizedCoordinates.y * imageSize[2], width = boxNormalizedCoordinates.width * imageSize[1], height = boxNormalizedCoordinates.height * imageSize[2]}

    local xc = box.x + box.width*0.5
    local yc = box.y + box.height*0.5
    local w = box.width + absPadding
    local h = box.height + absPadding
    
    local r = { x = xc - w * 0.5, y = yc - h * 0.5, width = w, height = h }
    
    --normalize box
    r.x = r.x / imageSize[1]
    r.y = r.y / imageSize[2]
    r.width = r.width / imageSize[1]
    r.height = r.height / imageSize[2]

    return r
end

---Pad a box with a relative padding
---@param box Rect
---@param relPadding number
---@return Rect
function PadBoxRel(box,relPadding)

    box.x = box.x - relPadding
    box.y = box.y - relPadding
    box.width = box.width + relPadding * 2
    box.height = box.height + relPadding * 2

    return box
end

function GetAreaSizeForTriggerObjectSizeInt(objectSizeInt, inputimg)
    if objectSizeInt == nil then
        objectSizeInt = 0
    end
    local inputImageSize = inputimg:getSize()
    local coef = objectSizeInt / 10
    local minSizeW = math.floor(inputImageSize[1]* 0.7 * coef)
    local minSizeH = math.floor(inputImageSize[2]* 0.7 * coef)
    -- return {minSizeW, minSizeH}
    return {minSizeW / inputImageSize[1], minSizeH / inputImageSize[2]}
end

function GetAreaSizeForTriggerObjectSizeString(objectSizeString, inputimg)
    local inputImageSize = inputimg:getSize()
    local sideLength = math.floor(inputImageSize[1]* 0.5) -- default / unknown

    if objectSizeString == MotionGuidedMaxObjectSize.Tiny then
        sideLength = math.floor(inputImageSize[1]* 0.1)
    elseif objectSizeString == MotionGuidedMaxObjectSize.Small then
        sideLength = math.floor(inputImageSize[1]* 0.2)
    elseif objectSizeString == MotionGuidedMaxObjectSize.Medium then
        sideLength = math.floor(inputImageSize[1]* 0.3)
    elseif objectSizeString == MotionGuidedMaxObjectSize.Large then
        sideLength = math.floor(inputImageSize[1]* 0.4)
    elseif objectSizeString == MotionGuidedMaxObjectSize.XLarge then
        sideLength = math.floor(inputImageSize[1]* 0.6)
    elseif objectSizeString == MotionGuidedMaxObjectSize.XXLarge then
        sideLength = math.floor(inputImageSize[1]* 0.7)
    end

    return {sideLength / inputImageSize[1], sideLength / inputImageSize[2]}
end

function EnsurePointIsInsideShape(point,shape)

    local shapeBBox = GetShapeBoundingBox(shape)

    local xmin = shapeBBox.x
    local xmax = shapeBBox.x + shapeBBox.width
    local ymin = shapeBBox.y
    local ymax = shapeBBox.y + shapeBBox.height

    local newPoint = {point[1],point[2]}


    if newPoint[1] < xmin then
        newPoint[1] = xmin
    end
    if newPoint[1] > xmax then
        newPoint[1] = xmax
    end

    
    if newPoint[2] < ymin then
        newPoint[2] = ymin
    end
    if newPoint[2] > ymax then
        newPoint[2] = ymax
    end
        
    return newPoint
end

function EnsureBoxIsInsideShape(box,shape)

    local shapeBBox = GetShapeBoundingBox(shape)
    local shapeXmin = shapeBBox.x
    local shapeXmax = shapeBBox.x + shapeBBox.width
    local shapeYmin = shapeBBox.y
    local shapeYmax = shapeBBox.y + shapeBBox.height

    local boxCenterX = box.x + box.width * 0.5
    local boxCenterY = box.y + box.height * 0.5
    local boxXmin = box.x
    local boxXmax = box.x + box.width
    local boxYmin = box.y
    local boxYmax = box.y + box.height

    if (box.width * box.height) > (shapeBBox.width * shapeBBox.height) then
        local boxCenterX = shapeBBox.x + shapeBBox.width * 0.5
        local boxCenterY = shapeBBox.y + shapeBBox.height * 0.5
        return {x = boxCenterX - box.width*0.5, y = boxCenterY - box.height*0.5, width = box.width, height = box.height }
    end

    if boxXmax > shapeXmax then
        local offset = (boxXmax - shapeXmax)
        boxCenterX = boxCenterX - (offset)
    elseif  boxXmin < shapeXmin then
        local offset = (shapeXmin - boxXmin)
        boxCenterX = boxCenterX + (offset)
    end 

    
    if boxYmax > shapeYmax then
        local offset = (boxYmax - shapeYmax)
        boxCenterY = boxCenterY - (offset)
    elseif  boxYmin < shapeYmin then
        local offset = (shapeYmin - boxYmin)
        boxCenterY = boxCenterY + (offset)
    end 

    return {x = boxCenterX - box.width*0.5, y = boxCenterY - box.height*0.5, width = box.width, height = box.height }
end


function ConvertVerticesToPairs(poly)
-- poly is a list of tables like {{"x" = 0.5, "y" = 0.6}} and this function 
-- converts it to a list of pairs like {{x1, y1}, {x2, y2}, ... {xn, yn}}
    local converted = {}
    for idx = 1, #poly, 1 do
        converted[idx] = {poly[idx]["x"], poly[idx]["y"]}
    end

    return converted
end


---By Pedro Gimeno, donated to the public domain, adapted by CVEDIA
---@param point number[]
---@param poly number[number[]]
---@return boolean
function IsPointInPolygon(point, poly)
-- poly is a list of "pairs" like {{x1, y1}, {x2, y2}, ... {xn, yn}}
    local x1, y1, x2, y2
    local polyLen = #poly
    x2, y2 = poly[polyLen][1], poly[polyLen][2]
    local wn = 0
    local x = point[1]
    local y = point[2]

    for idx = 1, polyLen, 1 do
        x1, y1 = x2, y2
        x2, y2 = poly[idx][1], poly[idx][2]

        if y1 > y then
            if (y2 <= y) and (x1 - x) * (y2 - y) < (x2 - x) * (y1 - y) then
            wn = wn + 1
            end
        else
            if (y2 > y) and (x1 - x) * (y2 - y) > (x2 - x) * (y1 - y) then
            wn = wn - 1
            end
        end
    end

    return wn % 2 ~= 0 -- even/odd rule
end

function NumberOfPointsInPolygon(points, poly)

    local count = 0
    for _, point in ipairs(points) do
        if IsPointInPolygon(point, poly) then
            count = count + 1
        end
    end
    return count
end

--- Converts a rectangle from normalized coordinates to absolute coordinates
---@param box Rect
---@param imgSize table<number, number>
---@return Rect
function GeomUtils.RectNormToAbs(box, imgSize)
    local x = box.x * imgSize[1]
    local y = box.y * imgSize[2]
    local width = box.width * imgSize[1]
    local height = box.height * imgSize[2]
    return {x = x, y = y, width = width, height = height}
end

--- Converts a rectangle from absolute coordinates to normalized coordinates
---@param box Rect
---@param imgSize table<number, number>
---@return Rect
function GeomUtils.RectAbsToNorm(box, imgSize)
    local x = box.x / imgSize[1]
    local y = box.y / imgSize[2]
    local width = box.width / imgSize[1]
    local height = box.height / imgSize[2]
    return {x = x, y = y, width = width, height = height}
end

--- Converts rectangles from absolute coordinates to normalized coordinates
---@param boxes Rect[]
---@param imgSize table<number, number>
---@return Rect[]
function GeomUtils.RectsAbsToNorm(boxes, imgSize)
    local newBoxes = {}
    for _, box in ipairs(boxes) do
        table.insert(newBoxes, GeomUtils.RectAbsToNorm(box, imgSize))
    end
    return newBoxes
end

--- Converts rectangles from normalized coordinates to absolute coordinates
---@param boxes Rect[]
---@param imgSize table<number, number>
---@return Rect[]
function GeomUtils.RectsNormToAbs(boxes, imgSize)
    local newBoxes = {}
    for _, box in ipairs(boxes) do
        table.insert(newBoxes, GeomUtils.RectNormToAbs(box, imgSize))
    end
    return newBoxes
end



function PadBoxRelFactor(box, imgSize, horizontalPaddingFactor, verticalPaddingFactor)

    local boxAbs = GeomUtils.RectNormToAbs(box,imgSize)
    local newWidth = boxAbs.width * horizontalPaddingFactor
    local newHeight = boxAbs.height * verticalPaddingFactor
    local newX = boxAbs.x - (newWidth - boxAbs.width) * 0.5
    local newY = boxAbs.y - (newHeight - boxAbs.height) * 0.5
    boxAbs = {x = newX, y = newY, width = newWidth, height = newHeight}
    return GeomUtils.RectAbsToNorm(boxAbs,imgSize)
end

---Merge redundant boxes by checking the iOU between them. If two boxes have an iOU greater than the threshold, they are merged by calculating the bounding box between the two.
---@param boxes Rect[]
---@param iouThreshold number
---@return Rect[]
function GeomUtils.MergeRedundantRects(boxes, iouThreshold)

    local mergeResult = _MergeRedundantBoxes(boxes, iouThreshold)
    local mergedBoxes = mergeResult[1]
    local numMerges = mergeResult[2]

    if numMerges > 0 then
        return GeomUtils.MergeRedundantRects(mergedBoxes, iouThreshold)
    else
        return mergedBoxes
    end

end

function _MergeRedundantBoxes(boxes, iouThreshold)

    local mergedBoxes = {}
    local mergedBoxesCount = 0
    local boxesCount = #boxes
    local numMerges = 0
    
    for i = 1, boxesCount, 1 do
        local box = boxes[i]
        local boxMerged = false
        for j = 1, mergedBoxesCount, 1 do
            local mergedBox = mergedBoxes[j]
            local iou = GeomUtils.Iou(box, mergedBox)
            if iou > iouThreshold then
                local newBox = GeomUtils.RectUnion(box, mergedBox)
                mergedBoxes[j] = newBox
                boxMerged = true
                numMerges = numMerges + 1
                break
            end
        end
        if not boxMerged then
            mergedBoxesCount = mergedBoxesCount + 1
            mergedBoxes[mergedBoxesCount] = box
        end
    end

    return {mergedBoxes, numMerges}

end