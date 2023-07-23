Faces = {

}

---@class Face
---@field x number
---@field y number
---@field width number
---@field height number
---@field confidence number

FaceDetectionEnabledKey = "Global/FaceDetection/enabled"
FaceDetectionBlurKey = "Global/FaceDetection/blur"
FaceDetectorKey = "Global/FaceDetection/FaceDetector"

--- Loads the face detector
---@param instance rt_instance
---@return boolean true if loaded
function Faces.LoadDetector(instance)
    local detectInst = api.factory.inference.get(instance, FaceDetectorKey)
    if detectInst == nil then
        local detectInst = api.factory.inference.create(instance, FaceDetectorKey)
        detectInst:loadModelFromConfig()
    end
    return true
end

--- Deletes the face detector
---@param instance rt_instance
function Faces.DeleteDetector(instance)
    deleteInferenceBackend(instance, FaceDetectorKey)
end

--- Checks if face detection is enabled
---@param instance rt_instance
---@return boolean
function Faces.IsFaceDetectionEnabled(instance)
    return instance:getConfigValue(FaceDetectionEnabledKey) == true
end

--- Checks if the face detector model has been loaded
---@param instance rt_instance
---@return boolean true if loaded
function Faces.IsModelLoaded(instance)
    local detectInst = api.factory.inference.get(instance, FaceDetectorKey)
    return detectInst ~= nil
end

--- Checks if blurring is enabled
---@param instance rt_instance
---@return boolean
function Faces.IsBlurringEnabled(instance)
    return instance:getConfigValue(FaceDetectionBlurKey) == true
end


--- Detects faces on image
---@param instance rt_instance
---@param personTracks Track[]
---@param image buffer
---@return Face[]
function Faces.Detect(instance,personTracks,image)

    local personBboxes = table.remap(personTracks, function(track)
        return track.bbox
    end)


    local faces = {}
    local peopleBoundingBox = { x = 0.0, y = 0.0, width = 1.0, height = 1.0 }
    if #personBboxes > 0 then
        peopleBoundingBox = boundingBox(personBboxes)
        peopleBoundingBox = TurnBoxSquare(peopleBoundingBox,image:getSize(),0,0)
        peopleBoundingBox = AdjustBoxToFitImageBoundaries(peopleBoundingBox,0)

        local detectInst = api.factory.inference.get(instance, FaceDetectorKey)
        faces = detectInst:runInference( {{ source = image, x = peopleBoundingBox.x, y = peopleBoundingBox.y, width = peopleBoundingBox.width, height = peopleBoundingBox.height }}, image)
    end
    return {faces,peopleBoundingBox}
end

--- Blurs the faces in the image
---@param faces Face[]
---@param outputimg buffer
function Faces.Blur(faces,outputimg)

    for _, face in pairs(faces) do
        local size = outputimg:getSize()
        -- prepare slightly bigger region for blur
        local offset_percent = 0.15
        local blur_area = {}
        blur_area.x = face.x - face.width * offset_percent
        blur_area.width = face.width + face.width * offset_percent * 2
        blur_area.y = face.y - face.height * offset_percent
        blur_area.height = face.height + face.height * offset_percent * 2
        -- fit in frame
        if (blur_area.x < 0) then
            blur_area.x = 0
        end
        if (blur_area.y < 0) then
            blur_area.y = 0
        end
        if (blur_area.x + blur_area.width > 1) then
            blur_area.width = 1 - blur_area.x
        end
        if (blur_area.y + blur_area.height > 1) then
            blur_area.height = 1 - blur_area.y
        end
        
        outputimg:blur(blur_area.x, blur_area.y, blur_area.width, blur_area.height, 0.06 * (blur_area.width*size[1] + blur_area.height*size[2]))
    end

end