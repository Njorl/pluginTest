dofile(project_root .. "/geometry_utils.lua")
dofile(project_root .. "/lua_helpers.lua")
dofile(project_root .. "/enums.lua")
dofile(project_root .. "/3dbbox.lua")
dofile(project_root .. "/definitions.lua")
dofile(project_root .. "/trackmeta.lua")
dofile(project_root .. "/vehicle_classifier.lua")
dofile(project_root .. "/par_classifier.lua")


-- DebugTrackId = "VehicleTracker_1845" -- Set this to nil on production
DebugTrackId = nil -- Set this to nil on production


---Checks if a track is relevant to a given zone. This only checks for class compatibility
---@param track Track
---@param zoneOrTripwireConfig table
---@return boolean
function ShouldTriggerEventForTrackOnZoneOrTripwire(track,zoneOrTripwireConfig)

    local instance = api.thread.getCurrentInstance()
    if track ~= nil and zoneOrTripwireConfig ~= nil then

        if IsTrackLocked(track) ~= true then
            return false
        end

        if zoneOrTripwireConfig.ignore_stationary_objects == true and IsTrackMoving(track) ~= true then
            return false
        end

        local trackWidth = track.bbox.width
        local trackHeight = track.bbox.height

        --Check object size restrictions
        if zoneOrTripwireConfig.object_min_width ~= nil then
            if trackWidth < zoneOrTripwireConfig.object_min_width then
                return false
            end
        end

        if zoneOrTripwireConfig.object_min_height ~= nil then
            if trackHeight < zoneOrTripwireConfig.object_min_height then
                return false
            end
        end

        if zoneOrTripwireConfig.object_max_width ~= nil then
            if trackWidth > zoneOrTripwireConfig.object_max_width then
                return false
            end
        end

        if zoneOrTripwireConfig.object_max_height ~= nil then
            if trackHeight > zoneOrTripwireConfig.object_max_height then
                return false
            end
        end

        if IsTrackPerson(track) and zoneOrTripwireConfig.detect_people == true then

            if zoneOrTripwireConfig.restrict_person_attributes == true then
                if ParClassifier.KnowIfIsCarryingGun(track) then
                    return ParClassifier.IsCarryingGun(track) == zoneOrTripwireConfig["restric_person_attribute_CarryingGun"]
                else
                    return false
                end
            end

            return true
        elseif IsTrackVehicle(track) and zoneOrTripwireConfig.detect_vehicles == true then

            -- If Vehicle subclassification is enabled, wait for the vehicle classifier to classify the vehicle
            if VehicleClassifier.IsEnabled(instance) then
                if VehicleClassifier.HasVehicleClass(track) ~= true then
                    return false
                end
                -- Vehicle subclassification
                if zoneOrTripwireConfig.restrict_vehicle_type == true then
                    local vehicleClass = VehicleClassifier.GetVehicleClass(track)
                    -- If not specified in the config that we don't want a certain vehicle class, then we consider it relevant
                    if zoneOrTripwireConfig["detect_vehicle_"..vehicleClass] == true then
                        return true
                    else
                        return false
                    end
                end
            end


            return true
        elseif IsTrackAnimal(track) == true and zoneOrTripwireConfig.detect_animals == true then
            return true
        elseif IsTrackUnknown(track) == true and zoneOrTripwireConfig.detect_unknowns == true and IsTrackMoving(track) == true then
            return true
        end
    end
    return false
end

function isDetectionRelevantToZoneOrTripwire(detection,zoneOrTripwireConfig)

    if detection ~= nil and zoneOrTripwireConfig ~= nil then
        if detection.label == ClassLabels.Person and zoneOrTripwireConfig.detect_people == true then
            return true
        elseif detection.label == ClassLabels.Vehicle and zoneOrTripwireConfig.detect_vehicles == true then
            return true
        elseif detection.label == ClassLabels.Animal and zoneOrTripwireConfig.detect_animals == true then
            return true
        end
    end
    return false
end

function MatchBestIoU(box, candidates, iouThreshold)
    iouThreshold = iouThreshold or 0.5
    local iouMax, idxMax, boxArea = 0, -1, box.width * box.height
    for idx, candidate in pairs(candidates) do
        local x0 = math.max(box.x, candidate.x)
        local x1 = math.min(box.x + box.width, candidate.x + candidate.width)
        if x0 >= x1 then goto continue end

        local y0 = math.max(box.y, candidate.y)
        local y1 = math.min(box.y + box.height, candidate.y + candidate.height)
        if y0 >= y1 then goto continue end

        local unionArea = (x1-x0) * (y1-y0)
        local candidateArea = candidate.width * candidate.height
        local iou = unionArea / (candidateArea + boxArea - unionArea)

        if iou > iouMax and iou > iouThreshold then
            iouMax = iou
            idxMax = idx
        end

        ::continue::
    end

    if idxMax < 0 then return nil end
    return { iou = iouMax, idx = idxMax, box = {x = candidates[idxMax].x, y = candidates[idxMax].y, width = candidates[idxMax].width, height = candidates[idxMax].height } }
end

--- This function updates the track's confidence field
---@param trackerInst    trackermanaged
---@param tracks    Track[]
---@param sourceBoxes    Detection[]
function UpdateTracksConfidence(tracks,trackerInst,sourceBoxes)

    local matchedTracks = trackerInst:getMatchedTracks()
    local matchedTracksMap = {}

    for _, matchPair in ipairs(matchedTracks) do
        matchedTracksMap[matchPair[1]] = matchPair[2]
    end

    for _, track in pairs(tracks) do

        local sourceTrackId = track.sourceTrackerTrackId

        if matchedTracksMap[sourceTrackId] ~= nil then
            -- Update the track's confidence based on the latest detection
            local detectionIdx = matchedTracksMap[sourceTrackId] + 1
            local detection = sourceBoxes[detectionIdx]
            TrackMeta.setValue(track.id,"confidence",detection.confidence)
        else
            TrackMeta.setValue(track.id,"confidence",0.0)
        end

    end


end

--- Gets regions for the object detector based on the inference strategy configured
--- @param instance rt_instance RT instance
--- @param inputimg buffer Input image buffer
--- @param motionRegions table
--- @param isDetectionEnabled boolean
--- @param detectInst inferencemanaged
--- @param tracks Track[]
--- @return DetectorRegionsInfo
function GetDetectorRegionsInfoFromInferenceStrategy(instance,inputimg,isDetectionEnabled,motionRegions,detectInst,tracks)

    local inputImgSize = inputimg:getSize()

    if not isDetectionEnabled then

        local detectorRegionsInfo = {}
        detectorRegionsInfo.isUsingAtlasPacking = false
        detectorRegionsInfo.regions = {}

        return detectorRegionsInfo
    end

    local detectionInferenceStrategy = instance:getConfigValue("Global/Detection/inference_strategy")

    if detectionInferenceStrategy == InferenceStrategy.FullFrameInference then

        local detectorRegionsInfo = {}
        detectorRegionsInfo.isUsingAtlasPacking = false
        detectorRegionsInfo.regions = {{ source = inputimg, x = 0.0, y = 0.0, width = 1.0, height = 1.0 }}

        return detectorRegionsInfo
    elseif detectionInferenceStrategy == InferenceStrategy.MotionGuided then

        local regionPadding = instance:getConfigValue("Global/Detection/motion_guided_settings/region_padding")


        local useVehicleDetectionsForTexturePacking = instance:getConfigValue("Global/Detection/motion_guided_settings/use_vehicle_detections_on_texture_packing") == true

        --Both detections and motion regions are considered for the texture packing
        local detectionRegions = {}

        for _, track in pairs(tracks) do
            if IsTrackPerson(track) then
                local personBbox = track.bbox
                table.insert(detectionRegions,personBbox)
            elseif useVehicleDetectionsForTexturePacking and IsTrackVehicle(track) then
                table.insert(detectionRegions,track.bbox)
            end
        end

        for _, motionRegion in ipairs(motionRegions) do
            table.insert(detectionRegions,motionRegion)
        end

        -- Merge redundant boxes
        local absDetectionRegions = GeomUtils.RectsNormToAbs(detectionRegions, inputImgSize)
        absDetectionRegions = GeomUtils.MergeRedundantRects(absDetectionRegions,0.01)
        detectionRegions = GeomUtils.RectsAbsToNorm(absDetectionRegions, inputImgSize)


        if #detectionRegions > 0 then

            ---@type buffer
            local atlas = nil

            --Pad the boxes and adjust them to fit the image boundaries
            for _, detectionRegion in pairs(detectionRegions) do
                detectionRegion = PadBoxRel(detectionRegion,regionPadding)
                detectionRegion = TrimBoxToFitImageBoundaries(detectionRegion)
            end

            atlas, locations = detectInst:packJobs(detectionRegions, inputimg)

            local detectorRegionsInfo = {}
            detectorRegionsInfo.isUsingAtlasPacking = true
            detectorRegionsInfo.atlasPackingInfo = { sourceRegions = detectionRegions, atlasRegions = locations, atlas = atlas}
            detectorRegionsInfo.regions = {{ source = atlas, x = 0.0, y = 0.0, width = 1.0, height = 1.0 }}

            return detectorRegionsInfo
        else
            local detectorRegionsInfo = {}
            detectorRegionsInfo.isUsingAtlasPacking = false
            detectorRegionsInfo.regions = {}

            return detectorRegionsInfo
        end
    end


end

---comment
---@param tracks Track[]
---@param requireLock boolean
---@return table
function GetBboxesFromTracks(tracks, requireLock)

    local tracks_bbox = {}
    local captureExtraTrackInfo = instance:getConfigValue("Global/Debug/capture_extra_track_info") == true

    if DebugTrackId ~= nil then
        tracks = table.filter(tracks, function(track)
            -- return IsTrackPerson(track)
            return track.id == DebugTrackId
        end)
        if #tracks > 0 then
            local instance = api.thread.getCurrentInstance()
            instance:setPause(true)
        end
    end



    for _, track in pairs(tracks) do

        local trackEvents = TrackMeta.getValue(track.id,"events")
        local bbox = CopyRect(track.bbox)

        if requireLock ~= true or (requireLock == true and IsTrackLocked(track)) then

            local trackLabel = track.classLabel

            if VehicleClassifier.HasVehicleClass(track) then
                bbox.vehicle_class = VehicleClassifier.GetVehicleClass(track)
                trackLabel = bbox.vehicle_class
            end

            if ParClassifier.KnowIfIsCarryingGun(track) then
                bbox.armed = ParClassifier.IsCarryingGun(track)
            end

            bbox.label = track.id

            if trackEvents ~= nil then
                bbox.events = trackEvents
            end

            bbox.trackId = track.id
            bbox.externalId = track.externalId

            bbox.class_label = getTrackClass(track)
            bbox.is_moving = IsTrackMoving(track)

            bbox.has_tentative_event = (TrackMeta.getValue(track.id,"has_tentative_event") == true)
            bbox.track_age = TrackMeta.getValue(track.id,"age")

            if captureExtraTrackInfo then
                bbox.atlas_crop = track.atlas_crop
                bbox.atlas = track.atlas
                bbox.accum_movement = TrackMeta.getValue(track.id,"accumMovement")
            end

            if TrackMeta.getValue(track.id,"confidence") ~= nil then
                bbox.confidence = TrackMeta.getValue(track.id,"confidence")
            end

            table.insert(tracks_bbox, bbox)
        end

    end
    return tracks_bbox
end

function GetMatchedTracksBboxesFromTracker(trackerInst)
    local tracks_bbox = {}
    local matchedTracksInfo = trackerInst:getMatchedTracks()
    for _, info in ipairs(matchedTracksInfo) do
        table.insert(tracks_bbox, trackerInst:getTrackValue(info[1], "bbox"))

    end
    return tracks_bbox
end

function GetMovingTracksBboxesFromTracker(trackerInst)
    local tracks_bbox = {}
    local trackIds = trackerInst:getTrackIds()
    for _, trackId in pairs(trackIds) do

        if IsTrackMoving(trackId) then
            table.insert(tracks_bbox, trackerInst:getTrackValue(trackId, "bbox"))
        end

    end
    return tracks_bbox
end


function getUnmatchedTracksBboxesFromTracker(trackerInst)
    local tracks_bbox = {}
    local unmatchedTracksInfo = trackerInst:getUnmatchedTracks()
    for _, trackId in ipairs(unmatchedTracksInfo) do
        table.insert(tracks_bbox,trackerInst:getTrackValue(trackId, "bbox"))
    end
    return tracks_bbox
end

function getBoundingBoxFromPath(path)

    local minX = math.huge
    local minY = math.huge
    local maxX = -math.huge
    local maxY = -math.huge

    for _, point in ipairs(path) do
        if point.x < minX then
            minX = point.x
        end
        if point.y < minY then
            minY = point.y
        end
        if point.x > maxX then
            maxX = point.x
        end
        if point.y > maxY then
            maxY = point.y
        end
    end

    return {xMin = minX, yMin = minY, xMax = maxX, yMax = maxY}
end

LastUpdateTrackMovementStatusTimeSec = 0

---Updates tracks movement status
---This goes through every track and keeps track of its recent path
---If both the top-left and bottom-right corners of the track's bounding box move more than a certain distance, the track is considered moving
---If the track is moving, the field .moving is set to true
---This function is called twice per second
---@param instance rt_instance
---@param trackerInst trackermanaged
---@param tracks Track[]
---@param currentTimeSec number
---@param inputSize number[]
function UpdateTracksMovementStatus(instance,tracks,trackerInst,currentTimeSec,inputSize)

    local vehicleMovementThreshold = instance:getConfigValue("Movement/vehicle_movement_threshold")
    local personMovementThreshold = instance:getConfigValue("Movement/person_movement_threshold")
    local animalMovementThreshold = instance:getConfigValue("Movement/animal_movement_threshold")
    local unknownMovementThreshold = instance:getConfigValue("Movement/unknown_movement_threshold")


    for _, track in pairs(tracks) do

        local trackId = track.id

        -- If track is already considered to be moving, ignore it
        if IsTrackMoving(track) then
            goto continue
        end

        TrackMeta.setValue(trackId,"lastBbox",track.bbox)
        local movementDirection = track.movementDirection
        local accumMovement = TrackMeta.getValue(trackId,"accumMovement")
        accumMovement = accumMovement or {x=0,y=0}

        local bboxHistory = TrackMeta.getValue(trackId,"bboxHistory")
        bboxHistory = bboxHistory or {}
        table.insert(bboxHistory,track.bbox)
        TrackMeta.setValue(trackId,"bboxHistory",bboxHistory)

        --Update accumMovement
        if movementDirection ~= nil then
            accumMovement = {x=accumMovement.x + movementDirection.x, y=accumMovement.y + movementDirection.y}
            TrackMeta.setValue(trackId,"accumMovement",accumMovement)
        end

        local relativeMovementTreshold = unknownMovementThreshold

        if IsTrackPerson(track) then
            relativeMovementTreshold = personMovementThreshold
        elseif IsTrackVehicle(track) then
            relativeMovementTreshold = vehicleMovementThreshold
        elseif IsTrackAnimal(track) then
            relativeMovementTreshold = animalMovementThreshold
        end

        if math.abs(accumMovement.x) + math.abs(accumMovement.y) > relativeMovementTreshold then
            TrackMeta.setValue(trackId,"moving", true)

            -- Since we already know the track is moving, we can disable the CPU-intensive feature tracking feature for it
            trackerInst:saveTrackValue(track.sourceTrackerTrackId, "track_features", false)
        end


        ::continue::
    end

end

AnchorPointWarnRaised = false;

---Gets an anchor point from an object either from the 2D or 3D bounding box
---@param bbox2d table
---@param bbox3d table
---@param boxAnchor string
function GetAnchorPointFromObject(bbox2d,bbox3d,boxAnchor)
    local x = bbox2d.x
    local y = bbox2d.y
    if boxAnchor == BoxAnchor.Center then
        x = x + bbox2d.width/2
        y = y + bbox2d.height/2
    elseif boxAnchor == BoxAnchor.TopRight then
        x = x + bbox2d.width
    elseif boxAnchor == BoxAnchor.BottomLeft then
        y = y + bbox2d.height
    elseif boxAnchor == BoxAnchor.BottomRight then
        x = x + bbox2d.width
        y = y + bbox2d.height
    elseif boxAnchor == BoxAnchor.TopCenter then
        x = x + bbox2d.width/2
    elseif boxAnchor == BoxAnchor.BottomCenter then
        x = x + bbox2d.width/2
        y = y + bbox2d.height
    elseif boxAnchor == BoxAnchor.BottomPlaneCenter then
        if BBox3d.hasBottomPlaneCenter(bbox3d) then
            local bpc = BBox3d.getBottomPlaneCenter(bbox3d)
            x = bpc[1]
            y = bpc[2]
        else
            if not AnchorPointWarnRaised then
                AnchorPointWarnRaised = true
                api.logging.LogWarning("No 3D Bounding Box available for object, falling back to 2D Bounding Box bottom center")
            end
            return GetAnchorPointFromObject(bbox2d,bbox3d,BoxAnchor.BottomCenter)
        end
    end

    return {x = x, y = y}
end

---writes Tracks to disk in MOT Format 1.1
---@param writeInst writedatamanaged
---@param frameNumber number
---@param tracks table
---@param filePath string
---@param imageSize number[]
function writeTracksToDiskInMOTFormat(writeInst,frameNumber,tracks, filePath,imageSize)

    for _, track in ipairs(tracks) do
        if track ~= nil then
            local line =
            tostring(frameNumber) .. "," ..
                    tostring(track.id) .. "," ..
                    tostring(track.bbox.x*imageSize[1]) .. "," ..
                    tostring(track.bbox.y*imageSize[2]) .. "," ..
                    tostring(track.bbox.width*imageSize[1]) .. "," ..
                    tostring(track.bbox.height*imageSize[2]) .. "," ..
                    "0,0,0,0"
            print("Appending " .. line .. " to "..filePath)
            writeInst:appendText(filePath, line, "\n")
        end
    end
end

---get detector key based on modality
---@param instance rt_instance
---@return string
function getDetectorKeyBasedOnModality(instance)
    local modality = instance:getConfigValue("Global/modality")
    local detectorKey = nil

    if modality == Modality.Thermal then
        detectorKey = PluginKeys.DetectorThermal
    elseif modality == Modality.RGB then
        detectorKey = PluginKeys.DetectorRGB
    else
        api.logging.LogError("Unknown modality: " .. inspect(modality)..", Please set Global/modality to either rgb or thermal")
        instance:setPause(true)
        return ""
    end

    return detectorKey
end

---Deletes inference backend
---@param instance rt_instance
---@param key string
function deleteInferenceBackend(instance,key)
    local inferenceInst = api.factory.inference.get(instance, key)
    if (inferenceInst ~= nil) then
        api.factory.inference.delete(instance, key)
    end
end

---create detector backend
---@param instance rt_instance
---@return inferencemanaged
function createDetectorBasedOnModality(instance)
    local detectorKey = getDetectorKeyBasedOnModality(instance)
    local detectInst = api.factory.inference.get(instance, detectorKey)

    if detectInst == nil then
        detectInst = api.factory.inference.create(instance, detectorKey)
        detectInst:loadModelFromConfig()
    end

    return detectInst
end

---Checks if track with id exists
---@param tracks Track[]
---@param trackId string
---@return boolean
function DoesTrackExist(tracks,trackId)
    for _, track in ipairs(tracks) do
        if track.id == trackId then
            return true
        end
    end

    return false
end

---Checks if motion should be deactivate based on the number of tracks
---@param instance rt_instance
---@param trackerInst trackermanaged
---@return boolean
function ShouldDeactivateMotion(instance,trackerInst)

    local maxNumTracks = instance:getConfigValue("Global/max_num_tracks_to_deactive_motion")
    local trackIds = trackerInst:getTrackIds()

    if #trackIds > maxNumTracks then
        return true
    else
        return false
    end
end

IsMotionActivate = true
MotionLastDeactivationTimestamp = nil

---Deactivates motion
---@param currentTimeSec number
function DeactivateMotion(currentTimeSec)
    api.logging.LogWarning("Deactivating motion due to too many tracks")
    IsMotionActivate = false
    MotionLastDeactivationTimestamp = currentTimeSec
end


function ActivateMotion()
    api.logging.LogInfo("Activating motion")
    IsMotionActivate = true
end

function IsMotionActive()
    return IsMotionActivate
end

---Checks if motion should be re-activated based on time elapsed since last deactivation
---@param instance rt_instance
---@param currentTimeSec number
function ShouldReactivateMotion(instance,currentTimeSec)
    local motionReactivationDelaySec = instance:getConfigValue("Global/motion_reactivation_delay_sec")
    if MotionLastDeactivationTimestamp ~= nil and (currentTimeSec - MotionLastDeactivationTimestamp) > motionReactivationDelaySec then
        return true
    else
        return false
    end
end

---Sets a new inference strategy temporarily. Can be reset back to default by calling ResetGlobalInferenceStrategy
---@param instance rt_instance
---@param inferenceStrategy string
function SetTemporaryGlobalInferenceStrategy(instance,inferenceStrategy)

    local originalInferenceStrategy = instance:getConfigValue("Global/Detection/inference_strategy")
    instance:setConfigValue("Global/Detection/original_inference_strategy",originalInferenceStrategy)
    instance:setConfigValue("Global/Detection/inference_strategy",inferenceStrategy)
end

---Resets the inference strategy for triggers to the original value
---@param instance rt_instance
function ResetGlobalInferenceStrategy(instance)

    instance:setConfigValue("Global/Detection/inference_strategy",instance:getConfigValue("Global/Detection/original_inference_strategy"))

end

---Checks if motion is required (any trigger with with detect unknowns, or classification is enabled)
---@param instance rt_instance
---@param zoneInst zonemanaged
---@param tripwireInst tripwiremanaged
function IsTrackingMotionRequired(instance,zoneInst,tripwireInst)

    local classificationSettings = instance:getConfigValue("Global/Classification")

    if classificationSettings.enabled == true then
        return true
    end

    local detectionSettings = instance:getConfigValue("Global/Detection")
    if detectionSettings.enabled == true then
        if detectionSettings.inference_strategy == InferenceStrategy.MotionGuided then
            return true
        end
    end

    -- Check if there is any zone that is expecting unknown objects
    local zoneIds = zoneInst:getZoneIds()
    for _,zoneId in pairs(zoneIds) do
        local zoneConfig = instance:getConfigValue("Zone/Zones/"..zoneId)
        if zoneConfig.detect_unknowns == true then
            return true
        end
    end

    local tripwireIds = tripwireInst:getTripwireIds()
    for _,tripwireId in pairs(tripwireIds) do
        local tripwireConfig = instance:getConfigValue("Tripwire/Tripwires/" .. tripwireId)
        if tripwireConfig.detect_unknowns == true then
            return true
        end
    end

    return false
end

---Reload detector if needed
---@param instance rt_instance
---@param detectInst inferencemanaged
---@param isDetectionEnabled boolean
function ReloadDetectorIfNeeded(instance,detectInst,isDetectionEnabled)
    if instance:getConfigValue("Global/reload_detector_required") == true then
        UpdateDetectorPreset(instance)
        api.logging.LogInfo("Reloading detector backend")
        detectInst:loadModelFromConfig()
        instance:setConfigValue("Global/reload_detector_required",nil)
    end

    if isDetectionEnabled and detectInst == nil then
        return createDetectorBasedOnModality(instance)
    end

    return detectInst
end


---Update settings for triggers to ensure they show up correctly in the UI
---@param instance rt_instance
---@param zoneInst zonemanaged
function UpdateTriggersUISettings(instance,zoneInst)

    local zones = zoneInst:getZones()
    for _,zone in pairs(zones) do
        local zoneConfig = instance:getConfigValue("Zone/Zones/"..zone.id)
        -- Object left/removed zones should not show up the object counter
        if (zoneConfig.trigger_when == "object_left" or zoneConfig.trigger_when == "object_removed") and zone.label_format ~= "None" then
            zone.label_format = "None"
            zoneInst:setZoneValue(zone.id, zone, 0);
        end
    end
end

---Retrieves detector regions defined by the user
---@param instance rt_instance
---@param inputimg buffer
---@return table
function GetUserDefinedDetectorRegions(instance,inputimg)

    if IsDetectionEnabled(instance) == false then
        return {}
    end

    local userDefinedDetectorRegions = instance:getInputShapes("DetectorRegions")

    for _, region in ipairs(userDefinedDetectorRegions) do
        region.source = inputimg
    end

    return userDefinedDetectorRegions
end

---Update track classes based on the results of the object classifier
---@param instance rt_instance
---@param tracks Track[]
---@param classifierInst inferencemanaged
---@param inputimg buffer
---@param currentTimeSec number
---@param isClassificationEnabled boolean
function UpdateTrackClassesBasedOnClassification(instance,tracks,classifierInst,inputimg,currentTimeSec,isClassificationEnabled)

    local classificationSettings = instance:getConfigValue("Global/Classification")

    if isClassificationEnabled ~= true then
        goto ret
    end

    local classificationJobs = {}

    for _, track in pairs(tracks) do

        local isTrackLocked = IsTrackLocked(track)

        if classificationSettings.require_locked_track == true and isTrackLocked == false then
            goto continue
        end

        local isTrackMoving = IsTrackMoving(track)

        if classificationSettings.require_moving_track == true and isTrackMoving == false then
            goto continue
        end

        local isTrackAlreadyClassified = TrackMeta.getValue(track.id,"classification_lock") == true

        -- print("classificationSettings "..inspect(classificationSettings))
        if isTrackAlreadyClassified and classificationSettings.periodic_reclassification ~= true then
            goto continue
        end

        local lastClassificationTime = TrackMeta.getValue(track.id,"last_classification_lock_time_sec")
        -- print("type(lastClassificationTime) "..inspect(type(lastClassificationTime)))
        if lastClassificationTime ~= nil then
            local timeSinceLastClassification = currentTimeSec - lastClassificationTime
            if isTrackAlreadyClassified and classificationSettings.periodic_reclassification == true and timeSinceLastClassification < classificationSettings.periodic_reclassification_time_sec then
                goto continue
            end
        end


        local trackBbox = CopyRect(track.bbox)
        trackBbox.trackId = track.id

        table.insert(classificationJobs,trackBbox)

        ::continue::
    end

    if #classificationJobs > 0 then
        local classificationResults = classifierInst:runInference(classificationJobs,inputimg)

        for idx, classificationResult in ipairs(classificationResults) do
            local classificationLabel = classificationResult.label
            local trackId = classificationJobs[idx].trackId
            local minHitsForLock = classificationSettings.min_hits_for_lock


            local lastClassificationLabel = TrackMeta.getValue(trackId,"last_classification_label")
            if lastClassificationLabel ~= classificationLabel then
                TrackMeta.setValue(trackId,"classification_hits",0)
            end
            TrackMeta.setValue(trackId,"last_classification_label",classificationLabel)
            local currentHits = TrackMeta.getValue(trackId,"classification_hits")
            local newHits = currentHits + 1
            TrackMeta.setValue(trackId,"classification_hits",newHits)

            if newHits >= minHitsForLock then

                if classificationLabel == ClassLabels.Background then
                    TrackMeta.setValue(trackId,"override_classification_label",ClassLabels.Unknown)
                else
                    TrackMeta.setValue(trackId,"override_classification_label",classificationLabel)
                end

                TrackMeta.setValue(trackId,"classification_lock",true)
                TrackMeta.setValue(trackId,"classification_hits",0)
                TrackMeta.setValue(trackId,"last_classification_lock_time_sec",currentTimeSec)

            end

        end
    end

    ::ret::

end


---create classifier backend
---@param instance rt_instance
---@return inferencemanaged
function createClassifierBasedOnModality(instance)
    local pluginKey = getClassifierKeyBasedOnModality(instance)
    local inferenceInst = api.factory.inference.get(instance, pluginKey)

    if inferenceInst == nil then
        inferenceInst = api.factory.inference.create(instance, pluginKey)
        inferenceInst:loadModelFromConfig()
    end

    return inferenceInst
end


---get detector key based on modality
---@param instance rt_instance
---@return string
function getClassifierKeyBasedOnModality(instance)
    local modality = instance:getConfigValue("Global/modality")
    local pluginKey = nil

    if modality == Modality.Thermal then
        pluginKey = PluginKeys.ClassifierThermal
    elseif modality == Modality.RGB then
        pluginKey = PluginKeys.ClassifierRGB
    else
        api.logging.LogError("Unknown modality: " .. inspect(modality)..", Please set Global/modality to either rgb or thermal")
        instance:setPause(true)
        return ""
    end

    return pluginKey
end


---Reload classifier if needed
---@param instance rt_instance
---@param classifierInst inferencemanaged
---@param isClassificationEnabled boolean
---@return inferencemanaged
function ReloadClassifierIfNeeded(instance,classifierInst,isClassificationEnabled)
    if instance:getConfigValue("Global/reload_classifier_required") == true then
        api.logging.LogInfo("Reloading classifier backend")
        classifierInst:loadModelFromConfig()
        instance:setConfigValue("Global/reload_classifier_required",nil)
    end

    if isClassificationEnabled and classifierInst == nil then
        return createClassifierBasedOnModality(instance)
    end

    return classifierInst
end


---Update 3d bounding boxes for tracks.
---@param instance rt_instance
---@param tracks Track[]
---@param boundingBox3dRegressorInst inferencemanaged
---@param inputimg buffer
---@param is3dbboxEnabled boolean
function UpdateTrack3dBoundingBoxes(instance,tracks,boundingBox3dRegressorInst,inputimg,is3dbboxEnabled)

    local boundingBox3dSettings = instance:getConfigValue("Global/Bbox3d")

    if is3dbboxEnabled ~= true then
        goto ret
    end

    local inferenceJobs = {}
    for _, track in pairs(tracks) do

        local is_people = IsTrackPerson(track)
        local is_animal = IsTrackAnimal(track)
        local is_vehicle = IsTrackVehicle(track)

        if (boundingBox3dSettings.run_on_people == true and is_people == true) or (boundingBox3dSettings.run_on_vehicles == true and is_vehicle == true) or (boundingBox3dSettings.run_on_animals == true and is_animal == true) then

            local trackBbox = CopyRect(track.bbox)
            trackBbox.trackId = track.id

            table.insert(inferenceJobs,trackBbox)
        end

    end

    if #inferenceJobs > 0 then

        BBox3d.ConvertToBBox3dData = BBox3d.Convert4PointsToBBox3dData
        local classificationResults = boundingBox3dRegressorInst:runInference(inferenceJobs,inputimg)

        for idx, box3d in ipairs(classificationResults) do
            local trackId = inferenceJobs[idx].trackId
            local bboxData = BBox3d.ConvertToBBox3dData(box3d.feat, box3d.job.width, box3d.job.height)
            local screenSpacePointsData = BBox3d.ConvertBBox3dDataToScreenSpace(bboxData, box3d.job, true)
            TrackMeta.setValue(trackId,"bbox3d",screenSpacePointsData)
        end
    end
    ::ret::

end

---Get 3d bounding boxes from tracker
---@param tracks Track[]
function Get3DBoundingBoxesFromTracks(tracks)
    local tracks3DBoundingBoxes = {}

    for _, track in pairs(tracks) do
        local boundingBox3D = TrackMeta.getValue(track.id,"bbox3d")
        if boundingBox3D ~= nil and next(boundingBox3D) ~= nil then
            table.insert(tracks3DBoundingBoxes, boundingBox3D)
        end
    end

    return tracks3DBoundingBoxes


end

---Calculates and returns stats of all zones
---@param zoneInst zonemanaged
---@param inputImage buffer
---@return ZoneStats
function GetZoneStats(zoneInst, inputImage)

    local numOccupiedZones = 0
    local numVacantZones = 0


    --iterate over all zones
    for _, zoneId in pairs(zoneInst:getZoneIds()) do
        local zone = zoneInst:getZoneById(zoneId)
        if zone ~= nil then
            if zone.cur_entries ~= nil and zone.cur_entries > 0 then
                numOccupiedZones = numOccupiedZones + 1
            else
                numVacantZones = numVacantZones + 1
            end
        end
    end


    local occupancyRate = 0.0
    if numOccupiedZones + numVacantZones > 0 then
        occupancyRate = numOccupiedZones/(numOccupiedZones+numVacantZones)
    end

    ---@type ZoneStats
    local zoneStats = {occupancyRate = occupancyRate, numOccupiedZones = numOccupiedZones, numVacantZones = numVacantZones}

    return {
        image = inputImage,
        data = zoneStats
    }
end


---Iterate through all tracks and update their locking status. Use IsTrackLocked to check if a given track is locked
---@param tracks Track[]
---@param trackerInst trackermanaged
---@param currentTimeSec number
function UpdateTracksLocking(tracks,trackerInst, currentTimeSec)

    local trackerLockingSettings = trackerInst:getConfig()["Locking"]

    local matchRatioThreshold = trackerLockingSettings.match_ratio_threshold
    local timeWindowDurationSec = trackerLockingSettings.time_window_duration_sec

    local mathedTracksInfo = trackerInst:getMatchedTracks()

    function hasTrackBeenMatched(trackId)
        for _, trackInfo in ipairs(mathedTracksInfo) do
            if trackInfo[1] == trackId then
                return true
            end
        end

        return false
    end


    for _, track in ipairs(tracks) do

        local trackId = track.id

        local trackBornTimeSec = TrackMeta.getValue(trackId,"born_time_sec")
        if trackBornTimeSec == nil then
            trackBornTimeSec = currentTimeSec
            TrackMeta.setValue(trackId,"born_time_sec",currentTimeSec)
        end

        local trackAgeSec = currentTimeSec - trackBornTimeSec
        TrackMeta.setValue(trackId,"age",trackAgeSec)

        -- Skip track if already lock
        if IsTrackLocked(track) then
            goto continue
        end

        local trackMatchHistory = TrackMeta.getValue(trackId,"match_history") or {}

--        api.logging.LogDebug(inspect(track))

        -- Discard old entries
        for idx, trackMatchHistoryEntry in ipairs(trackMatchHistory) do
            local entryTimestamp = trackMatchHistoryEntry.timestamp
            if currentTimeSec - entryTimestamp > timeWindowDurationSec then
                table.remove(trackMatchHistory,idx)
            end
        end

        local trackMatchedThisFrame = hasTrackBeenMatched(track.sourceTrackerTrackId)
        -- Add new entry
        local trackMatchHistoryEntry = {matched = trackMatchedThisFrame, timestamp = currentTimeSec}
        table.insert(trackMatchHistory,trackMatchHistoryEntry)

        --- Skip locking if track is too young or if it has not been matched this frame
        if trackAgeSec >= timeWindowDurationSec and trackMatchedThisFrame then


            -- Calculate match ratio
            local numMatched = 0
            local numTotal = 0
            for _, trackMatchHistoryEntry in ipairs(trackMatchHistory) do
                if trackMatchHistoryEntry.matched == true then
                    numMatched = numMatched + 1
                end
                numTotal = numTotal + 1
            end
            local matchRatio = numMatched/numTotal

            -- Update lock status
            local isLocked = matchRatio >= matchRatioThreshold
            TrackMeta.setValue(trackId,"locked",isLocked)
        end

        TrackMeta.setValue(trackId,"match_history",trackMatchHistory)

        ::continue::
    end

end

---Returns lock status of a track
---@param track Track
---@return boolean
function IsTrackLocked(track)
    local isLocked = TrackMeta.getValue(track.id,"locked") == true
    return isLocked
end

---Returns true if track is of an unknown class (e.g. motion)
---@param track Track
---@return boolean
function IsTrackUnknown(track)
    return track.classLabel == ClassLabels.Unknown
end

---Returns true if its a person track
---@param track Track
---@return boolean
function IsTrackPerson(track)
    return track.classLabel == ClassLabels.Person
end

---Returns true if its a vehicle track
---@param track Track
---@return boolean
function IsTrackVehicle(track)
    return track.classLabel == ClassLabels.Vehicle
end

---Returns true if its a animal track
---@param track Track
---@return boolean
function IsTrackAnimal(track)
    return track.classLabel == ClassLabels.Animal
end


---Returns true if track is moving
---@param track Track
---@return boolean
function IsTrackMoving(track)
    local isMoving = TrackMeta.getValue(track.id,"moving") == true
    return isMoving
end

---Returns true if track is moving
---@param track Track
---@return boolean
function IsTrackMotion(track)
    return track.classLabel == ClassLabels.Unknown
end


---Returns true if track is moving
---@param track Track
---@return boolean
function IsTrackMatchedLastFrame(track)

    return track.lastSeen < 0.001
end

---Reload 3d bounding box inference backend if needed (i.e. not yet loaded)
---@param instance rt_instance
---@param inferenceInst inferencemanaged
---@param is3dbboxEnabled boolean
---@return inferencemanaged
function Reload3DBboxIfNeeded(instance,inferenceInst,is3dbboxEnabled)

    if is3dbboxEnabled then
        inferenceInst = api.factory.inference.get(instance, PluginKeys.Bbox3d)

        if inferenceInst == nil then
            inferenceInst = api.factory.inference.create(instance, PluginKeys.Bbox3d)
            inferenceInst:loadModelFromConfig()
        end
    end

    return inferenceInst
end

---
---@param instance rt_instance
---@return boolean
function IsDetectionEnabled(instance)
    return instance:getConfigValue("Global/Detection/enabled")
end


--- Filters detections that are not overlaping with moving objects
---@param detections table
---@param motionRegions Rect[]
---@return table
function FilterDetectionsNotOverlappingMotionRegions(detections,motionRegions)

    local filteredDetections = {}


    for _, detection in ipairs(detections) do
        for _, motionRegion in ipairs(motionRegions) do
            if BoxContainment(detection,motionRegion) > 0.8 then
                table.insert(filteredDetections,detection)
                break
            end
        end
    end

    return filteredDetections

end

---Retrusn the inference strategy for the detector
---@param instance rt_instance
function GetDetectorInferenceStrategy(instance)
    return instance:getConfigValue("Global/Detection/inference_strategy")
end

---Calculates and returns stats of all zones
---@param tracks Track[]
---@param inputImage buffer
---@return GlobalObjectStats
function GetGlobalObjectStats(tracks, inputImage)

    local globalObjectStats = {numPeople = 0, numVehicles = 0, numAnimals = 0, numUnknown = 0}

    for _, track in ipairs(tracks) do

        if IsTrackLocked(track) then
            if IsTrackPerson(track) then
                globalObjectStats.numPeople = globalObjectStats.numPeople + 1
            elseif IsTrackVehicle(track) then
                globalObjectStats.numVehicles = globalObjectStats.numVehicles + 1
            elseif IsTrackAnimal(track) then
                globalObjectStats.numAnimals = globalObjectStats.numAnimals + 1
            elseif IsTrackUnknown(track) then
                globalObjectStats.numUnknown = globalObjectStats.numUnknown + 1
            end
        end
    end

    return {
        image = inputImage,
        data = globalObjectStats
    }
end


---Convert Packed Detections to Image Space
---@param atlasPackingInfo AtlasPackingInfo
---@param atlasDetections Detection[]
---@return Detection[]
function UnpackAtlasDetections(atlasPackingInfo, atlasDetections)


    local atlasRegions = atlasPackingInfo.atlasRegions
    local sourceRegions = atlasPackingInfo.sourceRegions

    -- Determine which atlas block does each detection belong to

    local detections = {}
    for idx, detection in ipairs(atlasDetections) do
        local atlasBlockIndex = -1

        local detectionCenterX = detection.x + detection.width/2
        local detectionCenterY = detection.y + detection.height/2

        for atlasBlockIdx, atlasLocation in ipairs(atlasRegions) do
            -- check if the center of the detection is inside the atlas block

            if detectionCenterX >= atlasLocation.x and detectionCenterX <= atlasLocation.x + atlasLocation.width and
                    detectionCenterY >= atlasLocation.y and detectionCenterY <= atlasLocation.y + atlasLocation.height then
                atlasBlockIndex = atlasBlockIdx
                break
            end
        end

        if atlasBlockIndex == -1 then

            goto continue
        end

        --Convert detection to atlas location space

        local atlasRegion = atlasRegions[atlasBlockIndex]
        local sourceRegion = sourceRegions[atlasBlockIndex]

        local detectionBoxImageSpace = ConvertAtlasBoxToImageSpace(detection,atlasRegion,sourceRegion)

        local detectionImageSpace = detectionBoxImageSpace

        detectionImageSpace.confidence = detection.confidence
        detectionImageSpace.classid = detection.classid
        detectionImageSpace.label = detection.label

        table.insert(detections,detectionImageSpace)

        ::continue::
    end

    return detections
end

---Returns true if box is at the edge of the image
---@param box Rect
---@param boundaries Rect
---@return boolean
function IsBoxAtTheEdge(box, boundaries)

    local edgeDistanceThreshold = 0.01

    local rightEdgeDistance = math.abs(boundaries.x + boundaries.width - (box.x + box.width))
    local leftEdgeDistance = math.abs(box.x - boundaries.x)
    local topEdgeDistance = math.abs(box.y - boundaries.y)
    local bottomEdgeDistance = math.abs(boundaries.y + boundaries.height - (box.y + box.height))

    if rightEdgeDistance < edgeDistanceThreshold or leftEdgeDistance < edgeDistanceThreshold or
            topEdgeDistance < edgeDistanceThreshold or bottomEdgeDistance < edgeDistanceThreshold then
        return true
    end

    return false
end

---Returns the sides of the box that are at the edge of the boundaries
---@param box Rect
---@param boundaries Rect
---@return number[] sidesAtTheEdge - 1: right, 2: left, 3: top, 4: bottom
function GetBoxSidesAtTheEdge(box,boundaries)

    local sidesAtTheEdge = {}

    local edgeDistanceThreshold = 0.01

    local rightEdgeDistance = math.abs(boundaries.x + boundaries.width - (box.x + box.width))
    local leftEdgeDistance = math.abs(box.x - boundaries.x)
    local topEdgeDistance = math.abs(box.y - boundaries.y)
    local bottomEdgeDistance = math.abs(boundaries.y + boundaries.height - (box.y + box.height))

    if rightEdgeDistance < edgeDistanceThreshold then
        table.insert(sidesAtTheEdge,1)
    end

    if leftEdgeDistance < edgeDistanceThreshold then
        table.insert(sidesAtTheEdge,2)
    end

    if topEdgeDistance < edgeDistanceThreshold then
        table.insert(sidesAtTheEdge,3)
    end

    if bottomEdgeDistance < edgeDistanceThreshold then
        table.insert(sidesAtTheEdge,4)
    end

    return sidesAtTheEdge

end

--- Convert Atlas Box to Image Space
---@param atlasBox Rect|Detection
---@param atlasRegion Rect
---@param sourceRegion Rect
---@return Rect
function ConvertAtlasBoxToImageSpace(atlasBox, atlasRegion, sourceRegion)

    local atlasSubRegionXMin = (atlasBox.x - atlasRegion.x) / atlasRegion.width
    local atlasSubRegionYMin = (atlasBox.y - atlasRegion.y) / atlasRegion.height
    local atlasSubRegionXMax = (atlasBox.x + atlasBox.width - atlasRegion.x) / atlasRegion.width
    local atlasSubRegionYMax = (atlasBox.y + atlasBox.height - atlasRegion.y) / atlasRegion.height

    local imageSpaceXMin = atlasSubRegionXMin * sourceRegion.width + sourceRegion.x
    local imageSpaceYMin = atlasSubRegionYMin * sourceRegion.height + sourceRegion.y
    local imageSpaceXMax = atlasSubRegionXMax * sourceRegion.width + sourceRegion.x
    local imageSpaceYMax = atlasSubRegionYMax * sourceRegion.height + sourceRegion.y

    local imageSpaceBox = {
        x = imageSpaceXMin,
        y = imageSpaceYMin,
        width = imageSpaceXMax - imageSpaceXMin,
        height = imageSpaceYMax - imageSpaceYMin,
    }

    return imageSpaceBox
end

--- Filter detections that are at the edge of the sub detection regions but not at the edge of the source image
---@param detections Detection[]
---@param detectionRegions Rect[]
---@param isUsingAtlasPacking boolean
---@param atlasPackingInfo AtlasPackingInfo
---@param labelRestrict table
---@return Detection[]
function FilterEdgeDetections(detections,detectionRegions, isUsingAtlasPacking, atlasPackingInfo,labelRestrict)

    local filteredDetections = {}

    local FullScreenDetectionRegion = {
        x = 0,
        y = 0,
        width = 1,
        height = 1
    }


    for _, detection in ipairs(detections) do

        if labelRestrict ~= nil and not labelRestrict[detection.label] then
            table.insert(filteredDetections,detection)
            goto continue
        end

        local isDetectionAtTheEdge = false
        if isUsingAtlasPacking then
            local sourceRegions = atlasPackingInfo.sourceRegions

            -- An atlas packed detection is only accepted if the number of box sides at the edge of the packing source regions is the same as the number of box sides at the edge of the detection regions
            local boxSidesAtTheEdgeOfTheScreen = GetBoxSidesAtTheEdge(detection,FullScreenDetectionRegion)
            for _, sourceRegion in ipairs(sourceRegions) do
                if BoxContainment(detection,sourceRegion) > 0.8 then

                    local boxSidesAtTheEdgeOfPackingRegion = GetBoxSidesAtTheEdge(detection,sourceRegion)

                    if #boxSidesAtTheEdgeOfTheScreen ~= #boxSidesAtTheEdgeOfPackingRegion then
                        isDetectionAtTheEdge = true
                        break
                    end

                    break
                end
            end
        else
            for _, detectionRegion in ipairs(detectionRegions) do
                local isFullScreenDetectionRegion = detectionRegion.x == 0 and detectionRegion.y == 0 and
                        detectionRegion.width == 1 and detectionRegion.height == 1

                if IsBoxAtTheEdge(detection,detectionRegion) then
                    if not isFullScreenDetectionRegion then
                        isDetectionAtTheEdge = true
                        break
                    else
                        isDetectionAtTheEdge = false
                    end
                end
            end
        end

        if not isDetectionAtTheEdge then
            table.insert(filteredDetections,detection)
        end

        ::continue::
    end

    return filteredDetections

end

--- Filter detections by confidence, allowing each class to have a different confidence threshold
---@param inferencePlugin inferencemanaged
---@param detections Detection[]
---@return Detection[]
function FilterDetectionsByConfidence(inferencePlugin,detections)

    local filteredDetections = {}

    local inferencePluginConfig = inferencePlugin:getConfig()

    local personConfidenceThreshold = inferencePluginConfig["person_confidence_threshold"]
    local animalConfidenceThreshold = inferencePluginConfig["animal_confidence_threshold"]
    local vehicleConfidenceThreshold = inferencePluginConfig["vehicle_confidence_threshold"]

    for _, detection in ipairs(detections) do
        if detection.label == ClassLabels.Person and detection.confidence >= personConfidenceThreshold then
            table.insert(filteredDetections,detection)
        elseif detection.label == ClassLabels.Animal and detection.confidence >= animalConfidenceThreshold then
            table.insert(filteredDetections,detection)
        elseif detection.label == ClassLabels.Vehicle and detection.confidence >= vehicleConfidenceThreshold then
            table.insert(filteredDetections,detection)
        end
    end

    return filteredDetections

end

--- Checks if events generated by a given track should be considered. This is used to discard events of tracks with unknown class when there is a known class track inside the region.
---@param track Track
---@param tracks Track[]
---@return boolean
function ShouldConsiderEventsGeneratedByTrack(track, tracks)


    if IsTrackUnknown(track) then
        local sourceTrackBbox = track.bbox
        for _, track in ipairs(tracks) do
            if IsTrackAnimal(track) or IsTrackPerson(track) or IsTrackVehicle(track) then
                if BoxContainment(track.bbox,sourceTrackBbox) > 0.8 then
                    return false
                end
            end
        end
    end

    return true

end


---Copy Rect
---@param rect Rect
---@return Rect
function CopyRect(rect)
    return {
        x = rect.x,
        y = rect.y,
        width = rect.width,
        height = rect.height
    }
end


function sigmoid(input)
    local output = {}
    for i=1, #input do
        output[i] = 1 / (1 + math.exp(-input[i]))
    end
    return output
end

function get_current_date_RFC_3339()
    local date_table = os.date("!*t") -- Get UTC time
    local year = string.format("%04d", date_table.year)
    local month = string.format("%02d", date_table.month)
    local day = string.format("%02d", date_table.day)
    local hour = string.format("%02d", date_table.hour)
    local minute = string.format("%02d", date_table.min)
    local second = string.format("%02d", date_table.sec)
    return year .. "-" .. month .. "-" .. day .. "T" .. hour .. ":" .. minute .. ":" .. second .. "Z"
end

CurrentDetectorPreset = nil

---Update Movement Preset Settings
---@param instance rt_instance
function UpdateDetectorPreset(instance)

    local storedPreset = instance:getConfigValue("Detector/current_preset")

    if storedPreset ~= CurrentDetectorPreset then

        CurrentDetectorPreset = storedPreset

        api.logging.LogDebug("[Detector] Updating detector preset to "..storedPreset)

        local settingsToUse = instance:getConfigValue("Detector/preset_values/" .. storedPreset)

        api.logging.LogDebug("[Detector] "..inspect(settingsToUse))

        --Apply settings
        for key, value in pairs(settingsToUse) do
            api.logging.LogDebug("[Detector] instance:setConfigValue(\""..key.."\","..inspect(value)..")")
            instance:setConfigValue(key,value)
        end
    end
end

CurrentDetectorSensitivityPreset = nil
---Update Movement Preset Settings
---@param instance rt_instance
function UpdateDetectorSensitivityPreset(instance)

    local storedPreset = instance:getConfigValue("Detector/current_sensitivity_preset")

    if storedPreset ~= CurrentDetectorSensitivityPreset then

        CurrentDetectorSensitivityPreset = storedPreset

        api.logging.LogDebug("[Detector] Updating detector sensitivity preset to "..storedPreset)

        local settingsToUse = instance:getConfigValue("Detector/sensitivity_preset_values/" .. storedPreset)

        api.logging.LogDebug("[Detector] "..inspect(settingsToUse))

        --Apply settings
        for key, value in pairs(settingsToUse) do
            api.logging.LogDebug("[Detector] instance:setConfigValue(\""..key.."\","..value..")")
            instance:setConfigValue(key,value)
        end
    end
end

CurrentMovementSensitivityPreset = nil
---Update Movement Preset Settings
---@param instance rt_instance
function UpdateMovementSensitivityPreset(instance)

    local storedPreset = instance:getConfigValue("Movement/current_sensitivity_preset")

    if storedPreset ~= CurrentMovementSensitivityPreset then

        CurrentMovementSensitivityPreset = storedPreset

        api.logging.LogDebug("[Movement] Updating sensitivity preset to "..storedPreset)

        local settingsToUse = instance:getConfigValue("Movement/sensitivity_preset_values/" .. storedPreset)

        api.logging.LogDebug("[Movement] "..inspect(settingsToUse))

        --Apply settings
        for key, value in pairs(settingsToUse) do
            api.logging.LogDebug("[Movement] instance:setConfigValue(\""..key.."\","..value..")")
            instance:setConfigValue(key,value)
        end
    end
end


---Get Tracks From Plugin
---@param trackerInst trackermanaged
---@param classLabel string @The label of the tracks to get. Can be "unknown", "person", "animal", "vehicle"
---@return Track[]
function GetTracksFromTracker(trackerInst, classLabel)

    local pluginName = trackerInst:getName()
    local trackIds = trackerInst:getTrackIds()

    ---@type Track[]
    local tracks = {}

    for _, trackId in ipairs(trackIds) do
        local bbox = trackerInst:getTrackValue(trackId,"bbox")
        local confidence = trackerInst:getTrackValue(trackId,"confidence")
        local lastSeen = trackerInst:getTrackValue(trackId,"last_seen")
        local movementDirectionTuple = trackerInst:getTrackValue(trackId,"movement_direction")
        local movementDirectionVec = {x = movementDirectionTuple[1], y = movementDirectionTuple[2]}
        local externalId = trackerInst:getTrackValue(trackId,"external_id")
        local trackAge = trackerInst:getTrackValue(trackId,"track_age")
        local id = pluginName .. "_" .. trackId

        local overrideClassLabel = TrackMeta.getValue(id,"override_classification_label")

        -- mark this track as alive
        TrackMeta.setValue(id,"alive",true)

        if overrideClassLabel ~= nil then
            classLabel = overrideClassLabel
        end

        ---@type Track
        local track = {bbox = bbox, classLabel = classLabel, id = id, trackAge = trackAge, lastSeen = lastSeen, originalTrackId = trackId, sourceTrackerTrackId = trackId, movementDirection = movementDirectionVec, externalId = externalId}

        table.insert(tracks,track)
    end

    return tracks
end

--- Filter detections by label
---@param detections Detection[]
---@param label string
---@return Detection[]
function FilterDetectionsByLabel(detections, label)
    local filteredDetections = {}
    for _, detection in ipairs(detections) do
        if detection.label == label then
            table.insert(filteredDetections, detection)
        end
    end
    return filteredDetections
end

--- Get track by id from list
---@param tracks Track[]
---@param id string
---@return Track
function GetTrackById(tracks, id)
    for _, track in ipairs(tracks) do
        if track.id == id then
            return track
        end
    end
    return nil
end

--- Filter overlapping detections (i.e detections that are fully contained within another detection). This can be useful for texture packed inference
---@param detections Detection[]
---@param labelRestrict table
---@return Detection[]
function FilterOverlappingDetections(detections,labelRestrict)

    local filteredDetections = {}

    for _, detection in ipairs(detections) do

        if labelRestrict ~= nil and not labelRestrict[detection.label] then
            table.insert(filteredDetections,detection)
            goto continue
        end

        local isDetectionOverlapping = false
        for _, detection2 in ipairs(detections) do
            if detection ~= detection2 and detection.label == detection2.label then
                if BoxContainment(detection,detection2) > 0.8 then
                    isDetectionOverlapping = true
                    break
                end
            end
        end

        if not isDetectionOverlapping then
            table.insert(filteredDetections,detection)
        end

        ::continue::
    end

    return filteredDetections
end

function FetchAtlasBlockForTracks(atlasPackingInfo, tracks)

    local atlasRegions = atlasPackingInfo.atlasRegions
    local sourceRegions = atlasPackingInfo.sourceRegions

    -- Determine which atlas block does each detection belong to

    local detections = {}
    for idx, detection in ipairs(tracks) do
        local atlasBlockIndex = -1

        local detectionCenterX = detection.bbox.x + detection.bbox.width/2
        local detectionCenterY = detection.bbox.y + detection.bbox.height/2

        for atlasBlockIdx, atlasLocation in ipairs(sourceRegions) do
            -- check if the center of the detection is inside the atlas block

            if detectionCenterX >= atlasLocation.x and detectionCenterX <= atlasLocation.x + atlasLocation.width and
                    detectionCenterY >= atlasLocation.y and detectionCenterY <= atlasLocation.y + atlasLocation.height then
                atlasBlockIndex = atlasBlockIdx
                break
            end
        end

        if atlasBlockIndex == -1 then

            goto continue
        end

        --Convert detection to atlas location space

        if #sourceRegions ~= #atlasRegions then
            goto continue
        end

        local atlasRegion = atlasRegions[atlasBlockIndex]

        image = atlasPackingInfo.atlas:copy(atlasRegion.x, atlasRegion.y, atlasRegion.width, atlasRegion.height, true)

        tracks[idx].atlas_crop = image
        tracks[idx].atlas = atlasPackingInfo.atlas

        ::continue::
    end

    return tracks
end

--- Checks if any zone or tripwire requires vehicles
---@param instance rt_instance
---@param zoneInst zonemanaged
---@param tripwireInst tripwiremanaged
---@return ClassRequirements
function CheckClassesRequiredFromTripwiresAndZones(instance,zoneInst,tripwireInst)

    ---@class ClassRequirements
    local classRequirements = {vehicles = false, people = false, animals = false, unknowns = false}

    local zoneIds = zoneInst:getZoneIds()

    for _, zoneId in ipairs(zoneIds) do
        classRequirements.unknowns = classRequirements.unknowns or (GetZoneConfigValue(instance,zoneId,"detect_unknowns") == true)
        classRequirements.vehicles = classRequirements.vehicles or (GetZoneConfigValue(instance,zoneId,"detect_vehicles") == true)
        classRequirements.people = classRequirements.people or (GetZoneConfigValue(instance,zoneId,"detect_people") == true)
        classRequirements.animals = classRequirements.animals or (GetZoneConfigValue(instance,zoneId,"detect_animals") == true)
    end

    local tripwireIds = tripwireInst:getTripwireIds()

    for _, tripwireId in ipairs(tripwireIds) do
        classRequirements.unknowns = classRequirements.unknowns or (GetTripwireConfigValue(instance,tripwireId,"detect_unknowns") == true)
        classRequirements.vehicles = classRequirements.vehicles or (GetTripwireConfigValue(instance,tripwireId,"detect_vehicles") == true)
        classRequirements.people = classRequirements.people or (GetTripwireConfigValue(instance,tripwireId,"detect_people") == true)
        classRequirements.animals = classRequirements.animals or (GetTripwireConfigValue(instance,tripwireId,"detect_animals") == true)
    end

    return classRequirements
end

--- Filters a list of tentative events by returning the ones that should be published. In addition, this function also returns the tentative events that should be kept for future processing.
---@param tentativeEvents Event[]
---@param instance rt_instance
---@param zoneInst zonemanaged
---@param tripwireInst tripwiremanaged
---@param currentTimeSec number
---@return Event[], Event[]
function ProcessTentativeEvents(tentativeEvents, instance,zoneInst, tripwireInst, currentTimeSec)

    local confirmedEvents = {}
    local updatedTentativeEvents = {}

    for _, event in ipairs(tentativeEvents) do

        -- print("event "..inspect(event))

        -- If any of the event tracks no longer exist, remove this event (i.e. dont add it to the updated list of tentative events)
        local eventTracks = event.tracks

        local eventType = event.type

        if (eventType == "object_left" or eventType == "object_removed") then
            table.insert(confirmedEvents,event)
            goto continue
        end

        for _, track in ipairs(eventTracks) do

--            print("track "..inspect(event))
--            print("IsTrackAlive(track) "..inspect(IsTrackAlive(track)))

            if IsTrackAlive(track) ~= true then
                goto continue
            end
        end

        -- Add a tentative flag to the tracks
        for _, track in ipairs(eventTracks) do
            TrackMeta.setValue(track.id,"has_tentative_event",true)
        end

        local zoneOrTripwireConfig = nil
        local zoneId = event.zone_id
        local tripwireId = event.tripwire_id
        if zoneId ~= nil then
            zoneOrTripwireConfig = GetZoneConfig(instance,zoneId)
        elseif tripwireId ~= nil then
            zoneOrTripwireConfig = GetTripwireConfig(instance,tripwireId)
        else
            api.logging.LogError("GetConfirmedEvents: event has no zone or tripwire id")
            local instance = api.thread.getCurrentInstance()
            instance:stop()
            return {}, {}
        end

        local eventTracks = event.tracks
        local shouldConfirmEvent = true

        for _, track in ipairs(eventTracks) do
--            print("track "..inspect(track))
--            print("zoneOrTripwireConfig "..inspect(zoneOrTripwireConfig))
            if ShouldTriggerEventForTrackOnZoneOrTripwire(track,zoneOrTripwireConfig) ~= true then
                shouldConfirmEvent = false
            end
        end

--        print("shouldConfirmEvent "..inspect(shouldConfirmEvent))

        if shouldConfirmEvent == true then

            -- Since we have confirmed this event, we need to update the track' meta to include this new event
            for _, track in ipairs(eventTracks) do
                local trackEvents = TrackMeta.getValue(track.id,"events")
                trackEvents = trackEvents or {}
                table.insert(trackEvents,event)
                TrackMeta.setValue(track.id,"events",trackEvents)
            end

            table.insert(confirmedEvents,event)
        else
            table.insert(updatedTentativeEvents,event)
        end

        ::continue::
    end

    return confirmedEvents,updatedTentativeEvents
end

---Checks if a track is relevant to a given zone. This only checks for class compatibility
---@param track Track
---@param zoneOrTripwireConfig table
---@return boolean
function IsTrackRelevantForZoneOrTripwire(track,zoneOrTripwireConfig)

    if track == nil or zoneOrTripwireConfig == nil then
        api.logging.LogError("IsTrackRelevantForZoneOrTripwire: track or zoneOrTripwireConfig is nil")
        local instance = api.thread.getCurrentInstance()
        instance:stop()
    end

    if IsTrackPerson(track) and zoneOrTripwireConfig.detect_people == true then
        return true
    elseif IsTrackVehicle(track) and zoneOrTripwireConfig.detect_vehicles == true then
        return true
    elseif IsTrackAnimal(track) == true and zoneOrTripwireConfig.detect_animals == true then
        return true
    elseif IsTrackUnknown(track) == true and zoneOrTripwireConfig.detect_unknowns == true then
        return true
    end

    return false
end

function GetZoneConfigValue(instance,zoneId,configKey)
    return instance:getConfigValue("Zone/Zones/" .. zoneId .. "/" .. configKey)
end

function GetTripwireConfigValue(instance,tripwireId,configKey)
    return instance:getConfigValue("Tripwire/Tripwires/" .. tripwireId .. "/" .. configKey)
end

function GetZoneConfig(instance,zoneId)
    return instance:getConfigValue("Zone/Zones/" .. zoneId)
end

function GetTripwireConfig(instance,tripwireId)
    return instance:getConfigValue("Tripwire/Tripwires/" .. tripwireId)
end

--- Get frame metadata
---@param instance rt_instance
---@param inputimg buffer
function GetFrameMetadata(instance,inputimg)
    return {
        date = os.date("%c"),
        frame_time = inputimg:getTimestamp(),
        frame_id = inputimg:getFrameId(),
        instance_id = instance:getName(),
        system_date = get_current_date_RFC_3339()
    }
end

function IsTrackAlive(track)
    return TrackMeta.getValue(track.id,"alive") == true
end