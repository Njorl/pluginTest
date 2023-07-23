VehicleClassifier = {

    Keys = {
        Base = "Global/VehicleClassification",
    },

    VehicleClassesMap = {
        MilitaryMBT = "militaryMBT",
        MilitaryAPC = "militaryAPC",
        MilitaryIFV = "militaryIFV",
        Bicycle = "bicycle",
        Motorbike = "motorbike",
        Car = "car",
        Bus = "bus",
        Truck = "truck",
        Van = "van",
        Background = "background",
        Construction = "construction"
    },

    VehicleClassesList = {
        "militaryMBT",
        "militaryAPC",
        "militaryIFV",
        "bicycle",
        "motorbike",
        "car",
        "bus",
        "truck",
        "van",
        "background",
        "construction"
    }
}


VehicleClassifier.Keys.Enabled = VehicleClassifier.Keys.Base.."/enabled"
VehicleClassifier.Keys.Duration = VehicleClassifier.Keys.Base.."/duration_sec"
VehicleClassifier.Keys.Frequency = VehicleClassifier.Keys.Base.."/frequency_per_sec"
VehicleClassifier.Keys.Model = VehicleClassifier.Keys.Base.."/VehicleClassifier"

VehicleClassifier.VehicleClassesUISelection = {
    VehicleClassifier.VehicleClassesMap.Car,
    VehicleClassifier.VehicleClassesMap.Bus,
    VehicleClassifier.VehicleClassesMap.Truck,
    VehicleClassifier.VehicleClassesMap.Van,
    VehicleClassifier.VehicleClassesMap.Bicycle,
    VehicleClassifier.VehicleClassesMap.Motorbike,
    VehicleClassifier.VehicleClassesMap.Construction,
}

--- Loads the face detector
---@param instance rt_instance
---@return boolean true if loaded
function VehicleClassifier.LoadModel(instance)
    local detectInst = api.factory.inference.get(instance, VehicleClassifier.Keys.Model)
    if detectInst == nil then
        local detectInst = api.factory.inference.create(instance, VehicleClassifier.Keys.Model)
        detectInst:loadModelFromConfig()
    end
    return true
end

--- Checks if the vehicle classifier model has been loaded
---@param instance rt_instance
---@return boolean true if loaded
function VehicleClassifier.IsModelLoaded(instance)
    local detectInst = api.factory.inference.get(instance, VehicleClassifier.Keys.Model)
    return detectInst ~= nil
end

--- Deletes the face detector
---@param instance rt_instance
function VehicleClassifier.DeleteModel(instance)
    deleteInferenceBackend(instance, VehicleClassifier.Keys.Model)
end

--- Checks if face detection is enabled
---@param instance rt_instance
---@return boolean
function VehicleClassifier.IsEnabled(instance)
    return instance:getConfigValue(VehicleClassifier.Keys.Enabled) == true
end

--- Detects faces on image
---@param instance rt_instance
---@param vehicleTracks Track[]
---@param image buffer
---@param currentTimeSec number
function VehicleClassifier.Classify(instance,vehicleTracks,image,currentTimeSec)

    local vehicleBboxes = table.remap(vehicleTracks, function(track)
        return track.bbox
    end)

    if #vehicleBboxes > 0 then
        local classificationDuration = instance:getConfigValue(VehicleClassifier.Keys.Duration)

        local classifierInst = api.factory.inference.get(instance, VehicleClassifier.Keys.Model)
        local vehicleBatchInferenceResults = classifierInst:runInference(vehicleBboxes, image)

        for vehicleIdx, vehicleInferenceResult in ipairs(vehicleBatchInferenceResults) do
            local vehicleTrackId = vehicleTracks[vehicleIdx].id
            local vehicleClassificationAccum = TrackMeta.getValue(vehicleTrackId, "vehicle_classification_accum")
            if vehicleClassificationAccum == nil then
                vehicleClassificationAccum = vehicleInferenceResult.feat
            else
                for classIdx, classConfidence in ipairs(vehicleInferenceResult.feat) do
                    vehicleClassificationAccum[classIdx] = vehicleClassificationAccum[classIdx] + classConfidence
                end
            end

            TrackMeta.setValue(vehicleTrackId, "vehicle_classification_accum", vehicleClassificationAccum)

            local firstClassificationTime = TrackMeta.getValue(vehicleTrackId, "vehicle_classification_first_classification_time")
            if firstClassificationTime == nil then
                firstClassificationTime = currentTimeSec
                TrackMeta.setValue(vehicleTrackId, "vehicle_classification_first_classification_time",currentTimeSec)
            end

            local timeElapsed = currentTimeSec - firstClassificationTime
            if timeElapsed >= classificationDuration then

                local winningClassIdx = table.argmax(vehicleClassificationAccum)
                local winningClassLabel = VehicleClassifier.VehicleClassesList[winningClassIdx]
                TrackMeta.setValue(vehicleTrackId, "vehicle_class", winningClassLabel)
            end


        end
    end
end

--- @param track Track
function VehicleClassifier.GetVehicleClass(track)
    return TrackMeta.getValue(track.id, "vehicle_class")
end

--- @param track Track
function VehicleClassifier.HasVehicleClass(track)
    return TrackMeta.getValue(track.id, "vehicle_class") ~=  nil
end