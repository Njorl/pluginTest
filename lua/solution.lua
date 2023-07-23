-- If you are using VSCode, it's recommended installing Lua (sumneko) extension 
-- and configure the Workspace: Library path by adding the full path of 
-- CVEDIA RT core lua library (eg: D:/CVEDIA-RT/files/assets/lua)

-- If you are using IntelliJ IDEA, it's recommended installing EmmyLua plugin

---@diagnostic disable: lowercase-global 
if project_root == nil then project_root = "" end

dofile(luaroot .. "/api/api.lua")
dofile(luaroot .. "/ui/draw_bbox.lua")
dofile(luaroot .. "/ui/ui_helpers.lua")
dofile(project_root .. "/enums.lua")
dofile(project_root .. "/utils.lua")
dofile(project_root .. "/faces.lua")
dofile(project_root .. "/vehicle_classifier.lua")
dofile(project_root .. "/par_classifier.lua")

---@param solution Solution
function onStartup(solution)
    print("Starting solution")

    solution:registerExportConfigCallback(onExportConfig)
    solution:registerConfigMenu("Solution Settings", "", "", solutionConfig)
    solution:registerUiCallback("onInputSinkConfig", "onInputSinkConfig", onInputSinkConfig)
    solution:registerUiCallback("onTriggerConfig", "onTriggerConfig", onTriggerConfig)
end

function getVersion()
	return "pluginTest v2.0"
end

---@param solution Solution
---@param instance rt_instance
function onStartInstance(solution, instance)
    print("Running instance " .. instance:getName())
end

---@param solution Solution
---@param instance rt_instance
function onStopInstance(solution, instance)
    print("Stopping instance " ..  instance:getName())

    
    deleteInferenceBackend(instance,PluginKeys.DetectorRGB)
    deleteInferenceBackend(instance,PluginKeys.DetectorThermal)
    deleteInferenceBackend(instance,PluginKeys.ClassifierRGB)
    deleteInferenceBackend(instance,PluginKeys.Bbox3d)
    Faces.DeleteDetector(instance)
    VehicleClassifier.DeleteModel(instance)
    ParClassifier.DeleteModel(instance)
end

---@param solution Solution
function onShutdown(solution)
    print("Shutting down solution")

    solution:unregisterExportConfigCallback()
    solution:unregisterConfigMenu("Solution Settings")
    solution:unregisterUiCallback("onInputSinkConfig", "onInputSinkConfig")
    solution:unregisterUiCallback("onTriggerConfig", "onTriggerConfig")
end

function onInputSinkConfig(input)

    local sane_defaults = {inference_strategy=InferenceStrategy.FullFrameInference, check_anchor_point = BoxAnchor.BottomCenter, trigger_on_enter = false, trigger_on_exit = false, detect_people=true, detect_vehicles=true, detect_animals=true, detect_unknowns=false,trigger_crowding=false, trigger_loitering=false,crowding_min_count=4.0,restrict_vehicle_type=false}

    config = {
        { name = "Detector regions", configkey = "DetectorRegions", introtext = "", itemprefix = "Region", type = "rectangle" },
        { name = "Area (Advanced)", configkey = "Zone", introtext = "Advanced area configuration, allowing triggers for enter/exit, movement, crowding, etc", itemprefix = "Area", type = "polygon", groupby = TriggerGroups.AreaAdvanced, default = sane_defaults },
        { name = "Tripwire (Advanced)", configkey = "Tripwire", introtext = "Advanced tripwire configuration, allowing triggers for crossing, counting, and tailgating", itemprefix = "Wire", type = "tripwire", groupby = TriggerGroups.TripwireAdvanced,default = sane_defaults },
        { name = "Tripwire Crossing", configkey = "Tripwire", introtext = "Triggers when an object crosses it", itemprefix = "Wire", type = "tripwire", groupby = TriggerGroups.TripwireCrossing, default = table.overwrite(sane_defaults,{trigger_crossing = true, ignore_stationary_objects=true})  },
        { name = "Tripwire Counting", configkey = "Tripwire", introtext = "Triggers when crossed multiple times", itemprefix = "Wire", type = "tripwire", groupby = TriggerGroups.TripwireCounting, default = table.overwrite(sane_defaults,{trigger_crossing = true, ignore_stationary_objects=true}) },
        { name = "Tripwire Tailgating", configkey = "Tripwire", introtext = "Triggers when two objects cross in quick succession", itemprefix = "Wire", type = "tripwire", groupby = TriggerGroups.TripwireTailgating, default = table.overwrite(sane_defaults,{trigger_tailgating = true, tailgating_maximum_crossing_elapsed=1.0}) },
        { name = "Area Crowding", configkey = "Zone", introtext = "Triggers when multiple objects are inside", itemprefix = "Area", type = "polygon", groupby = TriggerGroups.AreaCrowding, default = table.overwrite(sane_defaults,{trigger_crowding=true, crowding_min_count=4.0}) },
        { name = "Area Loitering", configkey = "Zone", introtext = "Triggers when an object remains inside for a long period", itemprefix = "Area", type = "polygon", groupby = TriggerGroups.AreaLoitering, default = table.overwrite(sane_defaults,{trigger_loitering=true, loitering_min_duration=3.0}) },
        { name = "Area Occupancy", configkey = "Zone", introtext = "Triggers when an object enters/exits", itemprefix = "Area", type = "polygon", groupby = TriggerGroups.AreaOccupancy, default = table.overwrite(sane_defaults,{trigger_on_enter = true, trigger_on_exit = true,ignore_stationary_objects=false}) },
        { name = "Area Intrusion", configkey = "Zone", introtext = "Triggers when a moving object enters", itemprefix = "Intrusion area", type = "polygon", groupby = TriggerGroups.AreaIntrusion, default = table.overwrite(sane_defaults,{trigger_on_enter = false, trigger_on_intrusion= true, trigger_on_exit = false, ignore_stationary_objects=true}) },
        { name = "Area Movement", configkey = "Zone", introtext = "Triggers when an object moves inside", itemprefix = "Area", type = "polygon", groupby = TriggerGroups.AreaMovement, default = table.overwrite(sane_defaults,{trigger_on_enter = true, ignore_stationary_objects=true}) },
        { name = "Area Object Left", configkey = "Zone", introtext = "Triggers when an object is left in the area", itemprefix = "Area", type = "polygon", groupby = TriggerGroups.ObjectLeft,  default = table.overwrite(sane_defaults,{detect_people=false, detect_vehicles=false, detect_animals=false, detect_unknowns=false,trigger_when="object_left", label_format="None", left_duration=4.0}) },
        { name = "Area Object Removed", configkey = "Zone", introtext = "Triggers when an object is removed from the area", itemprefix = "Area", type = "polygon", groupby = TriggerGroups.ObjectRemoved, default = table.overwrite(sane_defaults,{detect_people=false, detect_vehicles=false, detect_animals=false, detect_unknowns=false,trigger_when="object_removed", label_format="None", removed_duration=4.0}) },
    }
    
    return config
end

function onTriggerConfig(ctx)
    
    local instance = api.thread.getCurrentInstance()
    local path = ""

    if ctx.type == 1 then
        path = ctx.container .. "/Zones/" .. ctx.guid
    elseif ctx.type == 2 then
        path = ctx.container .. "/Tripwires/" .. ctx.guid
    else
        path = ctx.container .. "/" .. ctx.guid
    end

    local triggerConfig = instance:getConfigValue(path)
    local triggerWhen = instance:getConfigValue(path .. "/trigger_when")
    local isObjectRemovedLeftZone = (triggerWhen == "object_left" or triggerWhen == "object_removed")

    --To have a collapsing header opened by default use 1<<5 as flag, as per https://docs.rs/imgui/0.0.14/imgui/constant.ImGuiTreeNodeFlags_DefaultOpen.html
    --To have a simple non collapsible header use 1<<8 as flag, as per https://docs.rs/imgui/0.0.14/imgui/constant.ImGuiTreeNodeFlags_Leaf.html

    if isObjectRemovedLeftZone == false then

        local detectVehicles = false
        
        if api.ui.startConfigTable("Class detection", true, 1<<8) then
            detectPeople,_ = api.ui.configCheckbox("People", instance, path .. "/detect_people", true)
            detectVehicles,_ = api.ui.configCheckbox("Vehicles", instance, path .. "/detect_vehicles", true)
            api.ui.configCheckbox("Animals", instance, path .. "/detect_animals", true)
            api.ui.configCheckbox("Unidentified motion", instance, path .. "/detect_unknowns", true)
            api.ui.endConfigTable()
        end

        if api.ui.startConfigTable("Object size", true, 1<<8) then
            local restrictObjectsByMinSize,_ = api.ui.configCheckbox("Restrict objects by minimum size?", instance, path .. "/restrict_object_min_size", false)

            if restrictObjectsByMinSize then
                api.ui.configSliderFloat("Min width (%)", instance, path .. "/object_min_width", 0.0, 1.0, 0.01)
                api.ui.configSliderFloat("Min height (%)", instance, path .. "/object_min_height", 0.0, 1.0, 0.01)
            end

            local restrictObjectsByMaxSize,_ = api.ui.configCheckbox("Restrict objects by maximum size?", instance, path .. "/restrict_object_max_size", false)

            if restrictObjectsByMaxSize then
                api.ui.configSliderFloat("Max width (%)", instance, path .. "/object_max_width", 0.0, 1.0, 0.02)
                api.ui.configSliderFloat("Max height (%)", instance, path .. "/object_max_height", 0.0, 1.0, 0.02)
            end

            api.ui.endConfigTable()
        end

        if detectVehicles then
            if api.ui.startConfigTable("Vehicle subclass", true) then
            
                local restrictByVehicleType, _ =  api.ui.configCheckbox("Restrict by vehicle subclass?", instance, path .. "/restrict_vehicle_type", false)
                if restrictByVehicleType then
                    for _, vehicleName in ipairs(VehicleClassifier.VehicleClassesUISelection) do
                        local vehicleNameHumanReadable = firstToUpper(vehicleName)
                        api.ui.configCheckbox(vehicleNameHumanReadable, instance, path .. "/detect_vehicle_"..vehicleName, true)
                    end
                end
                
                api.ui.endConfigTable()
            end
        end

        if detectPeople then
            if api.ui.startConfigTable("Person attribute restrictions", true) then
            
                local restrictByPersonAttributes, _ =  api.ui.configCheckbox("Restrict by person attributes?", instance, path .. "/restrict_person_attributes", false)
                if restrictByPersonAttributes then
                    for _, classUIInfo in ipairs(ParClassifier.ClassesUISelection) do
                        local label = classUIInfo.Label
                        local field = classUIInfo.Field
                        api.ui.configCheckbox(label, instance, path .. "/restric_person_attribute_"..field, true)
                    end
                end
                
                api.ui.endConfigTable()
            end
        end
        
    

        if api.ui.startConfigTable("Trigger points", true, 1<<8) then
            if BBox3d.isEnabled(instance) then
                api.ui.configComboBox("Anchor point to check for", instance, path .. "/check_anchor_point", BoxAnchorIncluding3DBBoxesOptions, BoxAnchor.BottomCenter)
            else
                api.ui.configComboBox("Anchor point to check for", instance, path .. "/check_anchor_point", BoxAnchorOptions, BoxAnchor.BottomCenter)
            end
            api.ui.endConfigTable()
        end
    end
    
    -- Config options specific to zones
    if ctx.container == "Zone" then

        
        if triggerConfig.groupby == TriggerGroups.AreaOccupancy or triggerConfig.groupby == TriggerGroups.AreaAdvanced then
            if api.ui.startConfigTable("Enter/Exit",true) then
                api.ui.configCheckbox("Trigger event when an object enters?", instance, path .. "/trigger_on_enter", true)
                api.ui.configCheckbox("Trigger event when an object exits?", instance, path .. "/trigger_on_exit", false)
                api.ui.configSliderFloat("Enter activation delay (sec)", instance, path .. "/enter_activation_delay",0.0,2.0,0.2)
                api.ui.endConfigTable()
            end
        end
        

        if triggerConfig.groupby == TriggerGroups.AreaMovement or triggerConfig.groupby == TriggerGroups.AreaAdvanced or triggerConfig.groupby == TriggerGroups.AreaOccupancy then
            
            if api.ui.startConfigTable("Movement",true) then
                api.ui.configCheckbox("Ignore stationary objects?", instance, path .. "/ignore_stationary_objects", false)
                api.ui.endConfigTable()
            end
            
        end
        
        if triggerConfig.groupby == TriggerGroups.AreaCrowding or triggerConfig.groupby == TriggerGroups.AreaAdvanced then
            
            if api.ui.startConfigTable("Crowding",true) then
                local val, changed = api.ui.configCheckbox("Trigger event if crowding?", instance, path .. "/trigger_crowding", false)
                if val then
                    api.ui.configSliderInt("Number of objects", instance, path .. "/crowding_min_count", 2,20,3)
                end
                api.ui.endConfigTable()
            end
            
        end

        if triggerConfig.groupby == TriggerGroups.AreaLoitering or triggerConfig.groupby == TriggerGroups.AreaAdvanced then
            if api.ui.startConfigTable("Loitering",true) then
                local val, changed = api.ui.configCheckbox("Trigger event if loitering?", instance, path .. "/trigger_loitering", false)
                if val then
                    api.ui.configSliderFloat("Minimum stay time (sec)", instance, path .. "/loitering_min_duration", 0.1,20.0,3.0)
                end
                api.ui.endConfigTable()
            end
        end
    end

    -- Config options specific to tripwires
    if ctx.container == "Tripwire" then

        if api.ui.startConfigTable("General tripwire settings",true) then
            
            api.ui.configComboBox("Direction ", instance, path .. "/direction", {{ Both = "Both"}, {Up = "Up"}, {Down = "Down"} }, "Both")
            api.ui.configSliderFloat("Bandwidth", instance, path .. "/bandwidth", 0.0,1.42,0.0)
            api.ui.endConfigTable()
        end

        if triggerConfig.groupby == TriggerGroups.TripwireCrossing or triggerConfig.groupby == TriggerGroups.TripwireCounting or triggerConfig.groupby == TriggerGroups.TripwireAdvanced then
            if api.ui.startConfigTable("Crossing",false) then
                api.ui.configCheckbox("Trigger event when crossed?", instance, path .. "/trigger_crossing", true)
                api.ui.endConfigTable()
            end
        end

        if triggerConfig.groupby == TriggerGroups.TripwireTailgating or triggerConfig.groupby == TriggerGroups.TripwireAdvanced then
            if api.ui.startConfigTable("Tailgating",true) then
                local val, changed = api.ui.configCheckbox("Trigger event if tailgating is detected?", instance, path .. "/trigger_tailgating", false)
                if val then
                    api.ui.configSliderFloat("Maximum elpased time (sec) between crossings", instance, path .. "/tailgating_maximum_crossing_elapsed", 0.1,10.0,1.0)
                end
                api.ui.endConfigTable()
            end
        end
    end

    -- Config options specific to object left/removed zones
    local triggerWhen = instance:getConfigValue(path .. "/trigger_when")
    if ctx.container == "Zone" and (triggerWhen == "object_left" or triggerWhen == "object_removed") then
        if api.ui.startConfigTable("Object Left/Removed",true,1<<5) then
            local val, changed = api.ui.configComboBox("Trigger when ", instance, path .. "/trigger_when", { {object_left = "An Object is left in the area"}, {object_removed = "An object is removed from the area"}  }, "object_left")
            if val == "object_left" then
                api.ui.configSliderFloat("Left duration to trigger (sec)", instance, path .. "/left_duration", 0.1,10.0,4.0)
            else
                api.ui.configSliderFloat("Removed duration to trigger (sec)", instance, path .. "/removed_duration", 0.1,10.0,4.0)
            end
            api.ui.endConfigTable()
        end
    end
end

function solutionConfig(name, key, type)
    local instance = api.thread.getCurrentInstance()

    local modalityChanged = false
    local modality = instance:getConfigValue("Global/modality")
    
    if api.ui.startConfigTable("Sensor Modality",true) then
        modality, modalityChanged = api.ui.configComboBox("Sensor Modality", instance, "Global/modality", { {rgb = "RGB"}, {thermal = "Thermal"}  }, "rgb")
        api.ui.endConfigTable()
    end

    if api.ui.startConfigTable("Object detection",true) then

        detectionEnabled = api.ui.configCheckbox("Enable Object Detection?", instance, "Global/Detection/enabled", true)

        if detectionEnabled then

            local presetChanged = false
            
            if modality == Modality.RGB then
                val, presetChanged = api.ui.configComboBox("Detector preset", instance, "Detector/current_preset", DetectorPresetOptions, DetectorPreset.FullRegionInference)
            end

            if modalityChanged or presetChanged then
                instance:setConfigValue("Global/reload_detector_required", true)
            end

            local val, _ = api.ui.configComboBox("Inference strategy ", instance, "Global/Detection/inference_strategy", InferenceStrategyOptions, InferenceStrategy.FullFrameInference)
    
            if val == InferenceStrategy.MotionGuided then

                -- Show motion guided settings
                api.ui.configSliderFloat("Inference Region padding (%) ", instance,  "Global/Detection/motion_guided_settings/region_padding", 0.0,1.0,0.05)


            end

        end
        
        api.ui.endConfigTable()
    end

    if api.ui.startConfigTable("Object classification",true) then
        
        val, _ = api.ui.configCheckbox("Enable Object Classification?", instance, "Global/Classification/enabled", false)

        if val == true then
            api.ui.configCheckbox("Only moving objects?", instance, "Global/Classification/require_moving_track", false)
            api.ui.configSliderInt("Minimum hits for classification lock", instance,  "Global/Classification/min_hits_for_lock", 1,50,1)

            val, _ = api.ui.configCheckbox("Enable Periodic Classification?", instance, "Global/Classification/periodic_reclassification", false)

            if val == true then
                api.ui.configSliderFloat("Period (sec)", instance,  "Global/Classification/periodic_reclassification_time_sec", 0.1,10.0,1.0)
            end
        end

        api.ui.endConfigTable()
    end

    if api.ui.startConfigTable("3D Bounding boxes",true) then
        
        val, _ = api.ui.configCheckbox("Enable 3D Bounding Boxes?", instance, "Global/Bbox3d/enabled", false)

        if val == true then
            api.ui.configCheckbox("Run on vehicles?", instance, "Global/Bbox3d/run_on_vehicles", false)
        end

        api.ui.endConfigTable()
    end

    if api.ui.startConfigTable("Vehicle subclassification",true) then
        
        val, _ = api.ui.configCheckbox("Enable Vehicle subclassification?", instance, VehicleClassifier.Keys.Enabled, false)
        api.ui.configSliderFloat("Classification time window (sec)", instance,  VehicleClassifier.Keys.Duration, 0.1,5.0,1.0)
        api.ui.configSliderInt("Classification max frequency (times per sec)", instance,  VehicleClassifier.Keys.Frequency, 1,10,4)
        
        api.ui.endConfigTable()
    end

    if api.ui.startConfigTable("Armed person classification",true) then
        
        val, _ = api.ui.configCheckbox("Enable Person attribute classification?", instance, ParClassifier.Keys.Enabled, false)

        if val then
            api.ui.configSliderFloat("Armed person confidence threshold", instance,  ParClassifier.Keys.CarryingGunConfidenceThreshold, 0.0,1.0,0.85)
            api.ui.configSliderFloat("Classification time window duration (sec)", instance,  ParClassifier.Keys.Duration, 0.0,5.0,2.0)
            api.ui.configSliderInt("Classification frequency (times per sec)", instance,  ParClassifier.Keys.Frequency, 1,10,4)
            api.ui.configSliderInt("Bounding box horizontal padding factor (%)", instance,  ParClassifier.Keys.BBoxHorizontalPaddingFactor, 0,300,100)
            api.ui.configSliderInt("Bounding box vertical padding factor (%)", instance,  ParClassifier.Keys.BBoxVerticalPaddingFactor, 0,300,100)
        end
        
        api.ui.endConfigTable()
    end

    if api.ui.startConfigTable("Movement",true) then
        
        api.ui.configSliderFloat("Person Movement Threshold", instance,  "Movement/person_movement_threshold", 0.0,0.5,0.1)
        api.ui.configSliderFloat("Animal Movement Threshold", instance,  "Movement/animal_movement_threshold", 0.0,0.5,0.1)
        api.ui.configSliderFloat("Vehicle Movement Threshold", instance,  "Movement/vehicle_movement_threshold", 0.0,0.5,0.1)
        api.ui.configSliderFloat("Unidentified object Movement Threshold", instance,  "Movement/unknown_movement_threshold", 0.0,0.5,0.1)
        api.ui.endConfigTable()
    end
    

    if api.ui.startConfigTable("Tracking Lock",true) then
        api.ui.configSliderFloat("Time window duration (sec)", instance,  "Tracker/Locking/time_window_duration_sec", 0.0,10.0,1.0)
        api.ui.configSliderFloat("Minimum match ratio (%)", instance,  "Tracker/Locking/match_ratio_threshold", 0.1,1.0,0.5)
        api.ui.endConfigTable()
    end

    if api.ui.startConfigTable("Face detection",true) then

        local faceDetectionEnabled = api.ui.configCheckbox("Enable Face Detection?", instance, FaceDetectionEnabledKey, true)

        if faceDetectionEnabled then

            val, _ = api.ui.configCheckbox("Blur Faces?", instance, FaceDetectionBlurKey, true)
        end
        
        api.ui.endConfigTable()
    end

    if api.ui.startConfigTable("Debug",true) then

        api.ui.configCheckbox("Enable Debug Sinks?", instance, "Global/Debug/enable_debug_sinks", false)
        
        api.ui.endConfigTable()
    end
end

function onExportConfig()
    ---@type SinkDescriptor[]
    local sinks = {
        {
            name = "eventsExport",
            type = SinkDataType.Metadata,
            handlerScripts = {
                {name = "Events metadata - no crops", script = "assets/scripts/events_without_crops.lua"},
                {name = "Events metadata and crops", script = "assets/scripts/passthrough_augmented.lua", encode = true},
                {name = "Events metadata + crops on disk", script = "assets/scripts/events_with_crops_on_disk.lua"},
                {name = "Events tabular format", script = "assets/scripts/events_without_crops_flat.lua"}
            }
        },
        {
            name = "zone_stats",
            type = SinkDataType.Metadata,
            handlerScripts = {
                {name = "Zone Statistics", script = "assets/scripts/passthrough_augmented.lua"}
            }
        },
        {
            name = "output-detections",
            type = SinkDataType.Metadata,
            handlerScripts = {
                {name = "Detections metadata - no crops", script = "assets/scripts/detections_without_buffers.lua"},
                {name = "Detections and crops metadata", script = "assets/scripts/detections_with_crops.lua", encode = true},
                {name = "Detections metadata + crops on disk", script = "assets/scripts/detections_with_crops_on_disk.lua"},
                {name = "Detections tabular format", script = "assets/scripts/detections_without_buffers_flat.lua"},
                {name = "Detections and output image metadata", script = "assets/scripts/detections_with_output_buffer.lua", encode = true}
            }
        },
        {
            name = "output-tracks",
            type = SinkDataType.Metadata,
            handlerScripts = {
                {name = "Tracks metadata - no crops", script = "assets/scripts/passthrough_augmented.lua", encode = true},
                {name = "Tracks metadata and crops", script = "assets/scripts/tracks_with_crops.lua", encode = true},
                {name = "Tracks metadata + crops on disk", script = "assets/scripts/tracks_with_crops_on_disk.lua"},
                {name = "Tracks tabular format", script = "assets/scripts/tracks_flat.lua"}
            }
        },
        {
            name = "output-image",
            type = SinkDataType.Binary,
            handlerScripts = {
                {name = "Output image", script = "assets/scripts/output_buffer.lua"}
            }
        },
        {
            name = "global_object_stats",
            type = SinkDataType.Metadata,
            handlerScripts = {
                {name = "Global Object Statistics", script = "assets/scripts/passthrough_augmented.lua"}
            }
        }
    }

    local instance = api.thread.getCurrentInstance()
    DrawExportConfigOptions(instance, "Output", sinks)
end

