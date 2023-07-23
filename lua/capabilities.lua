---@diagnostic disable: lowercase-global

dofile(project_root .. "/enums.lua")
dofile(project_root .. "/trackmeta.lua")
dofile(project_root .. "/vehicle_classifier.lua")
dofile(project_root .. "/par_classifier.lua")


--- @param track Track
function getTrackDisplayClass(track)
    if IsTrackPerson(track) then

        if ParClassifier.KnowIfIsCarryingGun(track) then
            if ParClassifier.IsCarryingGun(track) then
                return "Armed Person"
            end
        end

        return "Person"
    elseif IsTrackVehicle(track)  then

        if VehicleClassifier.HasVehicleClass(track) then
            return firstToUpper(VehicleClassifier.GetVehicleClass(track))
        end

        return "Vehicle"
    elseif IsTrackAnimal(track)  then
        return "Animal"
    elseif IsTrackUnknown(track)  then
        return "Unknown"
    end
    return "Undefined"
end

--- @param track Track
function getTrackClass(track)
    if IsTrackPerson(track) then
        return "Person"
    elseif IsTrackVehicle(track)  then
        return "Vehicle"
    elseif IsTrackAnimal(track)  then
        return "Animal"
    elseif IsTrackUnknown(track)  then
        return "Unknown"
    end
    return "Undefined"
end


---Checks zones
---@param tracks Track
---@param zoneInst zonemanaged
---@return table
function checkZones(tracks, zoneInst)

    local tracksTargetInfo = {}

    for _, track in pairs(tracks) do

        local trackId = track.id

        local trackTargetInfo = {
            track = track,
            id = trackId,
            trackid=trackId,
            bbox = track.bbox,
            bbox3d = TrackMeta.getValue(trackId,"bbox3d"),
            prev_bbox = TrackMeta.getValue(trackId,"zone_prev_bbox"),
            custom = {
                is_people = TrackMeta.getValue(trackId,"is_people"),
                is_vehicle = TrackMeta.getValue(trackId,"is_vehicle"),
                is_animal = TrackMeta.getValue(trackId,"is_animal"),
                is_unknown = TrackMeta.getValue(trackId,"is_unknown"),
                moving = TrackMeta.getValue(trackId,"moving")
            },
            last_seen = track.last_seen,
        }
        -- getTrackValue returns {} if field is not found. We want it to be null
        -- if next(trackTargetInfo.prev_bbox) == nil then trackTargetInfo.prev_bbox = nil end

        table.insert(tracksTargetInfo,trackTargetInfo)
    end

    -- Check if any zones triggered and update current_hits internally
    -- Iterate over all zones, and check only the tracks relevant to each zone
    local zoneIds = zoneInst:getZoneIds()
    local zonesEvents = {}
    for _, zoneId in pairs(zoneIds) do

        local zoneConfig = instance:getConfigValue("Zone/Zones/"..zoneId)

        local zoneTargets = table.filter(tracksTargetInfo,function (zoneTarget)
            return IsTrackRelevantForZoneOrTripwire(zoneTarget.track,zoneConfig)
        end)

        zoneTargets = table.remap(zoneTargets, function (trackTargetInfo)
            
            local bbox_point = GetAnchorPointFromObject(trackTargetInfo.bbox,trackTargetInfo.bbox3d, zoneConfig.check_anchor_point)
            -- Ensure point is inside the frame
            bbox_point.x = math.max(0,math.min(bbox_point.x,0.99))
            bbox_point.y = math.max(0,math.min(bbox_point.y,0.99))

            return {id=trackTargetInfo.id,points={{bbox_point.x,bbox_point.y}}}
        end)

        zoneInst:processZones(zoneTargets, {zoneId})

        table.extend(zonesEvents,zoneInst:getZoneEvents())
    end

    --Update zone_prev_bbox
    for _, trackTargetInfo in pairs(tracksTargetInfo) do
        TrackMeta.setValue(trackTargetInfo.id,"zone_prev_bbox",CopyRect(trackTargetInfo.bbox))
    end

    return zonesEvents
end

---Checks tripwires. To avoid multiple crossings and other issues, we run this at a fixed (slower) rate than zones
---@param tracks Track[]
---@param tripwireInst tripwiremanaged
---@param currentTimeSec number
---@return table
function checkTripwires(tracks, tripwireInst, currentTimeSec)

    local pointsChecked = {}
    local tracksTargetInfo = {}

    for _, track in pairs(tracks) do

        local trackId = track.id

        local trackTargetInfo = {
            track = track,
            id = trackId,
            trackid=trackId,
            bbox = track.bbox,
            bbox3d = TrackMeta.getValue(trackId,"bbox3d"),
            prev_bbox = TrackMeta.getValue(trackId,"tripwire_prev_bbox"),
            custom = {
                is_people = TrackMeta.getValue(trackId,"is_people"),
                is_vehicle = TrackMeta.getValue(trackId,"is_vehicle"),
                is_animal = TrackMeta.getValue(trackId,"is_animal"),
                is_unknown = TrackMeta.getValue(trackId,"is_unknown"),
                moving = TrackMeta.getValue(trackId,"moving")
            },
            last_seen = track.lastSeen,
        }
        
        if trackTargetInfo.prev_bbox == nil then
            trackTargetInfo.prev_bbox = trackTargetInfo.bbox
        end

        table.insert(tracksTargetInfo,trackTargetInfo)
    end

    -- Check if any tripwires triggered and update current_hits internally
    local tripwiresEvents = {}
    local tripwireIds = tripwireInst:getTripwireIds()

    for _, tripwireId in ipairs(tripwireIds) do
        
        local tripwireConfig = instance:getConfigValue("Tripwire/Tripwires/"..tripwireId)

        local tripwireTargets = table.filter(tracksTargetInfo,function (tripwireTarget)
            -- Only consider relevant tracks that have a previous bbox and have not been checked recently
            return IsTrackRelevantForZoneOrTripwire(tripwireTarget.track,tripwireConfig)
        end)
        
        tripwireTargets = table.remap(tripwireTargets, function (trackTargetInfo)

            local prev_bbox_point = GetAnchorPointFromObject(trackTargetInfo.prev_bbox, trackTargetInfo.bbox3d, tripwireConfig.check_anchor_point)
            local bbox_point = GetAnchorPointFromObject(trackTargetInfo.bbox, trackTargetInfo.bbox3d, tripwireConfig.check_anchor_point)
            trackTargetInfo.points = {{prev_bbox_point.x,prev_bbox_point.y},{bbox_point.x,bbox_point.y}}

            table.insert(pointsChecked, {x=prev_bbox_point.x,y=prev_bbox_point.y})
            table.insert(pointsChecked, {x=bbox_point.x,y=bbox_point.y})

            return {id=trackTargetInfo.id,current_bbox=trackTargetInfo.bbox,points={{prev_bbox_point.x,prev_bbox_point.y},{bbox_point.x,bbox_point.y}}}
        end)

        if #tripwireTargets > 0 then

            tripwireInst:processTripwires(tripwireTargets,{tripwireId})

            table.extend(tripwiresEvents,tripwireInst:getTripwireEvents())

            for _, trackTargetInfo in pairs(tripwireTargets) do
                TrackMeta.setValue(trackTargetInfo.id,"tripwire_prev_bbox",CopyRect(trackTargetInfo.current_bbox))
            end
        end
        
    end

    return {tripwiresEvents, pointsChecked}
end

--- @param zoneInst zonemanaged Instance of Zone plugin
--- @param zoneEvents table Table with zone events
--- @param tracks Track[] List of tracks
--- @param imgbuffer buffer Image where debug text will be rendered to
--- @param currentTimeSec number Current time in seconds (gotten in the beggining of the loop)
--- @param outputEvents table Table with output events
function handleAreas(zoneInst, zoneEvents, tracks, imgbuffer, currentTimeSec, outputEvents)
    local instance = api.thread.getCurrentInstance()

    for _, zoneEvent in ipairs(zoneEvents) do
        local trackId = zoneEvent[1]
        local zoneid = zoneEvent[2]
        local enter = zoneEvent[3]

        local zone = zoneInst:getZoneById(zoneid)
        local zoneConfig = instance:getConfigValue("Zone/Zones/"..zoneid)
        if zone.custom == nil then zone.custom = {} end

        if enter == 0 then
            
            local track = GetTrackById(tracks,trackId)
            local trackBbox = track.bbox
            --Object entered zone            
            --Trigger on enter event
            if ShouldConsiderEventsGeneratedByTrack(track,tracks) == true and (zoneConfig.trigger_on_enter or zoneConfig.trigger_on_intrusion) then
                local label = "Entered area"
                if zoneConfig.trigger_on_enter and zoneConfig.ignore_stationary_objects then
                    label = getTrackDisplayClass(track).." movement in area"
                elseif zoneConfig.trigger_on_intrusion then
                    if IsTrackUnknown(track) then
                        label = "Unknown movement detected"
                    else
                        label = getTrackDisplayClass(track).." Intrusion detected"
                    end

                    -- Dont trigger intrusion for the same track
                    if zone.custom.last_intrusion_track_id == trackId then
                        goto skip_event
                    end

                    zone.custom.last_intrusion_track_id = trackId
                end

                local extra = {
                    bbox = trackBbox,
                    track_id = trackId,
                    external_id = track.externalId,
                    zone = zoneConfig,
                    current_entries = zone.cur_entries,
                    total_hits = zone.total_hits
                }
                extra.class = getTrackClass(track)
                if VehicleClassifier.HasVehicleClass(track) then
                    extra.vehicle_class = VehicleClassifier.GetVehicleClass(track)
                end
                if ParClassifier.KnowIfIsCarryingGun(track) then
                    extra.armed = ParClassifier.IsCarryingGun(track)
                end

                addEvent(outputEvents,EventTypes.AreaEnter,imgbuffer,extra,label,{track},zone,nil)
                ::skip_event::
            end
        
            --Check for crowding
            if zoneConfig.trigger_crowding then
                
                if zone.cur_entries >= zoneConfig.crowding_min_count and zone.custom.is_crowded ~= true then
                    
                    local extra = {
                        zone = zoneConfig,
                        current_entries = zone.cur_entries,
                        total_hits = zone.total_hits
                    }
                    local trackIds = table.keys(zone.custom.objects_inside)
                    local tracks = table.filter(tracks,function (track)
                        return table.contains(trackIds,track.id)
                    end)
                    addEvent(outputEvents,EventTypes.AreaCrowding,imgbuffer,extra,"Crowding in area",tracks,zone,nil)
                    zone.custom.is_crowded = true
                end
            end
        else
            --Object left zone
            -- If zone crowding is active and the number of objects dropped below the threshold, clear the flag so that we can trigger the event again once the threshold is reached
            if zoneConfig.trigger_crowding and zone.custom.is_crowded and zone.cur_entries < zoneConfig.crowding_min_count then
                zone.custom.is_crowded = false
            end

            --Trigger on exit event
            if zoneConfig.trigger_on_exit then
                local extra = {
                    track_id = trackId,
                    zone = zoneConfig,
                    current_entries = zone.cur_entries,
                    total_hits = zone.total_hits
                }
                local eventTracks = {}
                if DoesTrackExist(tracks,trackId) then
                    
                    local track = GetTrackById(tracks,trackId)
                    local trackBbox = track.bbox
                    extra.bbox = trackBbox
                    extra.class = getTrackClass(track)
                    extra.external_id = track.externalId
                    if VehicleClassifier.HasVehicleClass(track) then
                        extra.vehicle_class = VehicleClassifier.GetVehicleClass(track)
                    end
                    if ParClassifier.KnowIfIsCarryingGun(track) then
                        extra.armed = ParClassifier.IsCarryingGun(track)
                    end
                    eventTracks = {GetTrackById(tracks,trackId)}
                end
                addEvent(outputEvents,EventTypes.AreaExit,imgbuffer,extra,"Exited area",eventTracks,zone,nil)
            end

        end
        zoneInst:setZoneValue(zoneid, zone, 0);
    end

end



---Handle Line Crossing and Tailgating
---@param tripwireInst tripwiremanaged
---@param imgbuffer buffer
---@param tripwireEvents any
---@param tracks Track[]
---@param currentTimeSec number
---@param outputEvents table Sequence where line crossing and tailgating events will be added to
function handleLineCrossingAndTailgating(tripwireInst, imgbuffer, tripwireEvents, tracks, currentTimeSec, outputEvents)
    local instance = api.thread.getCurrentInstance()
    local tripWireActivationCooldownSec = 3.0

    if tripwireEvents ~= nil then
        for _, triggerEvent in pairs(tripwireEvents) do

            local trackId = triggerEvent[1]
            local track = GetTrackById(tracks,trackId)

            local trackTripwiresCrossed = TrackMeta.getValue(trackId, "tripwires_crossed")
            if trackTripwiresCrossed == nil then trackTripwiresCrossed = {} end
            
            local trackBbox = track.bbox

            local tripWireId = triggerEvent[2]
            local tripwireConfig = instance:getConfigValue("Tripwire/Tripwires/" .. tripWireId)

            local crossingDirection = triggerEvent[3]

            local tripWire = tripwireInst:getTripwireById(tripWireId)

            if tripWire.custom == nil then tripWire.custom = {} end
            if tripWire.custom.total_hits_with_cooldown == nil then tripWire.custom.total_hits_with_cooldown = 0 end

            if trackBbox ~= nil then

                local shouldTrigger = true

                -- Check if the track has crossed this tripwire before
                if trackTripwiresCrossed[tripWireId] ~= nil then
                    local timeSinceLastCrossing = currentTimeSec - trackTripwiresCrossed[tripWireId]
                    if timeSinceLastCrossing < tripWireActivationCooldownSec then
                        shouldTrigger = false
                    end
                end

                if shouldTrigger then

                    -- Update tripwire crossed time
                    trackTripwiresCrossed[tripWireId] = currentTimeSec
                    TrackMeta.setValue(trackId, "tripwires_crossed", trackTripwiresCrossed)

                    tripWire.custom.total_hits_with_cooldown = tripWire.custom.total_hits_with_cooldown + 1

                    -- Crossed event
                    if tripwireConfig.trigger_crossing then
                        local extra = {
                            count = tripWire.custom.total_hits_with_cooldown,
                            class = getTrackDisplayClass(track),
                            bbox = trackBbox,
                            track_id = trackId,
                            external_id = track.externalId,
                            tripwire = tripwireConfig,
                            total_hits = tripWire.custom.total_hits,
                            crossing_direction = crossingDirection
                        }
                        if VehicleClassifier.HasVehicleClass(track) then
                            extra.vehicle_class = VehicleClassifier.GetVehicleClass(track)
                        end
                        addEvent(outputEvents,EventTypes.TripwireCrossing,imgbuffer,extra,"Tripwire crossed",{track},nil,tripWire)
                    end

                    -- Check tailgating
                    if tripwireConfig.trigger_tailgating then

                        local tailgatingMaxDuration = tripwireConfig.tailgating_maximum_crossing_elapsed or 1.0
                        if tripWire.custom.last_trigger ~= nil then
                            local td = currentTimeSec - tripWire.custom.last_trigger

                            -- Time delta needs to be positive. If the video restarts, that might not be the case
                            if td >= 0.0 and td <= tailgatingMaxDuration then
                                -- Tailgating detected
                                tripWire.custom.last_tailgating = tostring(currentTimeSec)
                                local extra = {
                                    count = tripWire.custom.total_hits_with_cooldown,
                                    class = getTrackDisplayClass(track),
                                    bbox = trackBbox,
                                    track_id = trackId,
                                    external_id = track.externalId,
                                    tripwire = tripwireConfig,
                                    time_interval = td
                                }
                                if VehicleClassifier.HasVehicleClass(track) then
                                    extra.vehicle_class = VehicleClassifier.GetVehicleClass(track)
                                end

                                -- Check if the last track to have crossed the tripwire is still alive. If not, ignore it

                                local lastTrackId = tripWire.custom.last_triggered_track_id

                                local eventTracks = {track}
                                if lastTrackId ~= nil and DoesTrackExist(tracks,lastTrackId) then
                                    eventTracks = {track,GetTrackById(tracks,lastTrackId)}
                                end

                                addEvent(outputEvents,EventTypes.TripwireTailgating,imgbuffer,extra,"Tailgating",eventTracks,nil,tripWire)
                            end
                        end

                        tripWire.custom.last_trigger = currentTimeSec
                        tripWire.custom.last_triggered_track_id = trackId

                    end

                end

            end

            tripWire.total_hits_with_cooldown = tripWire.custom.total_hits_with_cooldown
            tripwireInst:setTripwireValue(tripWireId, tripWire, 0);
        end
    end
    
end

--- @param imgbuffer buffer Pointer to BufferLua object
--- @param zoneInst zonemanaged Instance of Zone plugin
--- @param currentTimeSec number Current time in seconds
--- @param tracks Track[] List of tracks
--- @param outputEvents table Table with output events
function handleAreaLoitering(imgbuffer, zoneInst, currentTimeSec, tracks, outputEvents)
    local instance = api.thread.getCurrentInstance()
    local zoneIds = zoneInst:getZoneIds()

    for _, zoneId in ipairs(zoneIds) do
        local zone = zoneInst:getZoneById(zoneId)
        local zoneConfig = instance:getConfigValue("Zone/Zones/"..zoneId)

        if zoneConfig.trigger_loitering then

            local maxObjectStayDuration = 0.0

            if zoneConfig.loitering_min_duration == nil then
                zoneConfig.loitering_min_duration = 3.0
            end

            if zone.custom ~= nil and zone.custom.objects_inside ~= nil then

                if zone.custom.loitering_notifications == nil then zone.custom.loitering_notifications = {} end

                for trackId, enterTime in pairs(zone.custom.objects_inside) do
                    
                    local trackAreaStayDuration = math.abs(currentTimeSec - tonumber(enterTime))
                
                    if trackAreaStayDuration > maxObjectStayDuration then
                        maxObjectStayDuration = trackAreaStayDuration
                    end

                    if trackAreaStayDuration > zoneConfig.loitering_min_duration and zone.custom.loitering_notifications[trackId] == nil then
                        local track = GetTrackById(tracks, trackId)

                        local extra = {
                            track_id = trackId,
                            external_id = track.externalId,
                            bbox = track.bbox,
                            class = getTrackDisplayClass(track),
                            zone = zoneConfig,
                            loitering_time = trackAreaStayDuration
                        }
                        if VehicleClassifier.HasVehicleClass(track) then
                            extra.vehicle_class = VehicleClassifier.GetVehicleClass(track)
                        end

                        addEvent(outputEvents,EventTypes.AreaLoitering,imgbuffer,extra,"Loitering in area",{track},zone,nil)

                        zone.custom.loitering_notifications[trackId] = tostring(currentTimeSec)

                        zoneInst:setZoneValue(zoneId, zone, 0);
                    end
                end
            end

            -- Clean up dead tracks from loitering notifications
            if zone.custom ~= nil and zone.custom.loitering_notifications ~= nil then
                local shouldSaveZone = false
                for trackId, enterTime in pairs(zone.custom.loitering_notifications) do
                    if not DoesTrackExist(tracks, trackId) then
                        zone.custom.loitering_notifications[trackId] = nil
                        shouldSaveZone = true
                    end
                end
                if shouldSaveZone then
                    zoneInst:setZoneValue(zoneId, zone, 0);
                end
            end
        end
    end
end

--- Updates zones with custom information on which objects are inside and when did they enter
--- In addition, it also smooths the events by applying require enough time to pass before considering the object as "inside"
--- @param zoneEvents table Zone Events
--- @param zoneInst zonemanaged Instance of Zone plugin
--- @param tracks Track[]
--- @param currentTimeSec number Current time in seconds
function updateObjectsInsideZonesAndSmoothEvents(zoneEvents, zoneInst, tracks, currentTimeSec)

    local smoothedZoneEvents = {}


    -- Go through all enter/exit zone events and update zone.custom.objects_inside accordingly 
    for _, zoneEvent in ipairs(zoneEvents) do
        local trackid = zoneEvent[1]
        local zoneId = zoneEvent[2]
        local enter = zoneEvent[3]

        local zone = zoneInst:getZoneById(zoneId)

        if zone.custom == nil then zone.custom = {} end
        if zone.custom.objects_inside == nil then zone.custom.objects_inside = {} end
        if zone.custom.objects_entered == nil then zone.custom.objects_entered = {} end

        -- Check if relevant track entered zone
        if enter == 0 then
            zone.custom.objects_entered[trackid] = currentTimeSec
            zoneInst:setZoneValue(zoneId, zone, 0);
        else
            if zone.custom.objects_inside[trackid] ~= nil then
                table.insert(smoothedZoneEvents,{trackid,zoneId,1})
                zone.custom.objects_inside[trackid] = nil
            else
                zone.custom.objects_entered[trackid] = nil
            end

            zoneInst:setZoneValue(zoneId, zone, 0);
        end
    end

    
    local zoneIds = zoneInst:getZoneIds()

    for _, zoneId in ipairs(zoneIds) do
        local zone = zoneInst:getZoneById(zoneId)

        local zoneConfig = zoneInst:getZoneById(zoneId,1)

        -- check if enough time has elapsed for them to be considered to be inside it
        if zone.custom ~= nil and zone.custom.objects_entered ~= nil then
                
            for trackId, enterTime in pairs(zone.custom.objects_entered) do

                local timeSinceEnter = currentTimeSec - enterTime

                local enterActivationDelay = zoneConfig.enter_activation_delay

                if enterActivationDelay == nil then
                    enterActivationDelay = 0.2
                end

                if timeSinceEnter >= enterActivationDelay then

                    zone.custom.objects_entered[trackId] = nil

                    -- Check if the track still exists (but might killed after entering, but before being considered inside)

                    if DoesTrackExist(tracks,trackId) then
                        zone.custom.objects_inside[trackId] = currentTimeSec
                        table.insert(smoothedZoneEvents,{trackId,zoneId,0})
                        zoneInst:setZoneValue(zoneId, zone, 0);
                    end
                end
            end
        end
        
        -- check if the track still exists.
        -- If not, update zone.custom.objects_inside accordingly
        if zone.custom ~= nil and zone.custom.objects_inside ~= nil then
            for trackId, _ in pairs(zone.custom.objects_inside) do
                if DoesTrackExist(tracks,trackId) == false then
                    -- Track no longer exists, remove entry from objects_inside
                    zone.custom.objects_inside[trackId] = nil
                    zoneInst:setZoneValue(zoneId, zone, 0);

                    -- Add track exited to zoneEvents
                    table.insert(smoothedZoneEvents, {trackId, zoneId, 1})
                end
            end
        end
    end


    return smoothedZoneEvents
end

--- @param trackerInst trackermanaged Tracker plugin instance
--- @param zoneInst zonemanaged Zone plugin instance
--- @param imgbuffer buffer Output image buffer
--- @param currentTimeSec number Current time in seconds
--- @param outputEvents table Output events
function handleObjectRemoved(trackerInst, zoneInst, imgbuffer, currentTimeSec, outputEvents)
    local instance = api.thread.getCurrentInstance()
    local trackIds = trackerInst:getTrackIds()

    local zoneIds = zoneInst:getZoneIds()

    for _, zoneId in pairs(zoneIds) do
        local zoneConfig = instance:getConfigValue("Zone/Zones/"..zoneId)
        local zoneVertices = ConvertVerticesToPairs(zoneConfig.vertices)
        if zoneConfig.trigger_when ~= "object_removed" then
            goto continue
        end

        local triggerDuration = zoneConfig.removed_duration

        for _, trackId in pairs(trackIds) do

            if trackerInst:getTrackValue(trackId,"tracking_lock") ~= true then
                goto continue
            end

            local trackBbox = trackerInst:getTrackValue(trackId,"bbox")
            local current_box = trackBbox
            local isInsideZone = false

            local shapeBox = GetShapeBoundingBox(zoneVertices)

            if BoxContainment(current_box,shapeBox) > 0.5 then
                isInsideZone = true
            end

            local trackCustomData = trackerInst:getTrackValue(trackId,"custom")

            if isInsideZone or trackCustomData.object_last_box_time ~= nil then

                if trackCustomData.object_last_box == nil then
                    trackCustomData.object_last_box = trackBbox
                    trackCustomData.object_last_box_time = currentTimeSec
                end

                local iou_compared_to_last_box = GeomUtils.Iou(trackCustomData.object_last_box,current_box)


                if iou_compared_to_last_box < 0.8 then
                    -- Reset loitering 
                    trackCustomData.object_last_box = trackBbox
                    trackCustomData.object_last_box_time = currentTimeSec
                end

                if trackCustomData.object_removed_track_notifications == nil then
                    trackCustomData.object_removed_track_notifications = {}
                end

                local loitering_duration = currentTimeSec - tonumber(trackCustomData.object_last_box_time)
                
                if loitering_duration > triggerDuration and trackCustomData.object_removed_track_notifications[tostring(trackId)] == nil then
                    -- POST EVENT
                    local extra = {
                        bbox = trackBbox,
                        track_id = trackId,
                        zone = zoneConfig
                    }
                    local track = {bbox=trackBbox}
                    
                    addEvent(outputEvents,EventTypes.ObjectRemoved,imgbuffer,extra,"Object removed",{track},nil,nil)

                    trackCustomData.object_removed_track_notifications[tostring(trackId)] = currentTimeSec
                end

                trackCustomData.object_last_box_time = tostring(trackCustomData.object_last_box_time)

                trackerInst:saveTrackValue(trackId,"custom",trackCustomData)
            end
            ::continue::
        end
        ::continue::
    end
end


--- @param trackerInst trackermanaged Tracker plugin instance
--- @param zoneInst zonemanaged Zone plugin instance
--- @param imgbuffer buffer Output image buffer
--- @param currentTimeSec number Current time in seconds
--- @param outputEvents table Output events
function handleObjectLeft(trackerInst, zoneInst, imgbuffer, currentTimeSec, outputEvents)
    local instance = api.thread.getCurrentInstance()
    local trackIds = trackerInst:getTrackIds()

    local zoneIds = zoneInst:getZoneIds()

    for _, zoneId in pairs(zoneIds) do
        local zoneConfig = instance:getConfigValue("Zone/Zones/"..zoneId)
        local zoneVertices = ConvertVerticesToPairs(zoneConfig.vertices)
        
        if zoneConfig.trigger_when ~= "object_left" then
            goto continue
        end

        local triggerDuration = zoneConfig.left_duration

        for _, trackId in pairs(trackIds) do

            if trackerInst:getTrackValue(trackId,"tracking_lock") ~= true then
                goto continue
            end

            local trackBbox = trackerInst:getTrackValue(trackId,"bbox")
            local current_box = trackBbox
            local isInsideZone = false

            if GeomUtils.Iou(GetShapeBoundingBox(zoneVertices), current_box) > 0.0 then
                isInsideZone = true
            end

            if isInsideZone then

                local trackCustomData = trackerInst:getTrackValue(trackId,"custom")

                if trackCustomData.object_last_box == nil then
                    trackCustomData.object_last_box = trackBbox
                    trackCustomData.object_last_box_time = currentTimeSec
                end


                local iou_compared_to_last_box = GeomUtils.Iou(trackCustomData.object_last_box,current_box)


                if iou_compared_to_last_box < 0.8 then
                    -- Reset timing 
                    trackCustomData.object_last_box = trackBbox
                    trackCustomData.object_last_box_time = currentTimeSec
                end

                local loitering_duration = currentTimeSec - tonumber(trackCustomData.object_last_box_time)

                if trackCustomData.object_left_track_notifications == nil then
                    trackCustomData.object_left_track_notifications = {}
                end

                if loitering_duration > triggerDuration and trackCustomData.object_left_track_notifications[tostring(trackId)] == nil then

                    --print("trackId "..inspect(trackId))
                    --print("trackCustomData.object_left_track_notifications "..inspect(trackCustomData.object_left_track_notifications))

                    -- POST EVENT
                    local extra = {
                        bbox = trackBbox,
                        track_id = trackId,
                        zone = zoneConfig
                    }
                    local track = {bbox=trackBbox}
                    addEvent(outputEvents,EventTypes.ObjectLeft,imgbuffer,extra,"Object left",{track},nil,nil)
                    trackCustomData.object_left_track_notifications[tostring(trackId)] = currentTimeSec
                end            

                trackCustomData.object_last_box_time = tostring(trackCustomData.object_last_box_time)

                trackerInst:saveTrackValue(trackId,"custom",trackCustomData)
            end
            ::continue::
        end
        ::continue::
    end
end


--- @param zoneInst zonemanaged Instance of Zone plugin
function updateZonesUI(zoneInst)

    local zoneIds = zoneInst:getZoneIds()

    for _, zoneId in ipairs(zoneIds) do
        local zone = zoneInst:getZoneById(zoneId)
        if zone.area_type == "intrusion" then
            zone.label_format = "None"
            zone.color = nil
            zoneInst:setZoneValue(zone.id, zone, 0);
        elseif zone.type == "loitering" then
            zone.label_format = "None"
            zoneInst:setZoneValue(zone.id, zone, 0);
        elseif zone.type == "object_left" or zone.type == "object_removed" then
            zone.label_format = "None"
            zoneInst:setZoneValue(zone.id, zone, 0);
        end
    end

end

--- Update the initial zone/tripwire custom fields to work around https://cvedia.atlassian.net/servicedesk/customer/portal/10/IS-610
--- This is a big ugly hack, but it works.
--- @param instance rt_instance Current instance
--- @param zoneInst zonemanaged Instance of Zone plugin
--- @param triggerInst tripwiremanaged Instance of Zone plugin
function updateZonesInitialCustomData(instance,zoneInst,triggerInst)

    local zoneIds = zoneInst:getZoneIds()

    local zonesDataIncludingCustomFields = instance:getConfigValue("Zone")

    for _, zoneId in ipairs(zoneIds) do
        local zone = zoneInst:getZoneById(zoneId)

        local zoneDataIncludingCustomFields = zonesDataIncludingCustomFields[zoneId]

        zone.area_type = zoneDataIncludingCustomFields.area_type
        zone.max_object_size = zoneDataIncludingCustomFields.max_object_size
        zone.type = zoneDataIncludingCustomFields.type
        zone.class_filter = zoneDataIncludingCustomFields.class_filter
        zone.cooldown = zoneDataIncludingCustomFields.cooldown

        zoneInst:setZoneValue(zone.id, zone, 0);

    end


    local tripwireIds = triggerInst:getTripwireIds()

    local tripwiresDataIncludingCustomFields = instance:getConfigValue("Tripwires")

    for _, tripwireId in ipairs(tripwireIds) do
        local tripwire = triggerInst:getTripwireById(tripwireId)

        local tripwireDataIncludingCustomFields = tripwiresDataIncludingCustomFields[tripwireId]

        tripwire.area_type = tripwireDataIncludingCustomFields.area_type
        tripwire.max_object_size = tripwireDataIncludingCustomFields.max_object_size
        tripwire.type = tripwireDataIncludingCustomFields.type
        tripwire.class_filter = tripwireDataIncludingCustomFields.class_filter
        tripwire.tailgating_max_duration = tripwireDataIncludingCustomFields.tailgating_max_duration

        triggerInst:saveTripwire(tripwire.id,tripwire)

    end

end

---Publishes event on the current instance
---@param outputEvents Event[]
---@param eventType string
---@param inputimage buffer
---@param extra table
---@param label string
---@param tracks Track[]
---@param zone table|nil
---@param tripwire table|nil
function addEvent(outputEvents,eventType,inputimage,extra,label,tracks,zone,tripwire)
    -- TODO: expose the padding as a config option
    local padding = 0.02
    local image = nil

    if #tracks > 0 then
        local imageBbox = GetTracksBoundingBox(tracks)
        image = inputimage:copy(imageBbox.x-padding,imageBbox.y-padding,imageBbox.width+padding*2,imageBbox.height+padding*2,true)
    elseif zone ~= nil and extra ~= nil and extra.zone ~= nil and extra.zone.vertices ~= nil then
        
        local zone_bbox = GetShapeBoundingBox(ConvertVerticesToPairs(extra.zone.vertices))
        image = inputimage:copy(zone_bbox.x-padding,zone_bbox.y-padding,zone_bbox.width+padding*2,zone_bbox.height+padding*2,true)
    end

    local instance = api.thread.getCurrentInstance()

    local eventData = {
        id = api.system.generateGuid(),
        type = eventType,
        label = label,
        image = image,
        date = os.date("%c"),
        frame_time = inputimage:getTimestamp(),
        frame_id = inputimage:getFrameId(),
        instance_id = instance:getName(),
        system_date = get_current_date_RFC_3339(),
        extra = extra,
        tracks = tracks,
        zone_id = zone ~= nil and zone.id or nil,
        tripwire_id = tripwire ~= nil and tripwire.id or nil
    }

    table.insert(outputEvents,eventData)
end


---Returns the number of object left/removed zones in an instance
---@param instance any
---@param zoneInst any
function numberOfObjectLeftRemovedAreas(instance,zoneInst)
    local zoneIds = zoneInst:getZoneIds()
    local numberOfAreas = 0
    for _, zoneId in ipairs(zoneIds) do
        local zone = zoneInst:getZoneById(zoneId)
        local zoneConfig = instance:getConfigValue("Zone/Zones/"..zoneId)
        if zoneConfig.trigger_when == "object_left" or zoneConfig.trigger_when == "object_removed" then
            numberOfAreas = numberOfAreas + 1
        end
    end
    return numberOfAreas
end

function anyTriggerRequiresDetector(instance,zoneInst,tripwireInst)
    if #tripwireInst:getTripwireIds() > 0 then
        return true
    end

    local numZones = #zoneInst:getZoneIds()
    if numZones > 0 and numberOfObjectLeftRemovedAreas(instance,zoneInst) < numZones then
        return true
    end

    return false
end
