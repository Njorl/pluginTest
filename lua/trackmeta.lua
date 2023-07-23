TrackMeta = {
    data = {

    }
}

--- Get a value for a track. If the track doesn't exist, nil will be returned.
---@param trackId string
---@param key string
---@return any
function TrackMeta.getValue(trackId, key)
    local trackIdStr = tostring(trackId)
    if TrackMeta.data[trackIdStr] == nil then
        return nil
    end
    return TrackMeta.data[trackIdStr][key]
end

--- Set a value for a track. If the track doesn't exist, it will be created.
---@param trackId string
---@param key string
---@param value any
function TrackMeta.setValue(trackId, key, value)
    local trackIdStr = tostring(trackId)
    -- print("setting value for track " .. trackIdStr .. " key " .. key .. " value " .. tostring(value) )
    if TrackMeta.data[trackIdStr] == nil then
        TrackMeta.data[trackIdStr] = {}
    end
    TrackMeta.data[trackIdStr][key] = value
end

--- Delete a track from the meta data.
---@param trackId string
function TrackMeta.delete(trackId)
    local trackIdStr = tostring(trackId)
    TrackMeta.data[trackIdStr] = nil
end

--- Delete all tracks from the meta data that are not in the given list.
---@param tracks Track[]
function TrackMeta.deleteMissingTracks(tracks)

    local trackIds = table.remap(tracks, function(track)
        return track.id
    end)

    for trackId, _ in pairs(TrackMeta.data) do

        if not table.contains(trackIds, trackId) then
            TrackMeta.delete(trackId)
        end
    end
end

function TrackMeta.print()
    for trackId, trackData in pairs(TrackMeta.data) do
        print("Track " .. trackId)
        for key, value in pairs(trackData) do
            print("  " .. key .. ": " .. inspect(value))
        end
    end
end