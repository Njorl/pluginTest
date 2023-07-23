Profiler = {
    timers = {},
    frameTime = 0,
    lastFrameTime = 0,
    enabled = false,

    Enable = function() 
        Profiler.enabled = true
        Profiler.instance:setPerformanceCounterState(true)
    end,

    Disable = function() 
        Profiler.enabled = false
        Profiler.instance:setPerformanceCounterState(false)
    end,

    GetTime = function()
        return api.time.getTime()
    end,

    TraceStart = function(name)
        if (not Profiler.enabled) then return end
        Profiler.timers[name] = Profiler.GetTime()
    end,
    
    TraceStop = function (name)
        if (not Profiler.enabled) then return end
        local t = Profiler.GetTime()
        if (Profiler.timers[name] == nil) then
            Profiler.timers[name] = t
        else
            Profiler.instance:writePerformanceCounter(name, (t - Profiler.timers[name]) / 1000000)
            Profiler.timers[name] = t
        end
    end,

    Reset = function(name)
        if (not Profiler.enabled) then return end
        if (name == nil) then
            Profiler.timers = {}
        else
            Profiler.timers[name] = nil
        end
    end,

    ---@param instance rt_instance
    Init = function(instance) 
        Profiler.instance = instance
    end
}
