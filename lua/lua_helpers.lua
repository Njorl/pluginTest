
---@generic T:any, R:any
---@param source T[]
---@param mapper fun(r:T):R
---@return R[]
function table.remap(source, mapper)
    local result = {}
    for _, v in pairs(source) do
        table.insert(result, mapper(v))
    end
    return result
end

---@generic T:any
---@param list      T[]
---@param predicate fun(item:T):boolean
---@return T[]
function table.filter(list, predicate)
    local result = {}

    for _, v in pairs(list) do
        if predicate(v) then
            table.insert(result, v)
        end
    end
    return result
end

---@generic T:any
---@param list      T[]                 @ elements to search through
---@param predicate fun(item:T):boolean @ predicate callback
---@return          integer             @ index of the first element matching predicate or 0 if no matches found
function table.firstIndex(list, predicate)
    for idx, v in ipairs(list) do
        if predicate(v) then
            return idx
        end
    end
    return 0
end

function tableConcat(t1,t2)
    if t2 == nil or #t2 == 0 then
        return t1
    end

    if t1 == nil or #t1 == 0 then
        return t2
    end

    t3 = {}

    for _, value in pairs(t1) do
        table.insert(t3,value)
    end

    for _, value in pairs(t2) do
        table.insert(t3,value)
    end

    return t3
end

function table.extend(listA,listB)
    for _, value in ipairs(listB) do
        table.insert(listA,value)
    end
end

function table.overwrite(map,overwrite)
    local result = {}
    for k, v in pairs(map) do
        result[k] = v
    end
    for k, v in pairs(overwrite) do
        result[k] = v
    end
    return result
end

function table.keys(t)
    local keys={}
    for key,_ in pairs(t) do
      table.insert(keys, key)
    end
    return keys
end

function table.contains(list,item)
    for _, value in ipairs(list) do
        if value == item then
            return true
        end
    end
    return false
end

function table.argmax(list)
    local max = -math.huge
    local maxIdx = 0
    for idx, value in ipairs(list) do
        if value > max then
            max = value
            maxIdx = idx
        end
    end
    return maxIdx
end

function firstToUpper(str)
    return (str:gsub("^%l", string.upper))
end

function table.reduce(list, reducer, start)
    local result = start
    for _, value in ipairs(list) do
        result = reducer(result, value)
    end
    return result
end