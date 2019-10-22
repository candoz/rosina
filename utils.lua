local utils = {}

-- Returns true if value higher than min and lower than max, false otherwise.
function utils.between(value, min, max)
  if value > min and value < max then return true end
  return false
end
  
-- Returns the highest value given a map of sensors, where every sensor has a "value" field.
function utils.get_sensor_with_highest_value(sensors)
  highest = nil
  for _, sensor in pairs(sensors) do
    if highest == nil or highest.value <= sensor.value then
      highest = sensor
    end
  end
  return highest
end

-- Returns true if there is at least one rab with the given value at data[index] under the range_of_sensing
function utils.check_neighbour_value(index, value, range_of_sensing)
  if utils.return_rab_neighbour(index, value, range_of_sensing) ~= nil then
    return true
  else
    return false
  end
end

-- Returns the first rab with the given value at data[index] under the range_of_sensing
function utils.return_rab_neighbour(index, value, range_of_sensing)
  for _, rab in ipairs(robot.range_and_bearing) do
    if rab.range < range_of_sensing and rab.data[index] == value then
      return rab
    end
  end
  return nil
end

-- Returns the first rab with the given value at data[index] under the range_of_sensing
function utils.return_max_rab_neighbour(index, range_of_sensing)
  local max, max_rab = -1, nil
  for _, rab in ipairs(robot.range_and_bearing) do
    if rab.range < range_of_sensing and max < rab.data[index] then
      max = rab.data[index]
      max_rab = rab
    end
  end
  return max_rab
end

return utils
