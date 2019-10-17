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

return utils
