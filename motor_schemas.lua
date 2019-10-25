utils = require "utils"

local motor_schemas = {}

local PROXIMITY_THRESHOLD = 0.1
local DISTANCE_TOLERANCE = 0.5  -- to avoidoscillating behaviour in Adjust_distance motor schema
local RAB_STATE_INDEX, RAB_POSITION_INDEX = 1, 2

function motor_schemas.move_straight()
  return {
    length = 1,
    angle = 0
  }
end

function motor_schemas.move_random()
  return {
    length = robot.random.uniform(),
    angle = robot.random.uniform(-math.pi/2, math.pi/2)
  }
end

function motor_schemas.move_perpendicular_monosensor()  -- TODO da fare, max.value oppure 1?
  local max = utils.get_sensor_with_highest_value(robot.proximity)
  if max.value > PROXIMITY_THRESHOLD then
    if max.angle > 0 then
      return {length = max.value, angle = max.angle - math.pi / 2}
    else
      return {length = max.value, angle = max.angle + math.pi / 2}
    end
  else
    return {length = 0, angle = 0}
	end
end

function motor_schemas.avoid_collisions_monosensor()
  local max = utils.get_sensor_with_highest_value(robot.proximity)
	if max.value > PROXIMITY_THRESHOLD then
    return {length = max.value, angle = max.angle + math.pi}
  else
    return {length = 0, angle = 0}
	end
end

function motor_schemas.avoid_collisions_multisensor()
  local v = { length = 0, angle = 0 }
  local counter = 0
  for i = 1, 24 do
    if robot.proximity[i].value > PROXIMITY_THRESHOLD then
      v = vector.vec2_polar_sum(v, {length = robot.proximity[i].value, angle = robot.proximity[i].angle + math.pi})
      counter = counter + 1
    end
  end
  if counter ~= 0 then
    v.length = v.length / counter   -- MaybeTODO another possible solution: take max value
  end
  return v
end

function motor_schemas.adjust_distance(angle, current_distance, desired_distance)
  

  return {
    --TODO
  }
end

function motor_schemas.adjust_distance_from_footbot(rab, desired_distance)
  if rab.range > desired_distance + DISTANCE_TOLERANCE then
    return { length = math.abs(rab.range - desired_distance) / 4, angle = rab.horizontal_bearing }
  elseif rab.range < desired_distance - DISTANCE_TOLERANCE then
    return { length = math.abs(rab.range - desired_distance) / 4, angle = rab.horizontal_bearing + math.pi }
  else
    return { length = 0, angle = 0 }
  end
end

function motor_schemas.align(position_in_chain, range_of_sensing)
  local prev, next = nil, nil
  for _, rab in ipairs(robot.range_and_bearing) do
    if rab.range < range_of_sensing then
      if rab.data[2] == position_in_chain - 1 then
        prev = rab
      elseif rab.data[2] == position_in_chain + 1 then
        next = rab
      end
    end
  end
  return vector.vec2_polar_sum({length = 0.3, angle = prev.horizontal_bearing}, {length = 0.3, angle = next.horizontal_bearing})
end

function motor_schemas.adjust_direction_to_prey(rab)
  return { length = 0.5, angle = rab.horizontal_bearing }
end

return motor_schemas
