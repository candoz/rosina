utils = require "utils"

local motor_schemas = {}

local PROXIMITY_THRESHOLD = 0.1
local DISTANCE_TOLERANCE = 0.5  -- to avoid oscillating behaviour in Adjust_distance motor schema

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

function motor_schemas.circumnavigate_towards_the_tail(max_rab, non_max_rab)
  if non_max_rab ~= nil then
    return vector.vec2_polar_sum({ length = max_rab.range / 30, angle = max_rab.horizontal_bearing }, { length = non_max_rab.range / 30, angle = non_max_rab.horizontal_bearing + math.pi })
  else
    if max_rab.horizontal_bearing > 0 then
      return {length = 0.7, angle = max_rab.horizontal_bearing - math.pi / 2}
    else
      return {length = 0.7, angle = max_rab.horizontal_bearing + math.pi / 2}
    end
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
    v.length = v.length / counter
  end
  return v
end

function motor_schemas.adjust_distance_from_footbot(rab, desired_distance)
  if rab.range > desired_distance + DISTANCE_TOLERANCE then
    return { length = math.abs(rab.range - desired_distance) / 3, angle = rab.horizontal_bearing }
  elseif rab.range < desired_distance - DISTANCE_TOLERANCE then
    return { length = math.abs(rab.range - desired_distance) / 3, angle = rab.horizontal_bearing + math.pi }
  else
    return { length = 0, angle = 0 }
  end
end

function motor_schemas.align(index, position_in_chain, range_of_sensing)
  local prev, next = nil, nil
  for _, rab in ipairs(robot.range_and_bearing) do
    if rab.range < range_of_sensing then
      if rab.data[index] == position_in_chain - 1 then
        prev = rab
      elseif rab.data[index] == position_in_chain + 1 then
        next = rab
      end
    end
  end
  return vector.vec2_polar_sum({length = 1.5, angle = prev.horizontal_bearing}, {length = 1.5, angle = next.horizontal_bearing})
end

function motor_schemas.rotate_chain(rab)
  return { length = 0.2, angle = rab.horizontal_bearing + 3 * math.pi / 2 }
end

return motor_schemas
