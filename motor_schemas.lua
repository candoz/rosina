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

function motor_schemas.follow_chain_direction(first_rab, second_rab)
  return vector.vec2_polar_sum({ length = first_rab.range / 30, angle = first_rab.horizontal_bearing }, { length = second_rab.range / 30, angle = second_rab.horizontal_bearing + math.pi })
end

function motor_schemas.move_perpendicular_to_rab(rab)
  if rab.horizontal_bearing > 0 then
    return {length = 0.7, angle = rab.horizontal_bearing - math.pi / 2}
  else
    return {length = 0.7, angle = rab.horizontal_bearing + math.pi / 2}
  end
end

function motor_schemas.avoid_collisions_monosensor()
  local max_proximity_sensor = utils.get_sensor_with_highest_value(robot.proximity)
	if max_proximity_sensor.value > PROXIMITY_THRESHOLD then
    return {length = max_proximity_sensor.value, angle = max_proximity_sensor.angle + math.pi}
  else
    return {length = 0, angle = 0}
	end
end

function motor_schemas.avoid_collisions_multisensor()
  local resulting_vector = { length = 0, angle = 0 }
  local counter = 0
  for i = 1, 24 do
    if robot.proximity[i].value > PROXIMITY_THRESHOLD then
      resulting_vector = vector.vec2_polar_sum(resulting_vector, {length = robot.proximity[i].value, angle = robot.proximity[i].angle + math.pi})
      counter = counter + 1
    end
  end
  if counter ~= 0 then
    resulting_vector.length = resulting_vector.length / counter
  end
  return resulting_vector
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

function motor_schemas.align(index, position_in_chain, chain_length, range_of_sensing)
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
  local constant_vector_component = vector.vec2_polar_sum({length = 3, angle = prev.horizontal_bearing}, {length = 3, angle = next.horizontal_bearing})
  local proportional_vector_module = position_in_chain / chain_length * 2
  local proportional_vector_component = vector.vec2_polar_sum({length = proportional_vector_module, angle = prev.horizontal_bearing}, {length = proportional_vector_module, angle = next.horizontal_bearing})
  return vector.vec2_polar_sum(proportional_vector_component, constant_vector_component)
end

function motor_schemas.rotate_chain(rab)
  return { length = 0.5, angle = rab.horizontal_bearing + 3 * math.pi / 2 }
end

return motor_schemas
