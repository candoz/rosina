vector = require "vector"
motor_schemas = require "motor_schemas"
motor_conversions = require "motor_conversions"

local MAX_SPEED = 10
local RANGE_OF_SENSING = 25
local SEARCH, ON_NEST, EXPLORE_CHAIN, CHAIN_LINK, CHAIN_TAIL, ON_PREY = 1, 2, 3, 4, 5, 6
local current_state = SEARCH
local motor_vector = {length = 0, angle = 0}

function init()
  robot.leds.set_all_colors("black")
end

function step()
  if check_ground() == "nest" then
    robot.leds.set_all_colors("green")
  elseif check_ground() == "prey" then
    robot.leds.set_all_colors("red")
  else
    robot.leds.set_all_colors("black")
  end

  local c_tbl =
    {
      [SEARCH] = search,
      [ON_NEST] = on_nest,
      [EXPLORE_CHAIN] = explore_chain,
      [CHAIN_LINK] = chain_link,
      [CHAIN_TAIL] = chain_tail,
      [ON_PREY] = on_prey
    }
  local fun = c_tbl[current_state]
  if(fun) then
    motor_vector = fun()
  end
  
  local vel_l, vel_r = motor_conversions.vec_to_vels(motor_vector, robot.wheels.axis_length)
  log("vel_l: " .. vel_l .. " vel_r: " .. vel_r)
  robot.wheels.set_velocity(vel_l, vel_r)
end

function check_ground()
  if robot.motor_ground[1].value > 0.9 or robot.motor_ground[2].value > 0.9 or
      robot.motor_ground[3].value > 0.9 or robot.motor_ground[4].value > 0.9 then
    return "nest"

  elseif robot.motor_ground[1].value < 0.1 or robot.motor_ground[2].value < 0.1 or
      robot.motor_ground[3].value < 0.1 or robot.motor_ground[4].value < 0.1 then
    return "prey"

  else
    return "empty floor"

  end
end

function search()
  local resulting_vector = {length = 0, angle = 0}
  local nest = find_neighbour_state(ON_NEST)
  local tail = find_neighbour_state(CHAIN_TAIL)
  local link = find_neighbour_state(CHAIN_LINK)

  robot.range_and_bearing.set_data(1, SEARCH)
  
  if (nest ~= nil and tail == nil and link == nil) or (tail ~= nil and nest == nil and link == nil) then
    current_state = CHAIN_TAIL
  elseif nest ~= nil or tail ~= nil or link ~= nil then
    current_state = EXPLORE_CHAIN
  elseif check_ground() == "nest" then
    current_state = ON_NEST
  else
    local straight = motor_schemas.move_straight()
    local random = motor_schemas.move_random()
    local avoid_mono = motor_schemas.avoid_collisions_monosensor()
    local avoid_multi = motor_schemas.avoid_collisions_multisensor()
    local move_perpendicular = motor_schemas.move_perpendicular_monosensor()
    resulting_vector = vector.vec2_polar_sum(straight, random)
    resulting_vector = vector.vec2_polar_sum(resulting_vector, avoid_mono)
    resulting_vector = vector.vec2_polar_sum(resulting_vector, avoid_multi)
    resulting_vector = vector.vec2_polar_sum(resulting_vector, move_perpendicular)
  end
  return resulting_vector
end

function find_neighbour_state(state)
  for _, rab in ipairs(robot.range_and_bearing) do
		if rab.range < RANGE_OF_SENSING and rab.data[1] == state then
      return rab
    end
  end
  return nil
end

function on_nest()
  robot.range_and_bearing.set_data(1, ON_NEST)
  -- emit on_nest signal with range and bearing
  return {length = 0, angle = 0}
end

function explore_chain()
  robot.range_and_bearing.set_data(1, EXPLORE_CHAIN)
  return {length = 0, angle = 0}
end

function chain_link()
  robot.range_and_bearing.set_data(1, CHAIN_LINK)
  return {length = 0, angle = 0}
end

function chain_tail()
  robot.range_and_bearing.set_data(1, CHAIN_TAIL)
  return {length = 0, angle = 0}
end

function on_prey()
  robot.range_and_bearing.set_data(1, ON_PREY)
  return {length = 0, angle = 0}
end
