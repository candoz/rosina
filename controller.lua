vector = require "vector"
motor_schemas = require "motor_schemas"
motor_conversions = require "motor_conversions"

local MAX_SPEED = 10
local SEARCH, ON_NEST, EXPLORE_CHAIN, CHAIN_LINK, ON_PREY = 1, 2, 3, 4, 5
local current_state = SEARCH
local motor_vector = {length = 0, angle = 0}

function init()
  robot.leds.set_all_colors("black")
end

function step()
  robot.leds.set_all_colors(check_ground())

  local c_tbl =
    {
      [SEARCH] = search,
      [ON_NEST] = on_nest,
      [EXPLORE_CHAIN] = explore_chain,
      [CHAIN_LINK] = chain_link,
      [ON_PREY] = on_pray
    }
  local fun = c_tbl[current_state]
  if(fun) then
    motor_vector = fun()
  end
  
  local vel_l, vel_r = motor_conversions.vec_to_vels(motor_vector, robot.wheels.axis_length)
  -- log("vel_l: " .. vel_l .. " vel_r: " .. vel_r)
  robot.wheels.set_velocity(vel_l, vel_r)
end

function check_ground()
  if robot.motor_ground[1].value > 0.9 or robot.motor_ground[2].value > 0.9 or
      robot.motor_ground[3].value > 0.9 or robot.motor_ground[4].value > 0.9 then
    return "green"

  elseif robot.motor_ground[1].value < 0.1 or robot.motor_ground[2].value < 0.1 or
      robot.motor_ground[3].value < 0.1 or robot.motor_ground[4].value < 0.1 then
    return "red"

  else
    return "black"

  end
end

function search()
  local resulting_vector = {length = 0, angle = 0}
  log(check_ground())
  if check_ground() == "green" then
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

function on_nest()
  return {length = 0, angle = 0}
end

function explore_chain()
  return {length = 0, angle = 0}
end

function chain_link()
  return {length = 0, angle = 0}
end

function on_pray()
  return {length = 0, angle = 0}
end
