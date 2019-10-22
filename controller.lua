vector = require "vector"
motor_schemas = require "motor_schemas"
motor_conversions = require "motor_conversions"
utils = require "utils"

local MAX_SPEED = 10
local SEARCH, ON_NEST, EXPLORE_CHAIN, CHAIN_LINK, CHAIN_TAIL, ON_PREY = 1, 2, 3, 4, 5, 6
local RAB_FIRST_RANGE_OF_SENSING, RAB_SECOND_RANGE_OF_SENSING = 30, 60
local RAB_STATE_INDEX, RAB_POSITION_INDEX = 1, 2  -- POSITION in chain

local current_state = SEARCH
local position_in_chain = 0
local motor_vector = {length = 0, angle = 0}

function init()
  robot.leds.set_all_colors("black")
end

function step()
  if check_ground() == "nest" then
    robot.leds.set_all_colors("green")
  elseif check_ground() == "prey" then
    robot.leds.set_all_colors("red")
  elseif current_state == CHAIN_TAIL then
    robot.leds.set_all_colors("yellow")
  elseif current_state == CHAIN_LINK then
    robot.leds.set_all_colors("white")
  else
    robot.leds.set_all_colors("black")
  end

  local c_tbl = {
      [SEARCH] = search,
      [ON_NEST] = on_nest,
      [EXPLORE_CHAIN] = explore_chain,
      [CHAIN_LINK] = chain_link,
      [CHAIN_TAIL] = chain_tail,
      [ON_PREY] = on_prey
    }
  local fun = c_tbl[current_state]
  if (fun) then
    motor_vector = fun()
  end
  
  local vel_l, vel_r = motor_conversions.vec_to_vels(motor_vector, robot.wheels.axis_length)
  -- log("l " .. vel_l .. " r " .. vel_r)
  robot.wheels.set_velocity(vel_l, vel_r)
end

function check_ground()
  if robot.motor_ground[1].value > 0.9 and robot.motor_ground[2].value > 0.9 and
      robot.motor_ground[3].value > 0.9 and robot.motor_ground[4].value > 0.9 then
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
  local nest = utils.check_neighbour_value(RAB_STATE_INDEX, ON_NEST, RAB_FIRST_RANGE_OF_SENSING)
  local tail = utils.check_neighbour_value(RAB_STATE_INDEX, CHAIN_TAIL, RAB_FIRST_RANGE_OF_SENSING)
  local link = utils.check_neighbour_value(RAB_STATE_INDEX, CHAIN_LINK, RAB_FIRST_RANGE_OF_SENSING)
  
  if nest == true or tail == true or link == true then
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

function on_nest()
  robot.range_and_bearing.set_data(1, ON_NEST)
  robot.range_and_bearing.set_data(2, position_in_chain)
  return {length = 0, angle = 0}
end

function explore_chain()
  local resulting_vector = {length = 0, angle = 0}
  local nest_first = utils.check_neighbour_value(RAB_STATE_INDEX, ON_NEST, RAB_FIRST_RANGE_OF_SENSING)
  local nest_second = utils.check_neighbour_value(RAB_STATE_INDEX, ON_NEST, RAB_SECOND_RANGE_OF_SENSING)
  local tail_first = utils.check_neighbour_value(RAB_STATE_INDEX, CHAIN_TAIL, RAB_FIRST_RANGE_OF_SENSING)
  local tail_second = utils.check_neighbour_value(RAB_STATE_INDEX, CHAIN_TAIL, RAB_SECOND_RANGE_OF_SENSING)
  local link_first = utils.check_neighbour_value(RAB_STATE_INDEX, CHAIN_LINK, RAB_FIRST_RANGE_OF_SENSING)
  local link_second = utils.check_neighbour_value(RAB_STATE_INDEX, CHAIN_LINK, RAB_SECOND_RANGE_OF_SENSING)
  
  if nest_first == true and tail_second == false and link_second == false then
    current_state = CHAIN_TAIL
    position_in_chain = utils.return_rab_neighbour(RAB_STATE_INDEX, ON_NEST, RAB_FIRST_RANGE_OF_SENSING).data[RAB_POSITION_INDEX] + 1
  elseif tail_first == true and nest_second == false and link_second == false then
    current_state = CHAIN_TAIL
    position_in_chain = utils.return_rab_neighbour(RAB_STATE_INDEX, CHAIN_TAIL, RAB_FIRST_RANGE_OF_SENSING).data[RAB_POSITION_INDEX] + 1
  --elseif check_ground() == "prey" then
  --  current_state = ON_PREY
  else
    -- controllo i due link per andare nella direzione giusta RANGE_OF_SENSING and rab.data[1] = CHAIN_LINK e RANGE_OF_SENSING and rab.data[2] ordinato

    local avoid_mono = motor_schemas.avoid_collisions_monosensor()
    local move_perpendicular = motor_schemas.move_perpendicular_monosensor()
    local adjust_distance = motor_schemas.adjust_distance_from_footbot(utils.return_max_rab_neighbour(RAB_POSITION_INDEX, RAB_FIRST_RANGE_OF_SENSING), 25, RAB_FIRST_RANGE_OF_SENSING)
    resulting_vector = vector.vec2_polar_sum(avoid_mono, move_perpendicular)
    resulting_vector = vector.vec2_polar_sum(resulting_vector, adjust_distance)
  end
  return resulting_vector
end

function chain_link()
  robot.range_and_bearing.set_data(RAB_STATE_INDEX, CHAIN_LINK)
  robot.range_and_bearing.set_data(RAB_POSITION_INDEX, position_in_chain)
  return {length = 0, angle = 0}
end

function chain_tail()
  robot.range_and_bearing.set_data(RAB_STATE_INDEX, CHAIN_TAIL)
  robot.range_and_bearing.set_data(RAB_POSITION_INDEX, position_in_chain)
  local nest = utils.check_neighbour_value(RAB_STATE_INDEX, ON_NEST, RAB_FIRST_RANGE_OF_SENSING)
  local tail = utils.check_neighbour_value(RAB_STATE_INDEX, CHAIN_TAIL, RAB_FIRST_RANGE_OF_SENSING)
  local link = utils.check_neighbour_value(RAB_STATE_INDEX, CHAIN_LINK, RAB_FIRST_RANGE_OF_SENSING)
  if tail == true and (nest == true or link == true) then
    current_state = CHAIN_LINK
  end
  return {length = 0, angle = 0}
end

function on_prey()
  robot.range_and_bearing.set_data(1, ON_PREY)
  return {length = 0, angle = 0}
end
