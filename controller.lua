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
  log(robot.id .. " " .. position_in_chain .. " " .. current_state)
  if check_ground() == "nest" then
    robot.leds.set_all_colors("green")
  elseif check_ground() == "prey" then
    robot.leds.set_all_colors("red")
    current_state = ON_PREY
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

  elseif robot.motor_ground[1].value < 0.1 and robot.motor_ground[2].value < 0.1 and
      robot.motor_ground[3].value < 0.1 and robot.motor_ground[4].value < 0.1 then
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
    position_in_chain = 1
    current_state = ON_NEST
  else
    local straight = motor_schemas.move_straight()
    local random = motor_schemas.move_random()
    local avoid_mono = motor_schemas.avoid_collisions_monosensor()
    local avoid_multi = motor_schemas.avoid_collisions_multisensor()
    resulting_vector = vector.vec2_polar_sum(straight, random)
    resulting_vector = vector.vec2_polar_sum(resulting_vector, avoid_mono)
    resulting_vector = vector.vec2_polar_sum(resulting_vector, avoid_multi)
  end
  return resulting_vector
end

function on_nest()
  emit_chain_info()
  local tail_second = utils.check_neighbour_value(RAB_STATE_INDEX, CHAIN_TAIL, RAB_SECOND_RANGE_OF_SENSING)
  local link_second = utils.check_neighbour_value(RAB_STATE_INDEX, CHAIN_LINK, RAB_SECOND_RANGE_OF_SENSING)
  if tail_second or link_second then
    robot.range_and_bearing.set_data(3, 1)
  else
    robot.range_and_bearing.set_data(3, 0)
  end
  return {length = 0, angle = 0}
end

function explore_chain()
  local resulting_vector = {length = 0, angle = 0}
  local nest_first = utils.return_rab_neighbour(RAB_STATE_INDEX, ON_NEST, RAB_FIRST_RANGE_OF_SENSING)

  local nest_second = utils.check_neighbour_value(RAB_STATE_INDEX, ON_NEST, RAB_SECOND_RANGE_OF_SENSING)
  local tail_first = utils.check_neighbour_value(RAB_STATE_INDEX, CHAIN_TAIL, RAB_FIRST_RANGE_OF_SENSING)
  local tail_second = utils.check_neighbour_value(RAB_STATE_INDEX, CHAIN_TAIL, RAB_SECOND_RANGE_OF_SENSING)
  local link_first = utils.check_neighbour_value(RAB_STATE_INDEX, CHAIN_LINK, RAB_FIRST_RANGE_OF_SENSING)
  local link_second = utils.check_neighbour_value(RAB_STATE_INDEX, CHAIN_LINK, RAB_SECOND_RANGE_OF_SENSING)
  local max_rab = utils.return_max_rab_neighbour(RAB_POSITION_INDEX, RAB_FIRST_RANGE_OF_SENSING)
  local non_max_rab = utils.return_rab_neighbour(RAB_POSITION_INDEX, max_rab.data[RAB_POSITION_INDEX] - 1, RAB_SECOND_RANGE_OF_SENSING)

  if max_rab == nil then
    current_state = SEARCH
  elseif (nest_first ~= nil and nest_first.data[3] == 0 and tail_second == false and link_second == false) or
          (tail_first == true and nest_second == false and link_second == false) then
    position_in_chain = max_rab.data[2] + 1
    current_state = CHAIN_TAIL
  else
    local avoid_mono = motor_schemas.avoid_collisions_monosensor()
    local move_perpendicular = motor_schemas.circumnavigate_towards_the_tail(max_rab, non_max_rab)

    local adjust_distance = motor_schemas.adjust_distance_from_footbot(max_rab, 25)
    resulting_vector = vector.vec2_polar_sum(avoid_mono, move_perpendicular)
    resulting_vector = vector.vec2_polar_sum(resulting_vector, adjust_distance)
  end
  return resulting_vector
end

function chain_link()
  local resulting_vector = {length = 0, angle = 0}
  emit_chain_info()

  local avoid_mono = motor_schemas.avoid_collisions_monosensor()

  local adjust_distance_prev = motor_schemas.adjust_distance_from_footbot(utils.return_rab_neighbour(RAB_POSITION_INDEX, position_in_chain - 1, RAB_SECOND_RANGE_OF_SENSING), 25)
  local adjust_distance_next = motor_schemas.adjust_distance_from_footbot(utils.return_rab_neighbour(RAB_POSITION_INDEX, position_in_chain + 1, RAB_SECOND_RANGE_OF_SENSING), 25)
  local align = motor_schemas.align(position_in_chain, RAB_SECOND_RANGE_OF_SENSING)
  resulting_vector = vector.vec2_polar_sum(avoid_mono, adjust_distance_prev)
  --resulting_vector = vector.vec2_polar_sum(resulting_vector, adjust_distance_next)
  resulting_vector = vector.vec2_polar_sum(resulting_vector, align)

  return resulting_vector
end

function chain_tail()
  emit_chain_info()
  local tail = utils.return_rab_neighbour(RAB_STATE_INDEX, CHAIN_TAIL, RAB_FIRST_RANGE_OF_SENSING)
  if tail ~= nil and tail.data[2] == position_in_chain + 1 then
    current_state = CHAIN_LINK
    return { length = 0, angle = 0 }
  else
    local prev_rab = utils.return_rab_neighbour(RAB_POSITION_INDEX, position_in_chain - 1, RAB_SECOND_RANGE_OF_SENSING)
    local adjust_distance = motor_schemas.adjust_distance_from_footbot(prev_rab, 25)
    local prey = utils.return_rab_neighbour(RAB_STATE_INDEX, ON_PREY, RAB_SECOND_RANGE_OF_SENSING)
    
    local direction_adjustment = nil
    if prey ~= nil then
      direction_adjustment = motor_schemas.adjust_direction_to_prey(prey)
    else
      direction_adjustment = motor_schemas.rotate_chain(prev_rab)
    end
    return vector.vec2_polar_sum(adjust_distance, direction_adjustment)
  end
end

function on_prey()
  robot.range_and_bearing.set_data(RAB_STATE_INDEX, current_state)
  return { length = 0, angle = 0 }
end

function emit_chain_info()
  robot.range_and_bearing.set_data(RAB_STATE_INDEX, current_state)
  robot.range_and_bearing.set_data(RAB_POSITION_INDEX, position_in_chain)
end
