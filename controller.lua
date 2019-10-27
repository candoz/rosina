vector = require "vector"
motor_schemas = require "motor_schemas"
motor_conversions = require "motor_conversions"
utils = require "utils"

local MAX_SPEED = 10
local SEARCH, ON_NEST, EXPLORE_CHAIN, CHAIN_LINK, CHAIN_TAIL, ON_PREY = 1, 2, 3, 4, 5, 6
local RAB_FIRST_RANGE_OF_SENSING, RAB_SECOND_RANGE_OF_SENSING = 30, 60

-- The rab.data INDEXES that represents:
local RAB_STATE_INDEX = 1            -- the footbot's current state;
local RAB_NUMBER_OF_CHAINS_INDEX = 2 -- how many chains are already formed;
local RAB_POSITION_INDEX = 3         -- the footbot's position in chain (nest = 1, then 2, 3, 4 ..);
local RAB_PREY_POSITION_INDEX = 4    -- the position in chain of the footbot ON_PREY (therefore, the length of the completed chain);
                                     --   NB: if rab.data[RAB_PREY_POSITION_INDEX] > 0 it means that the chain is completed.

local current_state = SEARCH
local position_in_chain = 0
local motor_vector = {length = 0, angle = 0}

function init()
  robot.leds.set_all_colors("black")
end

function step()
  log(robot.id .. " " .. position_in_chain .. " " .. current_state)

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
  robot.leds.set_all_colors("black")
  local resulting_vector = {length = 0, angle = 0}
  local nest = utils.check_neighbour_value(RAB_STATE_INDEX, ON_NEST, RAB_FIRST_RANGE_OF_SENSING)
  local tail = utils.check_neighbour_value(RAB_STATE_INDEX, CHAIN_TAIL, RAB_FIRST_RANGE_OF_SENSING)
  local link = utils.check_neighbour_value(RAB_STATE_INDEX, CHAIN_LINK, RAB_FIRST_RANGE_OF_SENSING)
  local ground = check_ground()
  local prey = utils.return_rab_neighboura(RAB_PREY_POSITION_INDEX, RAB_SECOND_RANGE_OF_SENSING)

  if prey == nil and (nest or tail or link) then
    current_state = EXPLORE_CHAIN
  elseif prey == nil and ground == "nest" then
    position_in_chain = 1
    current_state = ON_NEST
  elseif prey == nil and ground == "prey" then
    current_state = ON_PREY
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
  robot.leds.set_all_colors("green")
  emit_chain_info()
  local tail_second = utils.check_neighbour_value(RAB_STATE_INDEX, CHAIN_TAIL, RAB_SECOND_RANGE_OF_SENSING)
  local link_second = utils.check_neighbour_value(RAB_STATE_INDEX, CHAIN_LINK, RAB_SECOND_RANGE_OF_SENSING)
  if tail_second or link_second then
    robot.range_and_bearing.set_data(RAB_NUMBER_OF_CHAINS_INDEX, 1)
  else
    robot.range_and_bearing.set_data(RAB_NUMBER_OF_CHAINS_INDEX, 0)
  end
  return {length = 0, angle = 0}
end

function explore_chain()
  robot.leds.set_all_colors("black")
  local resulting_vector = {length = 0, angle = 0}
  local nest_first = utils.return_rab_neighbour(RAB_STATE_INDEX, ON_NEST, RAB_FIRST_RANGE_OF_SENSING)

  local nest_second = utils.check_neighbour_value(RAB_STATE_INDEX, ON_NEST, RAB_SECOND_RANGE_OF_SENSING)
  local tail_first = utils.check_neighbour_value(RAB_STATE_INDEX, CHAIN_TAIL, RAB_FIRST_RANGE_OF_SENSING)
  local tail_second = utils.check_neighbour_value(RAB_STATE_INDEX, CHAIN_TAIL, RAB_SECOND_RANGE_OF_SENSING)
  local link_first = utils.check_neighbour_value(RAB_STATE_INDEX, CHAIN_LINK, RAB_FIRST_RANGE_OF_SENSING)
  local link_second = utils.check_neighbour_value(RAB_STATE_INDEX, CHAIN_LINK, RAB_SECOND_RANGE_OF_SENSING)
  local max_rab = utils.return_max_rab_neighbour(RAB_POSITION_INDEX, RAB_SECOND_RANGE_OF_SENSING)
  local non_max_rab = utils.return_rab_neighbour(RAB_POSITION_INDEX, max_rab.data[RAB_POSITION_INDEX] - 1, RAB_SECOND_RANGE_OF_SENSING)
  local prey = utils.return_rab_neighboura(RAB_PREY_POSITION_INDEX, RAB_SECOND_RANGE_OF_SENSING)

  if prey ~= nil or max_rab == nil then
    current_state = SEARCH
  elseif (nest_first ~= nil and nest_first.data[RAB_NUMBER_OF_CHAINS_INDEX] == 0 and not tail_second and not link_second) or
          (tail_first and not nest_second and not link_second) then
    position_in_chain = max_rab.data[RAB_POSITION_INDEX] + 1
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
  robot.leds.set_all_colors("white")
  local resulting_vector = {length = 0, angle = 0}
  emit_chain_info()
  local prey = utils.return_rab_neighboura(RAB_PREY_POSITION_INDEX, RAB_SECOND_RANGE_OF_SENSING)

  if prey ~= nil and position_in_chain > prey.data[RAB_PREY_POSITION_INDEX] then
    position_in_chain = 0
    current_state = SEARCH
  else
    local avoid_mono = motor_schemas.avoid_collisions_monosensor()

    local adjust_distance_prev = motor_schemas.adjust_distance_from_footbot(utils.return_rab_neighbour(RAB_POSITION_INDEX, position_in_chain - 1, RAB_SECOND_RANGE_OF_SENSING), 25)
    local adjust_distance_next = motor_schemas.adjust_distance_from_footbot(utils.return_rab_neighbour(RAB_POSITION_INDEX, position_in_chain + 1, RAB_SECOND_RANGE_OF_SENSING), 25)
    local align = motor_schemas.align(RAB_POSITION_INDEX, position_in_chain, RAB_SECOND_RANGE_OF_SENSING)
    resulting_vector = vector.vec2_polar_sum(avoid_mono, adjust_distance_prev)
    --resulting_vector = vector.vec2_polar_sum(resulting_vector, adjust_distance_next)
    resulting_vector = vector.vec2_polar_sum(resulting_vector, align)
  end

  return resulting_vector
end

function chain_tail()
  robot.leds.set_all_colors("yellow")
  emit_chain_info()
  local tail = utils.return_rab_neighbour(RAB_STATE_INDEX, CHAIN_TAIL, RAB_FIRST_RANGE_OF_SENSING)
  local prey = utils.return_rab_neighboura(RAB_PREY_POSITION_INDEX, RAB_SECOND_RANGE_OF_SENSING)

  if prey ~= nil and position_in_chain > prey.data[RAB_PREY_POSITION_INDEX] then
    position_in_chain = 0
    current_state = SEARCH
  elseif tail ~= nil and tail.data[RAB_POSITION_INDEX] == position_in_chain + 1 then
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
  robot.leds.set_all_colors("red")
  robot.range_and_bearing.set_data(RAB_STATE_INDEX, current_state)

  if position_in_chain > 0 then
    robot.range_and_bearing.set_data(RAB_POSITION_INDEX, position_in_chain)
    robot.range_and_bearing.set_data(RAB_PREY_POSITION_INDEX, position_in_chain)
  else 
    local min_rab_neighbour = utils.return_min_rab_neighbour(RAB_POSITION_INDEX, RAB_FIRST_RANGE_OF_SENSING)
    if min_rab_neighbour ~= nil then
      position_in_chain = min_rab_neighbour.data[RAB_POSITION_INDEX] + 1
    end
  end

  return { length = 0, angle = 0 }
end

function emit_chain_info()
  robot.range_and_bearing.set_data(RAB_STATE_INDEX, current_state)
  robot.range_and_bearing.set_data(RAB_POSITION_INDEX, position_in_chain)
  local prey = utils.return_rab_neighboura(RAB_PREY_POSITION_INDEX, RAB_SECOND_RANGE_OF_SENSING)
  if prey ~= nil then
    robot.range_and_bearing.set_data(RAB_PREY_POSITION_INDEX, position_in_chain)
  end
end
