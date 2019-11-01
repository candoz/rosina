vector = require "vector"
motor_schemas = require "motor_schemas"
motor_conversions = require "motor_conversions"
utils = require "utils"

local CHAIN_BOTS_DISTANCE = 25
local EXPLORING_DISTANCE = 25
local SEARCH, ON_NEST, EXPLORE_CHAIN, CHAIN_LINK, CHAIN_TAIL, ON_PREY = 1, 2, 3, 4, 5, 6 -- All the possible states that a footbot can be in
local LIMITED_RANGE_OF_SENSING, EXTENDED_RANGE_OF_SENSING = 30, 60 -- The two RAB range of sensing considered

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
  robot.wheels.set_velocity(vel_l, vel_r)
end

function check_ground()
  local sensors_on_prey = 0
  local sensors_on_nest = 0
  for _, ground_sensor in ipairs(robot.motor_ground) do
    if ground_sensor.value < 0.1 then 
      sensors_on_prey = sensors_on_prey + 1
    elseif ground_sensor.value > 0.9 then 
      sensors_on_nest = sensors_on_nest + 1 
    end
  end
  if sensors_on_prey >= 1 then return "prey"
  elseif sensors_on_nest >= 3 then return "nest"
  else return "empty_floor" end
end

function search()
  robot.leds.set_all_colors("black")
  local resulting_vector = {length = 0, angle = 0}
  local sensing_a_close_nest_bot = utils.return_rab_neighbour(LIMITED_RANGE_OF_SENSING, function(data) return data[RAB_STATE_INDEX] == ON_NEST end) ~= nil
  local sensing_a_close_link_bot = utils.return_rab_neighbour(LIMITED_RANGE_OF_SENSING, function(data) return data[RAB_STATE_INDEX] == CHAIN_LINK end) ~= nil
  local sensing_a_close_tail_bot = utils.return_rab_neighbour(LIMITED_RANGE_OF_SENSING, function(data) return data[RAB_STATE_INDEX] == CHAIN_TAIL end) ~= nil
  local sensing_a_completed_chain = utils.return_rab_neighbour(EXTENDED_RANGE_OF_SENSING, function(data) return data[RAB_PREY_POSITION_INDEX] > 0 end) ~= nil

  if not sensing_a_completed_chain and (sensing_a_close_nest_bot or sensing_a_close_link_bot or sensing_a_close_tail_bot) then
    current_state = EXPLORE_CHAIN
  elseif not sensing_a_completed_chain and check_ground() == "nest" then
    position_in_chain = 1 -- the nest is at the first position of a chain
    current_state = ON_NEST
  else
    resulting_vector = vector.vec2_polar_sum(motor_schemas.move_straight(), motor_schemas.move_random())
    resulting_vector = vector.vec2_polar_sum(resulting_vector, motor_schemas.avoid_collisions_monosensor())
    resulting_vector = vector.vec2_polar_sum(resulting_vector, motor_schemas.avoid_collisions_multisensor())
  end
  return resulting_vector
end

function on_nest()
  robot.leds.set_all_colors("green")
  emit_chain_info()
  local sensing_a_link_bot = utils.return_rab_neighbour(EXTENDED_RANGE_OF_SENSING, function(data) return data[RAB_STATE_INDEX] == CHAIN_LINK end) ~= nil
  local sensing_a_tail_bot = utils.return_rab_neighbour(EXTENDED_RANGE_OF_SENSING, function(data) return data[RAB_STATE_INDEX] == CHAIN_TAIL end) ~= nil
  if sensing_a_link_bot or sensing_a_tail_bot then
    robot.range_and_bearing.set_data(RAB_NUMBER_OF_CHAINS_INDEX, 1) -- one chain is actually forming from this nest
  else
    robot.range_and_bearing.set_data(RAB_NUMBER_OF_CHAINS_INDEX, 0)
  end
  return {length = 0, angle = 0}
end

function explore_chain()
  robot.leds.set_all_colors("black")
  local resulting_vector = {length = 0, angle = 0}
  local sensing_a_completed_chain = utils.return_rab_neighbour(EXTENDED_RANGE_OF_SENSING, function(data) return data[RAB_PREY_POSITION_INDEX] > 0 end) ~= nil
  local max_rab = utils.return_max_rab_neighbour(RAB_POSITION_INDEX, LIMITED_RANGE_OF_SENSING)
  local sensing_max_rab = max_rab ~= nil

  if not sensing_max_rab or sensing_a_completed_chain then
    current_state = SEARCH -- lost the chain while exploring it
  
  else
    -- NEST bots around me?
    local close_nest_bot = utils.return_rab_neighbour(LIMITED_RANGE_OF_SENSING, function(data) return data[RAB_STATE_INDEX] == ON_NEST end)
    local sensing_a_close_nest_bot = close_nest_bot ~= nil
    local sensing_a_nest_bot = utils.return_rab_neighbour(EXTENDED_RANGE_OF_SENSING, function(data) return data[RAB_STATE_INDEX] == ON_NEST end) ~= nil
    -- LINK bots around me?
    local sensing_a_close_link_bot = utils.return_rab_neighbour(LIMITED_RANGE_OF_SENSING, function(data) return data[RAB_STATE_INDEX] == CHAIN_LINK end) ~= nil
    local sensing_a_link_bot = utils.return_rab_neighbour(EXTENDED_RANGE_OF_SENSING, function(data) return data[RAB_STATE_INDEX] == CHAIN_LINK end) ~= nil
    -- TAIL bots around me?
    local sensing_a_close_tail_bot = utils.return_rab_neighbour(LIMITED_RANGE_OF_SENSING, function(data) return data[RAB_STATE_INDEX] == CHAIN_TAIL end) ~= nil
    local sensing_a_tail_bot = utils.return_rab_neighbour(EXTENDED_RANGE_OF_SENSING, function(data) return data[RAB_STATE_INDEX] == CHAIN_TAIL end) ~= nil

    if (sensing_a_close_nest_bot and close_nest_bot.data[RAB_NUMBER_OF_CHAINS_INDEX] == 0 and not sensing_a_link_bot and not sensing_a_tail_bot) or
        (sensing_a_close_tail_bot and not sensing_a_link_bot and not sensing_a_nest_bot) then
      position_in_chain = max_rab.data[RAB_POSITION_INDEX] + 1 -- I have now the highest position in the chain
      current_state = CHAIN_TAIL

    else -- keep exploring the chain
      local non_max_rab = utils.return_rab_neighbour(LIMITED_RANGE_OF_SENSING, function(data) return data[RAB_POSITION_INDEX] == max_rab.data[RAB_POSITION_INDEX] - 1 end)
      local sensing_non_max_rab = non_max_rab ~= nil

      local avoid_mono = motor_schemas.avoid_collisions_monosensor()
      local move_perpendicular = nil
      if sensing_non_max_rab then
        move_perpendicular = motor_schemas.follow_chain_direction(max_rab, non_max_rab)
      else
        move_perpendicular = motor_schemas.move_perpendicular_to_rab(max_rab)
      end
      local adjust_distance = motor_schemas.adjust_distance_from_footbot(max_rab, EXPLORING_DISTANCE)
      resulting_vector = vector.vec2_polar_sum(avoid_mono, move_perpendicular)
      resulting_vector = vector.vec2_polar_sum(resulting_vector, adjust_distance)
    end
  end
  return resulting_vector
end

function chain_link()
  robot.leds.set_all_colors("white")
  emit_chain_info()
  local resulting_vector = {length = 0, angle = 0}

  if check_ground() == "prey" then
    current_state = ON_PREY
  else

    local prev_bot = utils.return_rab_neighbour(EXTENDED_RANGE_OF_SENSING, function(data) return data[RAB_POSITION_INDEX] == position_in_chain - 1 end)
    local sensing_prev_bot = prev_bot ~= nil
    local next_bot = utils.return_rab_neighbour(EXTENDED_RANGE_OF_SENSING, function(data) return data[RAB_POSITION_INDEX] == position_in_chain + 1 end)
    local sensing_next_bot = next_bot ~= nil

    -- search for a rab neighbour that knows if the chain is completed (if the prey bot position is > 0)
    local rab_neighbour_bot = utils.return_rab_neighbour(EXTENDED_RANGE_OF_SENSING, function(data) return data[RAB_PREY_POSITION_INDEX] > 0 end)
    local sensing_a_completed_chain = rab_neighbour_bot ~= nil
    
    if not sensing_prev_bot or not sensing_next_bot or (sensing_a_completed_chain and position_in_chain > rab_neighbour_bot.data[RAB_PREY_POSITION_INDEX]) then
      position_in_chain = 0  -- I'm not part of the chain anymore
      current_state = SEARCH
    else
      local avoid_mono = motor_schemas.avoid_collisions_monosensor()
      local adjust_distance_prev = motor_schemas.adjust_distance_from_footbot(prev_bot, CHAIN_BOTS_DISTANCE)
      -- local adjust_distance_next = motor_schemas.adjust_distance_from_footbot(next_bot, CHAIN_BOTS_DISTANCE)
      local align = motor_schemas.align(RAB_POSITION_INDEX, position_in_chain, EXTENDED_RANGE_OF_SENSING)
      resulting_vector = vector.vec2_polar_sum(avoid_mono, adjust_distance_prev)
      -- resulting_vector = vector.vec2_polar_sum(resulting_vector, adjust_distance_next)
      resulting_vector = vector.vec2_polar_sum(resulting_vector, align)
    end
  end

  return resulting_vector
end

function chain_tail()
  robot.leds.set_all_colors("yellow")
  emit_chain_info()
  local resulting_vector = {length = 0, angle = 0}

  if check_ground() == "prey" then
    current_state = ON_PREY
  else
    local rab_neighbour_bot = utils.return_rab_neighbour(EXTENDED_RANGE_OF_SENSING, function(data) return data[RAB_PREY_POSITION_INDEX] > 0 end)
    local sensing_a_completed_chain = rab_neighbour_bot ~= nil
    local prev_bot = utils.return_rab_neighbour(EXTENDED_RANGE_OF_SENSING, function(data) return data[RAB_POSITION_INDEX] == position_in_chain - 1 end)
    local sensing_prev_bot = prev_bot ~= nil
    if not sensing_prev_bot or (sensing_a_completed_chain and position_in_chain > rab_neighbour_bot.data[RAB_PREY_POSITION_INDEX]) then
      position_in_chain = 0
      current_state = SEARCH
    else
      local close_tail_bot = utils.return_rab_neighbour(LIMITED_RANGE_OF_SENSING, function(data) return data[RAB_STATE_INDEX] == CHAIN_TAIL end)
      local sensing_close_tail_bot = close_tail_bot ~= nil
      if sensing_close_tail_bot and close_tail_bot.data[RAB_POSITION_INDEX] == position_in_chain + 1 then
        current_state = CHAIN_LINK
      else
        local adjust_distance = motor_schemas.adjust_distance_from_footbot(prev_bot, CHAIN_BOTS_DISTANCE)
        local direction_adjustment = motor_schemas.rotate_chain(prev_bot)
        resulting_vector = vector.vec2_polar_sum(adjust_distance, direction_adjustment)
      end
    end
  end

  return resulting_vector
end

function on_prey()
  robot.leds.set_all_colors("red")
  local sensing_prey_in_lower_position = utils.return_rab_neighbour(EXTENDED_RANGE_OF_SENSING, function(data) return (data[RAB_PREY_POSITION_INDEX] > 0 and data[RAB_PREY_POSITION_INDEX] < position_in_chain) end) ~= nil
  if sensing_prey_in_lower_position then
    position_in_chain = 0
    current_state = SEARCH
  end
  emit_chain_info()
  return { length = 0, angle = 0 }
end

function emit_chain_info()
  robot.range_and_bearing.set_data(RAB_STATE_INDEX, current_state)
  robot.range_and_bearing.set_data(RAB_POSITION_INDEX, position_in_chain)
  if current_state == ON_PREY then
    robot.range_and_bearing.set_data(RAB_PREY_POSITION_INDEX, position_in_chain)
  else
    local rab_neighbour_bot = utils.return_rab_neighbour(EXTENDED_RANGE_OF_SENSING, function(data) return data[RAB_PREY_POSITION_INDEX] > 0 end)
    local sensing_a_completed_chain = rab_neighbour_bot ~= nil
    if sensing_a_completed_chain then
      robot.range_and_bearing.set_data(RAB_PREY_POSITION_INDEX, rab_neighbour_bot.data[RAB_PREY_POSITION_INDEX])
    end
  end
end

function reset()
  local current_state = SEARCH
  local position_in_chain = 0
end

function destroy()
end
