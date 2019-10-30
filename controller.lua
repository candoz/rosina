vector = require "vector"
motor_schemas = require "motor_schemas"
motor_conversions = require "motor_conversions"
utils = require "utils"

local MAX_SPEED = 10
local CHAIN_BOTS_DISTANCE = 25
local EXPLORING_DISTANCE = 25
local SEARCH, ON_NEST, EXPLORE_CHAIN, CHAIN_LINK, CHAIN_TAIL, ON_PREY = 1, 2, 3, 4, 5, 6 -- All the possible states that a footbot can be in
local RANGE_OF_SENSING_1, RANGE_OF_SENSING_2 = 30, 60 -- The two RAB range of sensing considered

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
  elseif sensors_on_nest >= 4 then return "nest"
  else return "empty_floor" end
end

function search()
  robot.leds.set_all_colors("black")
  local resulting_vector = {length = 0, angle = 0}
  local sensing_a_close_nest_bot = utils.return_rab_neighbour(RANGE_OF_SENSING_1, function(data) return data[RAB_STATE_INDEX] == ON_NEST end) ~= nil
  local sensing_a_close_link_bot = utils.return_rab_neighbour(RANGE_OF_SENSING_1, function(data) return data[RAB_STATE_INDEX] == CHAIN_LINK end) ~= nil
  local sensing_a_close_tail_bot = utils.return_rab_neighbour(RANGE_OF_SENSING_1, function(data) return data[RAB_STATE_INDEX] == CHAIN_TAIL end) ~= nil
  local sensing_a_completed_chain = utils.return_rab_neighbour(RANGE_OF_SENSING_2, function(data) return data[RAB_PREY_POSITION_INDEX] > 0 end) ~= nil

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
  local sensing_a_link_bot = utils.return_rab_neighbour(RANGE_OF_SENSING_2, function(data) return data[RAB_STATE_INDEX] == CHAIN_LINK end) ~= nil
  local sensing_a_tail_bot = utils.return_rab_neighbour(RANGE_OF_SENSING_2, function(data) return data[RAB_STATE_INDEX] == CHAIN_TAIL end) ~= nil
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
  local sensing_a_completed_chain = utils.return_rab_neighbour(RANGE_OF_SENSING_2, function(data) return data[RAB_PREY_POSITION_INDEX] > 0 end) ~= nil
  local max_rab = utils.return_max_rab_neighbour(RAB_POSITION_INDEX, RANGE_OF_SENSING_1)

  if max_rab == nil or sensing_a_completed_chain then
    current_state = SEARCH -- lost the chain while exploring it
  
  else
    -- NEST bots around me?
    local close_nest_bot = utils.return_rab_neighbour(RANGE_OF_SENSING_1, function(data) return data[RAB_STATE_INDEX] == ON_NEST end)
    local sensing_a_close_nest_bot = close_nest_bot ~= nil
    local sensing_a_nest_bot = utils.return_rab_neighbour(RANGE_OF_SENSING_2, function(data) return data[RAB_STATE_INDEX] == ON_NEST end) ~= nil
    -- LINK bots around me?
    local sensing_a_close_link_bot = utils.return_rab_neighbour(RANGE_OF_SENSING_1, function(data) return data[RAB_STATE_INDEX] == CHAIN_LINK end) ~= nil
    local sensing_a_link_bot = utils.return_rab_neighbour(RANGE_OF_SENSING_2, function(data) return data[RAB_STATE_INDEX] == CHAIN_LINK end) ~= nil
    -- TAIL bots around me?
    local sensing_a_close_tail_bot = utils.return_rab_neighbour(RANGE_OF_SENSING_1, function(data) return data[RAB_STATE_INDEX] == CHAIN_TAIL end) ~= nil
    local sensing_a_tail_bot = utils.return_rab_neighbour(RANGE_OF_SENSING_2, function(data) return data[RAB_STATE_INDEX] == CHAIN_TAIL end) ~= nil

    if (sensing_a_close_nest_bot and close_nest_bot.data[RAB_NUMBER_OF_CHAINS_INDEX] == 0 and not sensing_a_link_bot and not sensing_a_tail_bot) or
        (sensing_a_close_tail_bot and not sensing_a_link_bot and not sensing_a_nest_bot) then
      position_in_chain = max_rab.data[RAB_POSITION_INDEX] + 1 -- I have now the highest position in the chain
      current_state = CHAIN_TAIL

    else -- keep exploring the chain
      local non_max_rab = utils.return_rab_neighbour(RANGE_OF_SENSING_1, function(data) return data[RAB_POSITION_INDEX] == max_rab.data[RAB_POSITION_INDEX] - 1 end)

      local avoid_mono = motor_schemas.avoid_collisions_monosensor()
      local move_perpendicular = motor_schemas.circumnavigate_towards_the_tail(max_rab, non_max_rab)
      local adjust_distance = motor_schemas.adjust_distance_from_footbot(max_rab, EXPLORING_DISTANCE)
      resulting_vector = vector.vec2_polar_sum(avoid_mono, move_perpendicular)
      resulting_vector = vector.vec2_polar_sum(resulting_vector, adjust_distance)
    end
  end
  return resulting_vector
end

-- function chain_link()
--   robot.leds.set_all_colors("white")
--   emit_chain_info()
--   local resulting_vector = {length = 0, angle = 0}

--   local completed_chain_length = -1
--   local completed_chain_info_bot = utils.return_rab_neighboura(RAB_PREY_POSITION_INDEX, RANGE_OF_SENSING_2)
--   if completed_chain_info_bot ~= nil then
--     commpleted_chain_length = completed_chain_info_bot.data[RAB_PREY_POSITION_INDEX]
--   end

--   if position_in_chain >= completed_chain_length then
--     position_in_chain = 0
--     current_state = SEARCH -- I'm no longer needed as a link
  
--   else -- still a chain link

--     -- TODO: if floor == "prey" allora diventa tu il prey_bot

--     local prev_bot = utils.return_rab_neighbour(RAB_POSITION_INDEX, position_in_chain - 1, RANGE_OF_SENSING_2)
--     local next_bot = utils.return_rab_neighbour(RAB_POSITION_INDEX, position_in_chain + 1, RANGE_OF_SENSING_2)

--     local avoid_mono = motor_schemas.avoid_collisions_monosensor()
--     local adjust_distance_prev = motor_schemas.adjust_distance_from_footbot(prev_bot, CHAIN_BOTS_DISTANCE)
--     local adjust_distance_next = motor_schemas.adjust_distance_from_footbot(next_bot, CHAIN_BOTS_DISTANCE)
--     local align = motor_schemas.align(RAB_POSITION_INDEX, position_in_chain, RANGE_OF_SENSING_2)
--     resulting_vector = vector.vec2_polar_sum(avoid_mono, adjust_distance_prev)
--     resulting_vector = vector.vec2_polar_sum(resulting_vector, adjust_distance_next)
--     resulting_vector = vector.vec2_polar_sum(resulting_vector, align)
--   end

--   return resulting_vector
-- end

function chain_link()
  robot.leds.set_all_colors("white")
  emit_chain_info()
  local resulting_vector = {length = 0, angle = 0}
  local prey = utils.return_rab_neighbour(RANGE_OF_SENSING_2, function(data) return data[RAB_PREY_POSITION_INDEX] > 0 end)
  local prev_rab = utils.return_rab_neighbour(RANGE_OF_SENSING_2, function(data) return data[RAB_POSITION_INDEX] == position_in_chain - 1 end)
  local next_rab = utils.return_rab_neighbour(RANGE_OF_SENSING_2, function(data) return data[RAB_POSITION_INDEX] == position_in_chain + 1 end)

  if check_ground() == "prey" then
    current_state = ON_PREY
  elseif prev_rab == nil or next_rab == nil or (prey ~= nil and position_in_chain > prey.data[RAB_PREY_POSITION_INDEX]) then
    position_in_chain = 0
    current_state = SEARCH
  else
    local avoid_mono = motor_schemas.avoid_collisions_monosensor()

    local adjust_distance_prev = motor_schemas.adjust_distance_from_footbot(prev_rab, CHAIN_BOTS_DISTANCE)
    local adjust_distance_next = motor_schemas.adjust_distance_from_footbot(next_rab, CHAIN_BOTS_DISTANCE)
    local align = motor_schemas.align(RAB_POSITION_INDEX, position_in_chain, RANGE_OF_SENSING_2)
    resulting_vector = vector.vec2_polar_sum(avoid_mono, adjust_distance_prev)
    --resulting_vector = vector.vec2_polar_sum(resulting_vector, adjust_distance_next)
    resulting_vector = vector.vec2_polar_sum(resulting_vector, align)
  end

  return resulting_vector
end

function chain_tail()
  robot.leds.set_all_colors("yellow")
  emit_chain_info()
  local resulting_vector = {length = 0, angle = 0}
  local tail = utils.return_rab_neighbour(RANGE_OF_SENSING_1, function(data) return data[RAB_STATE_INDEX] == CHAIN_TAIL end)
  local prey_position = utils.return_rab_neighbour(RANGE_OF_SENSING_2, function(data) return data[RAB_PREY_POSITION_INDEX] > 0 end)
  local prev_rab = utils.return_rab_neighbour(RANGE_OF_SENSING_2, function(data) return data[RAB_POSITION_INDEX] == position_in_chain - 1 end)

  if check_ground() == "prey" then
    current_state = ON_PREY
  elseif prev_rab == nil or (prey_position ~= nil and position_in_chain > prey_position.data[RAB_PREY_POSITION_INDEX]) then
    position_in_chain = 0
    current_state = SEARCH
  elseif tail ~= nil and tail.data[RAB_POSITION_INDEX] == position_in_chain + 1 then
    current_state = CHAIN_LINK
  else
    local adjust_distance = motor_schemas.adjust_distance_from_footbot(prev_rab, CHAIN_BOTS_DISTANCE)
    local prey = utils.return_rab_neighbour(RANGE_OF_SENSING_2, function(data) return data[RAB_STATE_INDEX] == ON_PREY end)
    
    local direction_adjustment = nil
    if prey ~= nil then
      direction_adjustment = motor_schemas.adjust_direction_to_prey(prey)
    else
      direction_adjustment = motor_schemas.rotate_chain(prev_rab)
    end
    resulting_vector = vector.vec2_polar_sum(adjust_distance, direction_adjustment)
  end

  return resulting_vector
end

function on_prey()
  robot.leds.set_all_colors("red")
  local stica = utils.return_rab_neighbour(RANGE_OF_SENSING_2, function(data) return (data[RAB_PREY_POSITION_INDEX] > 0 and data[RAB_PREY_POSITION_INDEX] < position_in_chain) end)
  if stica ~= nil then
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
    local prey = utils.return_rab_neighbour(RANGE_OF_SENSING_2, function(data) return data[RAB_PREY_POSITION_INDEX] > 0 end)
    if prey ~= nil then
      robot.range_and_bearing.set_data(RAB_PREY_POSITION_INDEX, prey.data[RAB_PREY_POSITION_INDEX])
    end
  end
end

function reset()
  local current_state = SEARCH
  local position_in_chain = 0
end

function destroy()
end
