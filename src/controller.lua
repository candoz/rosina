vector = require "src/vector"
motor_schemas = require "src/motor_schemas"
motor_conversions = require "src/motor_conversions"
utils = require "src/utils"

local CHAIN_BOTS_DISTANCE = 25 -- the distance to keep between the bots that are forming the chain
local EXPLORING_DISTANCE = 25 -- the distance to keep from the chain while exploring it
local LIMITED_RANGE_OF_SENSING, EXTENDED_RANGE_OF_SENSING = 30, 60 -- The two RAB range of sensing considered

-- The five rab.data indexes, each one representing:
local IRAB_STATE = 1            -- the bot's current state;
local IRAB_POSITION = 2         -- the bot's position in chain (nestbot = 1, then 2, 3, 4 ...);
local IRAB_PREYBOT_POSITION = 3 -- the position in chain of the bot ON_PREY
                                --   > Note: if rab.data[IRAB_PREYBOT_POSITION] > 0 it means that the chain is completed;
local IRAB_MAX_POSITION = 4     -- the chain length;
                                --   > Note: if rab.data[IRAB_PREYBOT_POSITION] < rab.data[IRAB_MAX_POSITION] it means that there is at least a bots that need to detach from the chain
local IRAB_FORMED_CHAINS = 5    -- how many chains have already been formed.

function init()
  SEARCH, ON_NEST, EXPLORE_CHAIN, CHAIN_LINK, CHAIN_TAIL, ON_PREY = 1, 2, 3, 4, 5, 6 -- All the possible states that a bot can be in
  BEHAVIOURS = { -- Map the given state to the corresponding behaviour function
    [SEARCH] = search,
    [ON_NEST] = on_nest,
    [EXPLORE_CHAIN] = explore_chain,
    [CHAIN_LINK] = chain_link,
    [CHAIN_TAIL] = chain_tail,
    [ON_PREY] = on_prey
  }

  current_state = SEARCH
  position_in_chain = 0
end

function step()
  local motor_vector = BEHAVIOURS[current_state]()
  local vel_l, vel_r = motor_conversions.vec_to_vels(motor_vector, robot.wheels.axis_length)
  robot.wheels.set_velocity(vel_l, vel_r)
end

function reset()
  current_state = SEARCH
  position_in_chain = 0
end

function destroy()
end

-- SEARCH:

function search()
  robot.leds.set_all_colors("black")
  local resulting_vector = {length = 0, angle = 0}

  local sensing_a_close_nestbot = nil ~= utils.return_rab_neighbour(LIMITED_RANGE_OF_SENSING, function(data) return data[IRAB_STATE] == ON_NEST end)
  local sensing_a_close_linkbot = nil ~= utils.return_rab_neighbour(LIMITED_RANGE_OF_SENSING, function(data) return data[IRAB_STATE] == CHAIN_LINK end)
  local sensing_a_close_tailbot = nil ~= utils.return_rab_neighbour(LIMITED_RANGE_OF_SENSING, function(data) return data[IRAB_STATE] == CHAIN_TAIL end)
  local sensing_a_completed_chain = nil ~= utils.return_rab_neighbour(EXTENDED_RANGE_OF_SENSING, function(data) return data[IRAB_PREYBOT_POSITION] > 0 end)

  if not sensing_a_completed_chain and (sensing_a_close_nestbot or sensing_a_close_linkbot or sensing_a_close_tailbot) then
    current_state = EXPLORE_CHAIN
  elseif not sensing_a_completed_chain and check_ground() == "nest" then
    position_in_chain = 1 -- the nest is at the first position of a chain
    current_state = ON_NEST
  else
    resulting_vector = vector.vec2_polar_sum(motor_schemas.move_straight(), motor_schemas.move_random())
    resulting_vector = vector.vec2_polar_sum(resulting_vector, motor_schemas.avoid_collisions_monosensor())
  end
  return resulting_vector
end

-- ON_NEST:

function on_nest()
  robot.leds.set_all_colors("green")
  emit_chain_info()
  local sensing_a_linkbot = nil ~= utils.return_rab_neighbour(EXTENDED_RANGE_OF_SENSING, function(data) return data[IRAB_STATE] == CHAIN_LINK end)
  local sensing_a_tailbot = nil ~= utils.return_rab_neighbour(EXTENDED_RANGE_OF_SENSING, function(data) return data[IRAB_STATE] == CHAIN_TAIL end)
  if sensing_a_linkbot or sensing_a_tailbot then
    robot.range_and_bearing.set_data(IRAB_FORMED_CHAINS, 1) -- one chain is already formed/forming from this nest
  else
    robot.range_and_bearing.set_data(IRAB_FORMED_CHAINS, 0)
  end
  return {length = 0, angle = 0}
end

-- EXPLORE_CHAIN:

function explore_chain()
  robot.leds.set_all_colors("black")
  local resulting_vector = {length = 0, angle = 0}

  local sensing_a_completed_chain = nil ~= utils.return_rab_neighbour(EXTENDED_RANGE_OF_SENSING, function(data) return data[IRAB_PREYBOT_POSITION] > 0 end)
  local max_position_rab = utils.return_max_rab_neighbour(LIMITED_RANGE_OF_SENSING, IRAB_POSITION)
  local sensing_max_position_rab = nil ~= max_position_rab

  if not sensing_max_position_rab or sensing_a_completed_chain then
    current_state = SEARCH -- lost the chain while exploring it
  
  else
    -- NEST bots around me?
    local close_nestbot = utils.return_rab_neighbour(LIMITED_RANGE_OF_SENSING, function(data) return data[IRAB_STATE] == ON_NEST end)
    local sensing_a_close_nestbot = nil ~= close_nestbot
    local sensing_a_nestbot = nil ~= utils.return_rab_neighbour(EXTENDED_RANGE_OF_SENSING, function(data) return data[IRAB_STATE] == ON_NEST end)
    -- LINK bots around me?
    local sensing_a_close_linkbot = nil ~= utils.return_rab_neighbour(LIMITED_RANGE_OF_SENSING, function(data) return data[IRAB_STATE] == CHAIN_LINK end)
    local sensing_a_linkbot = nil ~= utils.return_rab_neighbour(EXTENDED_RANGE_OF_SENSING, function(data) return data[IRAB_STATE] == CHAIN_LINK end)
    -- TAIL bots close to me?
    local sensing_a_close_tailbot = nil ~= utils.return_rab_neighbour(LIMITED_RANGE_OF_SENSING, function(data) return data[IRAB_STATE] == CHAIN_TAIL end)

    if (sensing_a_close_nestbot and close_nestbot.data[IRAB_FORMED_CHAINS] == 0) or
        (sensing_a_close_tailbot and not sensing_a_linkbot and not sensing_a_nestbot) then
      position_in_chain = max_position_rab.data[IRAB_POSITION] + 1 -- I have now the highest position in the chain
      current_state = CHAIN_TAIL

    else -- keep exploring the chain
      local prev_max_position_rab = utils.return_rab_neighbour(LIMITED_RANGE_OF_SENSING, function(data) return data[IRAB_POSITION] == max_position_rab.data[IRAB_POSITION] - 1 end)
      local sensing_prev_max_position_rab = nil ~= prev_max_position_rab

      local move_perpendicular = nil
      if sensing_prev_max_position_rab then
        move_perpendicular = motor_schemas.follow_chain_direction(max_position_rab, prev_max_position_rab)
      else
        move_perpendicular = motor_schemas.move_perpendicular_to_rab(max_position_rab, false)
      end
      local avoid_mono = motor_schemas.avoid_collisions_monosensor()
      local adjust_distance = motor_schemas.adjust_distance_from_footbot(utils.return_closest_rab_neighbour(EXTENDED_RANGE_OF_SENSING), EXPLORING_DISTANCE)

      resulting_vector = vector.vec2_polar_sum(avoid_mono, move_perpendicular)
      resulting_vector = vector.vec2_polar_sum(resulting_vector, adjust_distance)
    end
  end
  return resulting_vector
end

-- CHAIN_LINK:

function chain_link()
  robot.leds.set_all_colors("white")
  emit_chain_info()
  local resulting_vector = {length = 0, angle = 0}

  if check_ground() == "prey" then
    current_state = ON_PREY
  else

    local prev_bot = utils.return_rab_neighbour(EXTENDED_RANGE_OF_SENSING, function(data) return data[IRAB_POSITION] == position_in_chain - 1 end)
    local sensing_prev_bot = nil ~= prev_bot
    local next_bot = utils.return_rab_neighbour(EXTENDED_RANGE_OF_SENSING, function(data) return data[IRAB_POSITION] == position_in_chain + 1 end)
    local sensing_next_bot = nil ~= next_bot

    -- search for a rab neighbour that knows if the chain is completed (if the prey bot position is > 0)
    local rab_neighbour_bot = utils.return_rab_neighbour(EXTENDED_RANGE_OF_SENSING, function(data) return data[IRAB_PREYBOT_POSITION] > 0 end)
    local sensing_a_completed_chain = nil ~= rab_neighbour_bot
    
    if not sensing_prev_bot or not sensing_next_bot or (sensing_a_completed_chain and position_in_chain > rab_neighbour_bot.data[IRAB_PREYBOT_POSITION]) then
      position_in_chain = 0  -- I'm not part of the chain anymore
      current_state = SEARCH
    
    else
      local avoid_mono = motor_schemas.avoid_collisions_monosensor()
      local adjust_distance_prev = motor_schemas.adjust_distance_from_footbot(prev_bot, CHAIN_BOTS_DISTANCE)
      local max_position_rab = utils.return_max_rab_neighbour(EXTENDED_RANGE_OF_SENSING, IRAB_MAX_POSITION)
      local align = motor_schemas.align(IRAB_POSITION, position_in_chain, max_position_rab.data[IRAB_MAX_POSITION], EXTENDED_RANGE_OF_SENSING)
      
      resulting_vector = vector.vec2_polar_sum(avoid_mono, adjust_distance_prev)
      resulting_vector = vector.vec2_polar_sum(resulting_vector, align)
    end
  end
  return resulting_vector
end

-- CHAIN_TAIL:

function chain_tail()
  robot.leds.set_all_colors("yellow")
  emit_chain_info()
  local resulting_vector = {length = 0, angle = 0}

  if check_ground() == "prey" then
    current_state = ON_PREY
  
  else
    local rab_neighbour_bot = utils.return_rab_neighbour(EXTENDED_RANGE_OF_SENSING, function(data) return data[IRAB_PREYBOT_POSITION] > 0 end)
    local sensing_a_completed_chain = nil ~= rab_neighbour_bot
    local prev_bot = utils.return_rab_neighbour(EXTENDED_RANGE_OF_SENSING, function(data) return data[IRAB_POSITION] == position_in_chain - 1 end)
    local sensing_prev_bot = nil ~= prev_bot
    
    if not sensing_prev_bot or (sensing_a_completed_chain and position_in_chain > rab_neighbour_bot.data[IRAB_PREYBOT_POSITION]) then
      position_in_chain = 0
      current_state = SEARCH
    
    else
      local close_tailbot = utils.return_rab_neighbour(LIMITED_RANGE_OF_SENSING, function(data) return data[IRAB_STATE] == CHAIN_TAIL end)
      local sensing_close_tailbot = nil ~= close_tailbot
      if sensing_close_tailbot and close_tailbot.data[IRAB_POSITION] == position_in_chain + 1 then
        current_state = CHAIN_LINK
      
      else
        local avoid_mono = motor_schemas.avoid_collisions_monosensor()
        local adjust_distance = motor_schemas.adjust_distance_from_footbot(prev_bot, CHAIN_BOTS_DISTANCE)
        local direction_adjustment = motor_schemas.move_perpendicular_to_rab(prev_bot, true)
        resulting_vector = vector.vec2_polar_sum(avoid_mono, adjust_distance)
        resulting_vector = vector.vec2_polar_sum(resulting_vector, direction_adjustment)
      end
    end
  end
  return resulting_vector
end

-- ON_PREY:

function on_prey()
  robot.leds.set_all_colors("red")
  emit_chain_info()

  local sensing_prey_in_lower_position = nil ~= utils.return_rab_neighbour(EXTENDED_RANGE_OF_SENSING, function(data) return (data[IRAB_PREYBOT_POSITION] > 0 and data[IRAB_PREYBOT_POSITION] < position_in_chain) end)
  if sensing_prey_in_lower_position then
    position_in_chain = 0
    current_state = SEARCH
  end
  return { length = 0, angle = 0 }
end

-- controller utils:

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
  if sensors_on_prey >= 1 then return "prey"     -- 1 sensor is enough because we'd like to stop as soon as we see the prey
  elseif sensors_on_nest >= 3 then return "nest" -- 3 sensors because we want to position the nest bot approximately at the center of the nest
  else return "empty_floor" end
end

function emit_chain_info()
  robot.range_and_bearing.set_data(IRAB_STATE, current_state)
  robot.range_and_bearing.set_data(IRAB_POSITION, position_in_chain)
  local max_position_rab = utils.return_max_rab_neighbour(EXTENDED_RANGE_OF_SENSING, IRAB_MAX_POSITION)
  if nil ~= max_position_rab then
    robot.range_and_bearing.set_data(IRAB_MAX_POSITION, math.max(position_in_chain, max_position_rab.data[IRAB_MAX_POSITION]))
  else
    robot.range_and_bearing.set_data(IRAB_MAX_POSITION, position_in_chain)
  end
  if current_state == ON_PREY then
    robot.range_and_bearing.set_data(IRAB_PREYBOT_POSITION, position_in_chain)
  else
    local rab_neighbour_bot = utils.return_rab_neighbour(EXTENDED_RANGE_OF_SENSING, function(data) return data[IRAB_PREYBOT_POSITION] > 0 end)
    local sensing_a_completed_chain = nil ~= rab_neighbour_bot
    if sensing_a_completed_chain then
      robot.range_and_bearing.set_data(IRAB_PREYBOT_POSITION, rab_neighbour_bot.data[IRAB_PREYBOT_POSITION])
    end
  end
end
