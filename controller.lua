local MAX_SPEED = 10
vector = require "vector"
motor_schemas = require "motor_schemas"
motor_conversions = require "motor_conversions"

function init()
  robot.leds.set_all_colors("black")
end

function step()
  if robot.motor_ground[1].value > 0.9 or
      robot.motor_ground[2].value > 0.9 or
      robot.motor_ground[3].value > 0.9 or
      robot.motor_ground[4].value > 0.9 then
    robot.leds.set_all_colors("green")

  elseif robot.motor_ground[1].value < 0.1 or
      robot.motor_ground[2].value < 0.1 or
      robot.motor_ground[3].value < 0.1 or
      robot.motor_ground[4].value < 0.1 then
    robot.leds.set_all_colors("red")

  else
    robot.leds.set_all_colors("black")
  end

  local straight = motor_schemas.move_straight()
  local random = motor_schemas.move_random()
  local avoid_mono = motor_schemas.avoid_collisions_monosensor()
  local avoid_multi = motor_schemas.avoid_collisions_multisensor()
  local move_perpendicular = motor_schemas.move_perpendicular_monosensor()
  local sum = vector.vec2_polar_sum(straight, random)
  sum = vector.vec2_polar_sum(sum, avoid_mono)
  -- sum = vector.vec2_polar_sum(sum, avoid_multi)
  sum = vector.vec2_polar_sum(sum, move_perpendicular)
  -- log("straight.length: " .. straight.length .. " straight.angle: " .. straight.angle)
  -- log("random.length: " .. random.length .. " random.angle: " .. random.angle)
  log("mono.length: " .. avoid_mono.length .. " mono.angle: " .. avoid_mono.angle)
  log("multi.length: " .. avoid_multi.length .. " multi.angle: " .. avoid_multi.angle)
  log("perpendicular.length: " .. move_perpendicular.length .. " perpendicular.angle: " .. move_perpendicular.angle)
  -- log("sum.length: " .. sum.length .. " sum.angle: " .. sum.angle)
  local vel_l, vel_r = motor_conversions.vec_to_vels(sum, robot.wheels.axis_length)
  -- log("vel_l: " .. vel_l .. " vel_r: " .. vel_r)
  robot.wheels.set_velocity(vel_l, vel_r)
end
