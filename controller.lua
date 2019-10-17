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

  log("mg3: " .. robot.motor_ground[3].value)
  local sum = vector.vec2_polar_sum(motor_schemas.move_straight(), motor_schemas.move_random())
  --local sum = vector.vec2_polar_sum(
  --                                      , motor_schemas.avoid_collisions_monosensor())
  local vel_l, vel_r = motor_conversions.vec_to_vels(sum, robot.wheels.axis_length)
  robot.wheels.set_velocity(vel_l, vel_r)
end
