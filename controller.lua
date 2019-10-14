local MAX_SPEED = 10
vector = require "vector"
motor_schemas = require "motor_schemas"
motor_conversions = require "motor_conversions"

function init()
  robot.leds.set_all_colors("black")
end

function step()
  -- log("" .. robot.motor_ground[1].value)
  if robot.motor_ground[1].value < 0.9 or
    robot.motor_ground[2].value < 0.9 or
    robot.motor_ground[3].value < 0.9 or
    robot.motor_ground[4].value < 0.9 then

    if robot.motor_ground[1].value < 0.4 or
      robot.motor_ground[2].value < 0.4 or
      robot.motor_ground[3].value < 0.4 or
      robot.motor_ground[4].value < 0.4 then

      robot.leds.set_all_colors("yellow")
    else
      --robot.leds.set_all_colors("red")
    end
  end
  local sum = vector.vec2_polar_sum(vector.vec2_polar_sum(
                                        motor_schemas.move_straight(), motor_schemas.move_random())
                                        , motor_schemas.avoid_collisions_monosensor())
  local final = motor_conversions.vec_to_vels(sum, robot.wheels.axis_length)
  robot.wheels.set_velocity(final.vel_l, final.vel_r)
end
