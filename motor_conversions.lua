local motor_conversions = {}

-- Convert from force (~velocitiy) vector to polar coordinates to wheels' linear velocities
function motor_conversions.vec_to_vels(vec, distance)
  local vel_l = vec.length - distance * vec.angle / 2
  local vel_r = vec.length + distance * vec.angle / 2
  return vel_l, vel_r
end

-- Convert from wheels' linear velocities to velocity (~force) vector in polar coordinates
function motor_conversions.vels_to_vec(vel_l, vel_r, distance)
  return {
    length = (vel_l + vel_r) / 2,
    angle = (vel_r - vel_l) / distance
  }
end

return motor_conversions
