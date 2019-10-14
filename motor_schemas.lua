local motor_schemas = {}

PROXIMITY_THRESHOLD = 0.1

function motor_schemas.move_straight()
  return {
    length = 1,
    angle = 0
  }
end

function motor_schemas.move_random()
  return {
    length = robot.random.uniform(),
    angle = robot.random.uniform(-math.pi/2, math.pi/2)
  }
end

function motor_schemas.move_perpendicular_monosensor()  -- TODO da fare
  local v = {length = 0, angle = 0}
  local max = 0
  local idx = 0
  for i = 1, 24 do
    if max < (robot.proximity[i].value - PROXIMITY_THRESHOLD) then
      idx = i
      max = max < (robot.proximity[i].value - PROXIMITY_THRESHOLD)
    end
  end
  if max > 0 then
    v.length = max
    v.angle = robot.proximity[idx].angle + (math.pi / 2)
  end
  return v
end

function motor_schemas.avoid_collisions_monosensor() --TODO: use utility function
  local v = {length = 0, angle = 0}
	local max = 0
	local idx = 0
	for i=1,24 do
		if max < (robot.proximity[i].value - PROXIMITY_THRESHOLD) then
			idx = i
			max = (robot.proximity[i].value - PROXIMITY_THRESHOLD)
		end
	end
	if max > 0 then
		v.length = max
		v.angle = robot.proximity[idx].angle
	end
	return v
end

function motor_schemas.avoid_collisions_multisensor()
  local v = { length = 0, angle = 0 }
  local counter = 0
  for i = 1, 24 do
    if robot.proximity[i].value > PROXIMITY_THRESHOLD then
      v = vector.vec2_polar_sum(v, {length = robot.proximity[i].value - PROXIMITY_THRESHOLD, angle = robot.proximity[i].angle + math.pi})
      counter = counter + 1
    end
  end
  if counter ~= 0 then
    v.length = v.length / counter   -- MaybeTODO another possible solution: take max value
  end
  return v
end

function motor_schemas.align(angle_previous, angle_next)
  return {
    --TODO
  }
end

return motor_schemas
