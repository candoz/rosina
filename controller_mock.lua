range_of_sensing = 60
s = 0.015
w = 0.1
alpha = 0.1
beta = 0.05
state = 0

function init()
	robot.leds.set_all_colors("black")
end

function step()
	stopped_neighbors = count_stopped_neighbors()
	if state == 0 then
		stop_probability = math.min(0.99, s + alpha * stopped_neighbors)
		if math.random() < stop_probability then
			state = 1
			stop()
		else
			wander()
		end
	else
		wander_probability = math.max(0.01, w - beta * stopped_neighbors)
		if math.random() < wander_probability then
			state = 0
			wander()
		else
			stop()
		end
	end
end

function count_stopped_neighbors()
	number_robot_sensed = 0
	for _, rab in ipairs(robot.range_and_bearing) do
		if rab.range < range_of_sensing and rab.data[1] == 1 then
			number_robot_sensed = number_robot_sensed + 1
		end
	end
	return number_robot_sensed
end

function wander()
	robot.range_and_bearing.set_data(1, 0)
	left_v = robot.random.uniform(0, 15)
	right_v = robot.random.uniform(0, 15)
	robot.wheels.set_velocity(left_v, right_v)
end

function stop()
	robot.range_and_bearing.set_data(1, 1)
	robot.wheels.set_velocity(0, 0)
end

function reset()
	robot.leds.set_all_colors("black")
end

function destroy()
end
