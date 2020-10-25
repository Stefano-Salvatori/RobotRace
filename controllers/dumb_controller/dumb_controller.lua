local vector = require "vector"

MAX_VELOCITY = 20
FRONT_LEFT_SENSOR_IDX = 1
FRONT_RIGHT_SENSOR_IDX = 24
L = 0

ledColor = "black"

function init()
    left_v = MAX_VELOCITY
    right_v = MAX_VELOCITY
    robot.wheels.set_velocity(left_v,right_v)
    robot.leds.set_all_colors(ledColor)
    L = robot.wheels.axis_length
end

function step()
end

function reset()
end

function destroy()
end
