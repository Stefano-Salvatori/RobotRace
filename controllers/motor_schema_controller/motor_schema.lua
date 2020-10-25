local vector = require "vector"

MAX_DIST = 150
MIN_DIST = 20

MAX_VELOCITY = 20
COURSE_VELOCITY = 4

MIN_PROXIMITY = 10
DIST_SCANNER_RPM = 50

FOREWARD_VECTOR_LEN = 4

PI = 3.1415926535898

ledColor = "black"
half_l = 0


function init()
    half_l = robot.wheels.axis_length / 2
    robot.distance_scanner.enable()
    robot.distance_scanner.set_rpm(DIST_SCANNER_RPM)
end

function stay_on_path()
    local forces = {}
    for i=1,#robot.distance_scanner.long_range do
        local range = robot.distance_scanner.long_range[i]
        forces[i] = {length = 0.0, angle = 0.0}       
        forces[i].angle = -range.angle 
        local d = range.distance
        if d~=-1 and d ~= -2 and d <= MAX_DIST then 
            forces[i].length = ((MAX_DIST - d)/ MAX_DIST)
        end     
    end

    local stay_on_path = {length = 0.0, angle = 0.0}
    for i=1,#forces do
        stay_on_path = vector.vec2_polar_sum(stay_on_path, forces[i])
    end
    return {length = stay_on_path.length * 0.3, angle = stay_on_path.angle}

end

function avoid_obstacoles()
    local forces = {}
    for i=1,#robot.proximity do
        local prox = robot.proximity[i]
        forces[i] = {length = 0.0, angle = 0.0}      
        forces[i].angle = PI/2 + prox.angle 
        forces[i].length = prox.value   
    end

    local avoid_obstacoles = {length = 0.0, angle = 0.0}
    for i=1,#forces do
        avoid_obstacoles = vector.vec2_polar_sum(avoid_obstacoles, forces[i])
    end
    return {length = avoid_obstacoles.length, angle = avoid_obstacoles.angle}
end

function get_robot_angle()
    orientationString = tostring(robot.positioning.orientation)
    i,j = string.find(orientationString, ",", 1)
    return math.rad(tonumber(string.sub(orientationString, 1, i-1)))
end

function vec2str(v)
    return string.format("(%.2f,%.2f°)",v.length,math.deg(v.angle))
end

function step()
    local robotAngle = get_robot_angle()
    log("angle: " .. robotAngle)

    -- stay on path
    local stay_on_path =  stay_on_path()
    log("path: " .. vec2str(stay_on_path))

    -- avoid obstacoles
    local avoid_obstacoles = avoid_obstacoles()
     --local avoid_obstacoles =  {length = 0.0, angle = 0.0}
     log("obs: " .. vec2str(avoid_obstacoles))

         -- compute result vector
    local resultant = {length = 0.0, angle = 0.0}
    resultant = vector.vec2_polar_sum(stay_on_path, avoid_obstacoles)
   
    -- go foreward
    local go_foreward = {length = FOREWARD_VECTOR_LEN , angle =  0.0}
    log("fore: " .. vec2str(go_foreward))


    resultant = vector.vec2_polar_sum(resultant, go_foreward)
    log("res: " .. vec2str(resultant))

    -- transform to differential model
    left_v = ((resultant.length ) - (half_l * resultant.angle)) 
    right_v = ((resultant.length) + (half_l * resultant.angle)) 
    log(string.format("l:%.2f r:%.2f°",left_v,right_v))
    robot.wheels.set_velocity(left_v * COURSE_VELOCITY , right_v * COURSE_VELOCITY )
end

function reset()
end

function destroy()
end
