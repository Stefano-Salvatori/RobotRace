local vector = require "vector"

MAX_LONG_DIST = 150
MIN_LONG_DIST = 20

MAX_SHORT_DIST = 30
MIN_SHORT_DIST = 4

MAX_VELOCITY = 80
COURSE_VELOCITY = 30

MIN_PROXIMITY = 10
DIST_SCANNER_RPM = 100

GO_FOREWORD_VEC_LEN = 0.1

PI = math.pi
half_l = 0


function init()
    half_l = robot.wheels.axis_length / 2
    robot.distance_scanner.enable()
    robot.distance_scanner.set_rpm(DIST_SCANNER_RPM)
end

-- Utils
function sign(i)
    if(i == 0) then return 0 end
    if(i > 0) then return 1 end
    return -1
end

function vec2str(v)
    return string.format("(%.2f,%.2f°)",v.length,math.deg(v.angle))
end

function vec2_polar_summation(vecArray)
    local res = vector.zero()
    for f=1,#vecArray do
        res = vector.vec2_polar_sum(res, vecArray[f])
    end
    return res
end

function math.map(x, in_min, in_max, out_min, out_max) 
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
end
-- 




function stay_on_path()
    local forces = {}
    local i = 1
    for j=1,#robot.distance_scanner.long_range do
        local range = robot.distance_scanner.long_range[j]
        local d = range.distance
        forces[i] = {length = 1, angle = 0.0}         
        if d~=-1 and d ~= -2 and d <= MAX_LONG_DIST and d > MIN_LONG_DIST then 
            forces[i].length = ((MAX_LONG_DIST - d)/ MAX_LONG_DIST)
            forces[i].angle =  - sign(range.angle) * PI /2
            
        end  
        i = i + 1   
    end
    
    for j=1,#robot.distance_scanner.short_range do
        local range = robot.distance_scanner.short_range[j]
        local d = range.distance
        forces[i] = {length = 1, angle = 0.0}         
        if d~=-1 and d ~= -2 and d <= MAX_SHORT_DIST and d > MIN_SHORT_DIST then 
            forces[i].length = ((MAX_SHORT_DIST - d)/ MAX_SHORT_DIST)
            forces[i].angle =  - sign(range.angle) * PI /2
            
        end  
        i = i + 1   
    end

    local summation = vec2_polar_summation(forces)
    return {length = summation.length / #forces  , angle = summation.angle}

end

function avoid_obstacoles(robotAngle)
    local forces = {}
    for i=1,#robot.proximity do
        local prox = robot.proximity[i]
        forces[i] = vector.zero()
        forces[i].length = prox.value    
        forces[i].angle = PI/2 + prox.angle 
    end

    local summation = vec2_polar_summation(forces)
    return {length = summation.length/#forces, angle = summation.angle}
end

function get_robot_angle()
    local orientationString = tostring(robot.positioning.orientation)
    local i,j = string.find(orientationString, ",", 1)
    return math.rad(tonumber(string.sub(orientationString, 1, i-1)))
end



function step()
    local robotAngle = get_robot_angle()
    --log("angle: " .. robotAngle)

    local stay_on_path =  stay_on_path()
    --log("path: " .. vec2str(stay_on_path))

    local avoid_obstacoles = avoid_obstacoles()
    --log("obs: " .. vec2str(avoid_obstacoles))

    local go_foreward = vector.zero()
    go_foreward.length = GO_FOREWORD_VEC_LEN
    go_foreward.angle = -PI/2 - robotAngle
    --log("fore: " .. vec2str(go_foreward))

    -- compute result vector
    local schemas = {stay_on_path, avoid_obstacoles, go_foreward}
    local resultant = vec2_polar_summation(schemas);
    resultant.length = resultant.length / #schemas
    --log("res: " .. vec2str(resultant))

    -- transform to differential model
    left_v = ((resultant.length ) - (half_l * resultant.angle)) 
    right_v = ((resultant.length) + (half_l * resultant.angle)) 

    robot.wheels.set_velocity(math.map(left_v, 0, 1, COURSE_VELOCITY, MAX_VELOCITY)  , math.map(right_v, 0, 1, COURSE_VELOCITY, MAX_VELOCITY)  )
end

function reset()
    half_l = robot.wheels.axis_length / 2
    robot.distance_scanner.enable()
    robot.distance_scanner.set_rpm(DIST_SCANNER_RPM)
end

function destroy()
end
