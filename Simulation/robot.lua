local BASE_SPEED  = 3.0
local TURN_SPEED  = 2.5
local INNER_SPEED = 0.3

local leftMotor, rightMotor
local leftSensor, middleSensor, rightSensor
local robotBody
local stepCount = 0
local lapCount = 0
local startPos = nil
local hasLeftStartZone = false
local isTrackingStarted = false
local MIN_STEPS_PER_LAP = 300
local stepsSinceLastLap = 0
local ui
local isStopped = false

function stopButton_callback(uiHandle, id, newVal)
    isStopped = true
    setMotors(0, 0)
end

function updateUI()
    if ui then
        simUI.setLabelText(ui, 100, "Laps: " .. lapCount)
    end
end

function sysCall_init()
    leftMotor    = sim.getObjectHandle('DynamicLeftJoint')
    rightMotor   = sim.getObjectHandle('DynamicRightJoint')
    leftSensor   = sim.getObjectHandle('LeftSensor')
    middleSensor = sim.getObjectHandle('MiddleSensor')
    rightSensor  = sim.getObjectHandle('RightSensor')
    robotBody    = sim.getObjectHandle('LineTracerBase')

    local xml = [[
        <ui title="Control Panel" closeable="false" layout="vbox">
            <label text="Laps: 0" id="100" style="font-size: 20px; font-weight: bold; qproperty-alignment: 'AlignCenter';" />
            <button text="EMERGENCY STOP" on-click="stopButton_callback" style="background-color: #ff0000; color: white; font-weight: bold; padding: 10px;" />
        </ui>
    ]]
    ui = simUI.create(xml)
end

local function getDistance(p1, p2)
    if not p1 or not p2 then return 999 end
    return math.sqrt((p1[1]-p2[1])^2 + (p1[2]-p2[2])^2)
end

local function getRawBrightness(handle)
    local image = sim.getVisionSensorImage(handle)
    if image == nil then return nil end
    local total = 0
    for i = 1, #image do total = total + image[i] end
    return total / #image
end

local rangeDetected, isNormalized = false, true
local function autoDetectRange(val)
    if not rangeDetected and val ~= nil then
        isNormalized = (val <= 1.0)
        rangeDetected = true
    end
end

local function normalize(val)
    if val == nil then return 1.0 end
    return isNormalized and val or val / 255.0
end

local function readSensor(handle)
    local raw = getRawBrightness(handle)
    autoDetectRange(raw)
    local brightness = normalize(raw)
    return (brightness < 0.5), brightness
end

local function setMotors(left, right)
    sim.setJointTargetVelocity(leftMotor,  left)
    sim.setJointTargetVelocity(rightMotor, right)
end

function sysCall_actuation()
    if isStopped then
        setMotors(0, 0)
        return
    end

    stepCount = stepCount + 1
    stepsSinceLastLap = stepsSinceLastLap + 1

    local L = readSensor(leftSensor)
    local M = readSensor(middleSensor)
    local R = readSensor(rightSensor)

    if (L or M or R) and stepCount > 10 then
        isTrackingStarted = true
        startPos = sim.getObjectPosition(robotBody, -1)
        stepsSinceLastLap = 0
        print('--- START POINT LOCKED on black line at step ' .. stepCount .. ' ---')
    end

    if isTrackingStarted and startPos then
        local currentPos = sim.getObjectPosition(robotBody, -1)
        local distFromStart = getDistance(currentPos, startPos)

        if not hasLeftStartZone and distFromStart > 0.8 then
            hasLeftStartZone = true
            print('--- LEFT START ZONE ---')
        end

        local enoughStepsPassed = stepsSinceLastLap > MIN_STEPS_PER_LAP

        if hasLeftStartZone and distFromStart < 0.15 and enoughStepsPassed then
            lapCount = lapCount + 1
            hasLeftStartZone = false
            stepsSinceLastLap = 0
            updateUI()
            print('[LAP] Finished Lap #' .. lapCount)
        end
    end

    if M and not L and not R then
        setMotors(BASE_SPEED, BASE_SPEED)

    elseif L and not M and not R then
        setMotors(INNER_SPEED, TURN_SPEED)

    elseif R and not M and not L then
        setMotors(TURN_SPEED, INNER_SPEED)

    elseif L and M and not R then
        setMotors(INNER_SPEED, BASE_SPEED)

    elseif R and M and not L then
        setMotors(BASE_SPEED, INNER_SPEED)

    elseif L and R then
        setMotors(BASE_SPEED, BASE_SPEED)

    else
        setMotors(BASE_SPEED * 0.4, BASE_SPEED * 0.4)
    end
end

function sysCall_cleanup()
    setMotors(0, 0)
    if ui then
        simUI.destroy(ui)
    end
end
