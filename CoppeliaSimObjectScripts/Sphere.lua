function sysCall_init()
    sphereHandle = sim.getObject('/Sphere')
    amplitude = 2       -- Range from -2 to 2
    frequency = 0.2     -- Hz (cycles per second) ? adjust for speed
    phase = 0.0
    startTime = sim.getSimulationTime()
    
    -- Get original position to preserve X and Z
    originalPosition = sim.getObjectPosition(sphereHandle, -1)
end

function sysCall_actuation()
    local timeElapsed = sim.getSimulationTime() - startTime
    local y = amplitude * math.sin(2 * math.pi * frequency * timeElapsed + phase)
    
    -- Set the sphere's new position
    local newPos = {originalPosition[1], y, originalPosition[3]}
    sim.setObjectPosition(sphereHandle, -1, newPos)
end
