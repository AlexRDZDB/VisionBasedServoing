sim=require'sim'
simROS2=require'simROS2'

function sysCall_init()
    simROS2 = require('simROS2')
    twistSub = simROS2.createSubscription('/cmd_vel', 'geometry_msgs/msg/Twist', 'twistCallback')
    leftMotor = sim.getObjectHandle('/PioneerP3DX/leftMotor')
    rightMotor = sim.getObjectHandle('/PioneerP3DX/rightMotor')

end

function twistCallback(msg)
    linear = msg.linear.x
    angular = msg.angular.z
    wheelRadius = 0.0975
    interWheelDist = 0.381

    leftVel = (linear - angular * interWheelDist / 2) / wheelRadius
    rightVel = (linear + angular * interWheelDist / 2) / wheelRadius

    sim.setJointTargetVelocity(leftMotor, leftVel)
    sim.setJointTargetVelocity(rightMotor, rightVel)
end