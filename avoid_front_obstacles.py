import brickpi
import time

interface=brickpi.Interface()
interface.initialize()

motors = [0,1]

interface.motorEnable(motors[0])
interface.motorEnable(motors[1])

motorParams_0 = interface.MotorAngleControllerParameters()
motorParams_1 = interface.MotorAngleControllerParameters()

motorParams_0.maxRotationAcceleration = 3.0
motorParams_1.maxRotationAcceleration = 3.0

motorParams_0.maxRotationSpeed = 18.0
motorParams_1.maxRotationSpeed = 18.0

motorParams_0.feedForwardGain = 255/20.0
motorParams_1.feedForwardGain = 255/20.0

motorParams_0.minPWM = 16.0
motorParams_1.minPWM = 16.0

motorParams_0.pidParameters.minOutput = -255
motorParams_1.pidParameters.minOutput = -255

motorParams_0.pidParameters.maxOutput = 255
motorParams_1.pidParameters.maxOutput = 255

motorParams_0.pidParameters.k_p = 450
motorParams_1.pidParameters.k_p = 450

motorParams_0.pidParameters.k_i = 420 
motorParams_1.pidParameters.k_i = 420

k_d = 400
motorParams_0.pidParameters.k_d = k_d 
motorParams_1.pidParameters.k_d = k_d  

interface.setMotorAngleControllerParameters(motors[0],motorParams_0)
interface.setMotorAngleControllerParameters(motors[1],motorParams_1)

def rotate(angle1, angle2):
    interface.increaseMotorAngleReferences(motors,[angle1,angle2])
    while not interface.motorAngleReferencesReached(motors) :
        motorAngles = interface.getMotorAngles(motors)
        #if motorAngles :
         #   print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]
        time.sleep(0.1)

    print "Destination reached!"


reverse_length = 4

forty_cm_length = 11.6755
ninety_deg_turn = 3.4875 

left_touch_port = 3
right_touch_port = 2

interface.sensorEnable(left_touch_port, brickpi.SensorType.SENSOR_TOUCH)
interface.sensorEnable(right_touch_port, brickpi.SensorType.SENSOR_TOUCH)

left_touched = 0
right_touched = 0

currX = 0
currY = 0
currAngle = 0

left_touched = interface.getSensorValue(left_touch_port)[0]
right_touched = interface.getSensorValue(right_touch_port)[0]


motorAngles = interface.getMotorAngles(motors)
x = motorAngles[0][0]
y = motorAngles[1][0]
print "Current pos is our origin: " + str(x) + "    " + str(y)
    
interface.setMotorRotationSpeedReferences(motors, [6.0, 6.0])
while not left_touched and not right_touched :
    left_touched = interface.getSensorValue(left_touch_port)[0]
    right_touched = interface.getSensorValue(right_touch_port)[0]
    
motorAngles = interface.getMotorAngles(motors)
currX = motorAngles[0][0] - x
currY = motorAngles[1][0] - y
print "Current pos: " + str(currX) + "    " + str(currX)        

if left_touched or right_touched:
    interface.setMotorPwm(motors[0],0)
    interface.setMotorPwm(motors[1],0)
    rotate(-reverse_length, -reverse_length)
        
motorAngles = interface.getMotorAngles(motors)
currX = motorAngles[0][0] - x
currY = motorAngles[1][0] - y
print "Current pos: " + str(currX) + "    " + str(currX)


interface.terminate()

