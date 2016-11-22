import brickpi
import time
import random as random
import math

interface=brickpi.Interface()
interface.initialize()
NUMBER_OF_PARTICLES = 100
particles = [(0, 0, 0, 0.5)]*NUMBER_OF_PARTICLES  

motors = [0,1]

interface.motorEnable(motors[0])
interface.motorEnable(motors[1])

motorParams_0 = interface.MotorAngleControllerParameters()
motorParams_1 = interface.MotorAngleControllerParameters()

motorParams_0.maxRotationAcceleration = 6.0
motorParams_1.maxRotationAcceleration = 6.0

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

motorParams_0.pidParameters.k_i = 340 
motorParams_1.pidParameters.k_i = 340

k_d = 400
motorParams_0.pidParameters.k_d = k_d 
motorParams_1.pidParameters.k_d = k_d  

interface.setMotorAngleControllerParameters(motors[0],motorParams_0)
interface.setMotorAngleControllerParameters(motors[1],motorParams_1)



def rotate(angle1, angle2):
    interface.increaseMotorAngleReferences(motors,[angle1,angle2])
    while not interface.motorAngleReferencesReached(motors) :
        motorAngles = interface.getMotorAngles(motors)
        if motorAngles :
            print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]
        time.sleep(0.1)

    print "Destination reached!"

forty_cm_length = 11.6755
ten_cm_length = forty_cm_length/4
ninety_deg_turn = 3.4875 
left_wheel_strength_multiplier  = 1
right_wheel_strength_multiplier = 1


sonarPort = 1
interface.sensorEnable(sonarPort, brickpi.SensorType.SENSOR_ULTRASONIC);

while True:
        usReading = interface.getSensorValue(sonarPort)
        if usReading :
            print usReading[0]
        else:
            print "Failed US reading"
        time.sleep(0.05)




interface.terminate()
