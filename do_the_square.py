import brickpi
import time

interface=brickpi.Interface()
interface.initialize()

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

motorParams_0.pidParameters.k_i = 420 
motorParams_1.pidParameters.k_i = 420

k_d = 400
motorParams_0.pidParameters.k_d = k_d 
motorParams_1.pidParameters.k_d = k_d  

#Amey Vals
#210
#400
#500
#200

# Values earlier.
#450

# Increase Ki for steady state error.
#400

#200

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


#rotate(11.58,11.58)
#rotate(-3.69,3.69)
#rotate(11.58,11.58)
#rotate(-3.69,3.69)
#rotate(11.58,11.58)
#rotate(-3.69,3.69)
#rotate(11.58,11.58)
rotate(-3.69,3.69)

interface.terminate()

