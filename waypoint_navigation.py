import brickpi
import time
import random as random
import math

interface=brickpi.Interface()
interface.initialize()
NUMBER_OF_PARTICLES = 100

w1 = (84, 30)
w2 = (180, 30)
w3 = (180, 54)
w4 = (138, 54)
w5 = (138, 168)
w6 = (114, 168)
w7 = (114, 84)
w8 = (84, 84)
w9 = (84, 30)

particles = [(w1[0], w1[1], 0, 0.01)]* NUMBER_OF_PARTICLES

# Adjust if motors turn differently for the same angle.
ANGLE_MULTIPLIER = 1
ROTATION_RADIANS_MULTIPLIER= 2.2969
NINETY_DEG_TURN = 3.77

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

k_d = 0
motorParams_0.pidParameters.K_d = k_d 
motorParams_1.pidParameters.K_d = k_d  

interface.setMotorAngleControllerParameters(motors[0],motorParams_0)
interface.setMotorAngleControllerParameters(motors[1],motorParams_1)

unit_cm = math.pi/10.35

def move(distance):
	# Get the corresponding motor rotation in radians
	_distance = distanceToRobotRadians(distance)

	interface.increaseMotorAngleReferences(motors,[(_distance),(_distance)])
	while not interface.motorAngleReferencesReached(motors) :
		motorAngles = interface.getMotorAngles(motors)
		if motorAngles :
			print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]
		time.sleep(0.1)

	print "Destination reached!"

def rotate(angle):
	_angle = normaliseAngle(angle)
	angle1 = _angle * ROTATION_RADIANS_MULTIPLIER
	angle2 = (_angle * ROTATION_RADIANS_MULTIPLIER) * ANGLE_MULTIPLIER
	interface.increaseMotorAngleReferences(motors,[angle1,-angle2])
	while not interface.motorAngleReferencesReached(motors) :
		motorAngles = interface.getMotorAngles(motors)
		time.sleep(0.1)

	print "Destination reached!"

def getDistanceToTravel(currX, currY, givenX, givenY):
	return math.sqrt((givenX - currX)**2 + (givenY - currY)**2)	

forty_cm_length = 11.6755
unit_cm_length = forty_cm_length/40

def distanceToRobotRadians(distance):
	radiansMultiplier = math.pi/10.35
	return distance * radiansMultiplier

def displayParticles():
    print "drawParticles:" + str([(150 + 15 * x, 50 + 15 * y, theta, weight*50) for (x, y, theta, weight) in particles])

def normaliseAngle(angle):
	if angle > math.pi :
		angle = -(2 * math.pi) + angle
	elif angle < -math.pi :
		angle = (2 * math.pi) + angle

	return angle

def updateMotion(distance, particles):
    print "updating motion"
    newParticles = []
    for (x, y, theta, weight) in particles:
        e = random.gauss(0,0.01)
        f = random.gauss(0,0.01)
        newX = x + (distance + e)*math.cos(theta)
        newY = y + (distance + e)*math.sin(theta)
        newTheta = theta + f
        newParticles.append((newX, newY, newTheta, weight))
    return newParticles

def updateRotation(angle, particles):
    #print("updating Rotation by "  + str(angle * 180/math.pi))
    newParticles = []
    for (x, y, theta, weight) in particles:
        g = random.gauss(0,0.025)
        newTheta = (theta + angle + g)
        newParticles.append((x, y, newTheta , weight))
    return newParticles

def updateCurrentValues(particles):
	xCounter = 0
	yCounter = 0
	thetaCounter = 0
	# Summing up.
	for (x, y, theta, weight) in particles:
		xCounter += x 
		yCounter += y 
		thetaCounter += theta
	# Dividing by total number.
	xCounter *= weight
	yCounter *= weight
	thetaCounter *= weight

	print("This is the angle the particles think they are at: " + str(thetaCounter * 180/math.pi))
	# Normalise angle so it's between pi and -pi
	thetaCounter = normaliseAngle(thetaCounter)

	return (xCounter, yCounter, thetaCounter)

currX = w1[0] 
currY = w1[1]
currAngle = 0

waypoints = [w2, w3, w4, w5, w6, w7, w8, w9]

for (givenX, givenY) in waypoints:
	# Calculate distance and angle
	distance = getDistanceToTravel(currX, currY, givenX, givenY)
	angle = (math.atan2(givenY-currY, givenX-currX)) - currAngle
	angle = normaliseAngle(angle)

	print("currX is:")
	print(currX)
	print("currY is:")
	print(currY)
	print("givenX is:")
	print(givenX)
	print("givenY is:")
	print(givenY)
	print("currAngle is:")
	print(currAngle)

        print("angle: " + str(angle * (180/math.pi)))
	# Rotate
	rotate(angle)
	particles = updateRotation(angle, particles)
	(currX, currY, currAngle) = updateCurrentValues(particles)
	# Move
	move(distance)
	particles = updateMotion(distance, particles)
	(currX, currY, currAngle) = updateCurrentValues(particles)

interface.terminate()

# Debugging printlines.

#	print("currX is:")
#	print(currX)
#	print("currY is:")
#	print(currY)
#	print("givenX is:")
#	print(givenX)
#	print("givenY is:")
#	print(givenY)
#	print("currAngle is:")
#	print(currAngle)

#	print("Angle to turn by")
#	print(angle) if currX - givenX > 0 and currY - givenY > 0:
#		print("Curr x is:")
#		print(currX)
#		print("Curr y is:")
#		print(currY)
#		print("IN 3rd QUADRANT")
#	elif currX - givenX > 0 and currY - givenY < 0:
#		print("IN 2nd QUADRANT")
#	elif currX - givenX < 0 and currY - givenY > 0:
#		print("Curr x is:")
#		print(currX)
#		print("Curr y is:")
#		print(currY)
#		print("IN 4th QUADRANT")
#	elif currX - givenX < 0 and currY - givenY < 0:
#		print("IN 1st QUADRANT")	
#		print("Going to rotate through angle:")
#	print("Distance to travel is:")
#	print(distance)
#	print("angle after quad")
#	print(angle)


#	print("-----DISTANCE--------")
#	print(distance)
#	print("-----//DISTANCE--------")
#	distance = distanceToRobotRadians(distance)
#	print("-----ROBOT DISTANCE--------")
#	print(distance)
#	print("-----//ROBOT DISTANCE--------")

#	print("currX is:")
#	print(currX)
#	print("currY is:")
#	print(currY)
#	print("givenX is:")
#	print(givenX)
#	print("givenY is:")
#	print(givenY)
#	print("currAngle is:")
#	print(currAngle)

#	print("-----DISTANCE--------")
#	print(distance)
#	print("-----//DISTANCE--------")
#	_distance = distanceToRobotRadians(distance)
#	print("-----ROBOT DISTANCE--------")
#	print(_distance)
#	print("-----//ROBOT DISTANCE--------")
