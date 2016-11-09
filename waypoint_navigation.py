import brickpi
import time
import random as random
import math

interface=brickpi.Interface()
interface.initialize()
NUMBER_OF_PARTICLES = 100
particles = [(0, 0, 0, 0.01)]* NUMBER_OF_PARTICLES

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

def rotate(angle1, angle2):
	interface.increaseMotorAngleReferences(motors,[angle1,angle2])
	while not interface.motorAngleReferencesReached(motors) :
		motorAngles = interface.getMotorAngles(motors)
		if motorAngles :
			print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]
		time.sleep(0.1)

	print "Destination reached!"

def getDistanceToTravel(currX, currY, givenX, givenY):
	return math.sqrt((abs(givenX - currX))**2 + (abs(givenY - currY))**2)	

def distanceToRobotRadians(distance):
	radiansMultiplier = 1
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
	newTheta = normaliseAngle(newTheta)
        newParticles.append((newX, newY, newTheta, weight))
    return newParticles

def updateRotation(angle, particles):
    print "updating Rotation"
    newParticles = []
    for (x, y, theta, weight) in particles:
        g = random.gauss(0,0.025)
        newTheta = (theta + angle + g)
	newTheta = normaliseAngle(newTheta)
        newParticles.append((x, y, newTheta , weight ))
    return newParticles

def updateCurrentValues(particles):
	xCounter = 0
	yCounter = 0
	thetaCounter = 0
	for (x, y, theta, weight) in particles:
		xCounter += x * weight
		yCounter += y * weight
		thetaCounter += theta * weight
	return (xCounter, yCounter, thetaCounter)


#forty_cm_length = 11.6755
#ten_cm_length = forty_cm_length/4
ninety_deg_turn = 3.6075
currX = 0
currY = 0
currAngle = 0
rotationRadiansMultiplier = 2.2969

while True:
	givenX = float(input("Give an X co-ordinate for the robot to navigate to:\n"))
	givenY = float(input("Give a Y co-ordinate:\n"))
	distance = getDistanceToTravel(currX, currY, givenX, givenY)
	robotDistance = distanceToRobotRadians(distance) 
	angle = (math.atan2(givenY-currY, givenX-currX)) - currAngle

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

	angle = normaliseAngle(angle)

	#angle *= rotationRadiansMultiplier
	rotate(angle * rotationRadiansMultiplier, -angle * rotationRadiansMultiplier)

	#particles = (updateRotation(angle/rotationRadiansMultiplier, particles))
	print("UPDATING PARTICLES WITH THE ANGLE:")
	print(angle)
	particles = (updateRotation(angle, particles))
	(currX, currY, currAngle) = updateCurrentValues(particles)
	print(str(particles))
	rotate(robotDistance, robotDistance)
	particles = (updateMotion(robotDistance, particles))
        print(str(particles))
	(currX, currY, currAngle) = updateCurrentValues(particles)
	print("new currX is:")
	print(currX)
	print("new currY is:")
	print(currY)
	print("new currAngle is:")
	print(currAngle)


interface.terminate()

