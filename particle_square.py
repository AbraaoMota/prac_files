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

motorParams_0.pidParameters.k_p = 275
motorParams_1.pidParameters.k_p = 275

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

forty_cm_length = 11.6755
ten_cm_length = forty_cm_length/4
ninety_deg_turn = 3.6075 
left_wheel_strength_multiplier  = 1
right_wheel_strength_multiplier = 1


def displayParticles():
    print "drawParticles:" + str([(150 + 15 * x, 50 + 15 * y, theta, weight) for (x, y, theta, weight) in particles])

def updateMotion(distance, particles):
    print "updating motion"
    newParticles = []
    for (x, y, theta, weight) in particles[-100:]:
        e = random.gauss(0,0.01)
        f = random.gauss(0,0.01)
        newX = x + (distance + e)*math.cos(theta)
        newY = y + (distance + e)*math.sin(theta)
        newTheta = theta + f
        newParticles.append((newX, newY, newTheta, weight))
    return newParticles

def updateRotation(angle, particles):
    print "updating Rotation"
    newParticles = []
    for (x, y, theta, weight) in particles[-100:]:
        g = random.gauss(0,0.025)
        newTheta = theta + angle + g
        newParticles.append((x, y, newTheta , weight ))
    return newParticles

for i in range(0,4):
    for j in range(0,4):
        rotate(right_wheel_strength_multiplier * ten_cm_length,  left_wheel_strength_multiplier * ten_cm_length)
        particles.extend(updateMotion(10, particles))
        #particles = updateMotion(10, particles) # only the last set of particles is displayed
        displayParticles()
        time.sleep(0.25)
    rotate(right_wheel_strength_multiplier * -ninety_deg_turn, left_wheel_strength_multiplier * ninety_deg_turn)
    time.sleep(0.25)
    # we use pi/2 because the degrees are in radians.
    particles.extend(updateRotation(math.pi/2, particles))
    #particles = updateRotation(math.pi/2, particles) # only the last set of particles is displayed
    displayParticles()

interface.terminate()

