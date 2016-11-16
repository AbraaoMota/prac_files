#!/usr/bin/env python 

# Some suitable functions and data structures for drawing a map and particles

import time
import random
import math
import brickpi
import numpy as np
import math

def radians(d):
    return math.radians(d)

def cos(d):
    return math.cos(d)

def sin(d):
    return math.sin(d)

range1 = lambda start, end: range(start, end+1)

NUMBER_OF_PARTICLES = 100
# Adjust if motors turn differently for the same angle.
ANGLE_MULTIPLIER = 1
ROTATION_RADIANS_MULTIPLIER= 2.2969
NINETY_DEG_TURN = 3.77


interface=brickpi.Interface()
interface.initialize()


motors = [0,1]

interface.motorEnable(motors[0])
interface.motorEnable(motors[1])

sonarPort = 1
interface.sensorEnable(sonarPort, brickpi.SensorType.SENSOR_ULTRASONIC);

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

forty_cm_length = 11.6755
ten_cm_length = forty_cm_length/4
ninety_deg_turn = 3.6575 
left_wheel_strength_multiplier  = 1
right_wheel_strength_multiplier = 1

# Definitions of waypoints
w1 = (84, 30)
# 96
# x=1.84

w2 = (180, 30)
w3 = (180, 54)
w4 = (138, 54)
w5 = (138, 168)
w6 = (114, 168)
w7 = (114, 84)
w8 = (84, 84)
w9 = (84, 30)

# A Canvas class for drawing a map and particles:
#     - it takes care of a proper scaling and coordinate transformation between
#      the map frame of reference (in cm) and the display (in pixels)
class Canvas:
    def __init__(self,map_size=210):
        self.map_size    = map_size;    # in cm;
        self.canvas_size = 768;         # in pixels;
        self.margin      = 0.05*map_size;
        self.scale       = self.canvas_size/(map_size+2*self.margin);

    def drawLine(self,line):
        x1 = self.__screenX(line[0]);
        y1 = self.__screenY(line[1]);
        x2 = self.__screenX(line[2]);
        y2 = self.__screenY(line[3]);
        print "drawLine:" + str((x1,y1,x2,y2))

    def drawParticles(self,data):
        display = [(self.__screenX(d[0]),self.__screenY(d[1]), d[2], d[3] * 100) for d in data];
        print "drawParticles:" + str(display);

    def __screenX(self,x):
        return (x + self.margin)*self.scale

    def __screenY(self,y):
        return (self.map_size + self.margin - y)*self.scale

# A Map class containing walls
class Map:
    def __init__(self):
        self.walls = [];
        self.WP = [];

    def add_wall(self,wall):
        self.walls.append(wall);
        
    def add_WP(self, x, y):
        self.WP.append(x+y);

    def clear(self):
        self.walls = [];

    def draw(self):
        for wall in self.walls:
            canvas.drawLine(wall);
    def drawWP(self):
        for wayp in self.WP:
            canvas.drawLine(wayp);
            
            
def getDistanceToTravel(currX, currY, givenX, givenY):
    return math.sqrt((givenX - currX)**2 + (givenY - currY)**2)  
            
    
    
def normaliseAngle(angle):
    if angle > math.pi : 
        angle = -(2 * math.pi) + angle
    elif angle < -math.pi :
        angle = (2 * math.pi) + angle

    return angle

def distanceToRobotRadians(distance):
    radiansMultiplier = math.pi/10.35
    return distance * radiansMultiplier

def rotate(angle):
    _angle = normaliseAngle(angle)
    angle1 = _angle * ROTATION_RADIANS_MULTIPLIER
    angle2 = (_angle * ROTATION_RADIANS_MULTIPLIER) * ANGLE_MULTIPLIER
    interface.increaseMotorAngleReferences(motors,[angle1,-angle2])
    while not interface.motorAngleReferencesReached(motors) :
        motorAngles = interface.getMotorAngles(motors)
        #if motorAngles :
            #print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]
        time.sleep(0.1)

    print "Destination reached!"

    
        
def move(distance):
    # Get the corresponding motor rotation in radians
    _distance = distanceToRobotRadians(distance)
    interface.increaseMotorAngleReferences(motors,[(_distance),(_distance)])

    while not interface.motorAngleReferencesReached(motors) :
        motorAngles = interface.getMotorAngles(motors)
        #if motorAngles :
            #print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]
        time.sleep(0.1)
    
    print "Destination reached!"
    
            
# Simple Particles set
class Particles:
    def __init__(self):
        self.n = 100;    
        self.data = [(w1[0], w1[1], 0, 0.01)]*NUMBER_OF_PARTICLES;

    def updateM(self, distance):
        print "updating motion particles"
        newParticles = []
        for (x, y, theta, weight) in self.data[-100:]:
            # To get correct sd we perform the following calc:
            rne = random.gauss(0,0.5)
            rnf = random.gauss(0,0.02)
            rnh = random.gauss(0,0.01)

            e = math.sqrt((distance/10) * math.pow(rne,2))
            f = math.sqrt((distance/10) * math.pow(rnf,2))
            h = math.sqrt((distance/10) * math.pow(rnh,2))
            newX = x + (distance + e * np.sign(rne)) * math.cos(theta)
            newY = y + (distance + h * np.sign(rnh)) * math.sin(theta)
            newTheta = theta + f * np.sign(rnf)
            newParticles.append((newX, newY, newTheta, weight))
        self.data = newParticles    
        #self.data = [(calcX(), calcY(), calcTheta(), calcW()) for i in range(self.n)];
    
    # After each rotation, take a sonar measurement
    # Update all particles by calling likelihood on each

    def calculate_fwd_distance(self, x, y, theta, wall):
        (Ax, Ay, Bx, By) = wall
        m = (abs(By - Ay) * abs(Ax - x) - abs(Bx - Ax) * abs(Ay - y))/(abs(By - Ay) * cos(theta) - abs(Bx - Ax) * sin(theta))
        #xWall = x + m*cos(theta)
        #yWall = y + m*sin(theta)
        return abs(m)

    #returns likelihood based on position and sonar measurement
    def calculate_likelihood(self, x, y, theta, z):
        # Find the wall closest to particle
        # Calculate the distance to wall.
        (wall, minD) = self.findWall((x, y, theta))
        m = self.calculate_fwd_distance(x, y, theta, wall)
        #print("wall and m: " + str(wall) + str(m))
        print("M IS: " + str(m))
        #Check which one works between m and mWall (to compare with z)
        #Calculate difference (sonar compensates for 2cm addition)
        #If incidence angle is too big for sensible readings, skip update
        SENSOR_ACCURACY = -2
        DIST_SENSOR_CENTER = 4
        SENSOR_UPPER_BOUND = 150
        SENSOR_LOWER_BOUND = 10
        
        d = z - m + SENSOR_ACCURACY + DIST_SENSOR_CENTER
        if(m > SENSOR_UPPER_BOUND or m < SENSOR_LOWER_BOUND):
            gauss = 1
        else:
            gauss = self.gaussian_estimate(d)
        return gauss
    
    #forty_cm_length = 11.6755
    #unit_cm_length = forty_cm_length/40


    def displayParticles():
        print "drawParticles:" + str([(150 + 15 * x, 50 + 15 * y, theta, weight) for (x, y, theta, weight) in particles])


    def updateCurrentValues(self):
        xCounter = 0
        yCounter = 0
        thetaCounter = 0
        # Summing up.
        for (x, y, theta, weight) in self.data:
            xCounter += x 
            yCounter += y
            thetaCounter += theta
            
        xCounter /= NUMBER_OF_PARTICLES
        yCounter /= NUMBER_OF_PARTICLES
        thetaCounter /= NUMBER_OF_PARTICLES
        
        #print("This is the angle the particles think they are at: " + str(thetaCounter * 180/math.pi))
#        print("This is the x and the y where the particles think they are at: " + str(x) + ", " + str(y)) # Normalise angle so it's between pi and -pi
        thetaCounter = normaliseAngle(thetaCounter)
        print("This is the angle the particles think they are at after normalising: " + str(thetaCounter * 180/math.pi))

        return (xCounter, yCounter, thetaCounter)


    def gaussian_estimate(self, d):
        # Random pick for now
        #sd = 0.025
        sd = 3
        # Each particle gets at least 0.5% chance of occurring
        const = 0.0001
        return math.exp(-(d**2)/(2 * sd**2)) + const

    def findWall(self, position):
        (x, y, theta) = position
        y = int(y)
        x = int(x)
        minDist = 1049
        wall = mymap.walls[0]
        for (x1, y1, x2, y2) in mymap.walls:
            if theta >= radians(315) or theta < radians(45):
                if x < x1 and y in range1(*sorted((y1, y2))) and x1 == x2:
                    if(x1-x < minDist):
                        minDist = x1-x
                        wall = (x1, y1, x2, y2)
            if theta >= radians(45) and theta < radians(135):
                if y < y1 and x in range1(*sorted((x1, x2))) and y1 == y2:
                    if(y1-y < minDist):
                        minDist = y1-y
                        wall = (x1, y1, x2, y2)
            if theta >= radians(135) and theta < radians(225):
                if x > x1 and y in range1(*sorted((y1, y2))) and x1 == x2:
                    if(x-x1 < minDist):
                        minDist = x-x1
                        wall = (x1, y1, x2, y2)
            if theta >= radians(225) and theta < radians(315):
                if y > y1 and x in range1(*sorted((x1, x2))) and y1 == y2:
                    if(y-y1 < minDist):
                        minDist = y-y1
                        wall = (x1, y1, x2, y2)
        return (wall, minDist)

    
    def takeSonarM(self):
        sonarAvg = 0
        for i in range(0, 3):
            usReading = interface.getSensorValue(sonarPort)
            if usReading :
                sonarAvg += usReading[0]
        return sonarAvg/3

    def updateParticles(self):
        # Take sonar measurement:
        sonar = self.takeSonarM()
        print("I HAVE TAKEN SONAR: " + str(sonar))
        newParticles = []
        # For each particle, calculate likelihood and add it to the new Particles with updated weight
        for (x, y, theta, weight) in self.data:
            weight *= self.calculate_likelihood(x, y, theta, sonar)
            newParticles.append((x,y,theta,weight))
        self.data = newParticles

    def updateR(self, angle):
        newParticles = []
        for (x, y, theta, weight) in self.data:
            g = random.gauss(0,0.025)
            newTheta = theta + angle + g
            newParticles.append((x, y, newTheta, weight))
        self.data = newParticles

    def normalise(self):
        sum = 0
        newParticles = []

        for(x, y, theta, weight) in self.data:
            sum += weight
        for (x, y, theta, weight) in self.data:
            newParticles.append((x, y, theta, weight / sum))

        self.data = newParticles

    def resample(self):
        lotteryTicketParticles = []
        result                 = []
#        print(self.data)
        for (x1, y1, theta1, weight1) in self.data:
            for i in range(0, int(round(weight1 * NUMBER_OF_PARTICLES))):
                lotteryTicketParticles.append((x1, y1, theta1, weight1))
        for j in range(0, NUMBER_OF_PARTICLES):
            randomIndex = random.randint(0, len(lotteryTicketParticles) - 1)
            (newX, newY, newTheta, newWeight) = lotteryTicketParticles[randomIndex]
            result.append((newX, newY, newTheta, newWeight))
#        print(result)
        self.data = result

    def draw(self):
        canvas.drawParticles(self.data);

canvas = Canvas();    # global canvas we are going to draw on

mymap = Map();
# Definitions of walls
# a: O to A
# b: A to B
# c: C to D
# d: D to E
# e: E to F
# f: F to G
# g: G to H
# h: H to O
mymap.add_wall((0,0,0,168));        # a
mymap.add_wall((0,168,84,168));     # b
mymap.add_wall((84,126,84,210));    # c
mymap.add_wall((84,210,168,210));   # d
mymap.add_wall((168,210,168,84));   # e
mymap.add_wall((168,84,210,84));    # f
mymap.add_wall((210,84,210,0));     # g
mymap.add_wall((210,0,0,0));        # h
mymap.draw();


mymap.add_WP(w1, w2)
mymap.add_WP(w2, w3)
mymap.add_WP(w3, w4)
mymap.add_WP(w4, w5)
mymap.add_WP(w5, w6)
mymap.add_WP(w6, w7)
mymap.add_WP(w7, w8)
mymap.add_WP(w8, w9)
mymap.drawWP()

particles = Particles();

# Definitions of waypoints
#w1 = (84, 30)
#w2 = (180, 30)
#w3 = (180, 54)
#w4 = (138, 54)
#w5 = (138, 168)
#w6 = (114, 168)
#w7 = (114, 84)
#w8 = (84, 84)
#w9 = (84, 30)

waypoints = [w2, w3, w4, w5, w6, w7, w8, w9]

currX = w1[0]
currY = w1[1]
currAngle = 0

for (givenX, givenY) in waypoints:
    # Calculate distance and angle
    distance = getDistanceToTravel(currX, currY, givenX, givenY)
    angle = (math.atan2(givenY-currY, givenX-currX)) - currAngle
    angle = normaliseAngle(angle)

    print("angle to turn: " + str(angle * (180/math.pi)))

    # Rotate
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
    rotate(angle)
    particles.updateR(angle)
    particles.updateParticles()
    particles.normalise()
    particles.resample()
    (currX, currY, currAngle) = particles.updateCurrentValues()
    particles.draw()
    # Move in 20 cm steps.
    while (distance > 20):
        move(20)
        distance -= 20
        particles.updateM(20)
        particles.updateParticles()
        particles.normalise()
        particles.resample()
        (currX, currY, currAngle) = particles.updateCurrentValues()
        #print(str(currX) + " " + str(currY)+ " " + str(currAngle) + "PRINTING CURRENT")
        particles.draw()
        #print("Finished drawing particles")
        time.sleep(0.25)
        #print(particles.data)
    
    move(distance)
    particles.updateM(distance)
    particles.updateParticles()
    particles.normalise()
    particles.resample()
    (currX, currY, currAngle) = particles.updateCurrentValues()
    particles.draw()
    #print("Finished drawing particles")
    time.sleep(0.25)
    #print(particles.data)


#for i in range(0,1):
#    for j in range(0,1):
#        #rotate(right_wheel_strength_multiplier * ten_cm_length,  left_wheel_strength_multiplier * ten_cm_length)
#        particles.updateM(20)
#        particles.updateParticles()
#        particles.normalise()
#        particles.resample()
#        #particles = updateMotion(10, particles) # only the last set of particles is displayed
#        particles.draw()
#        print("Finished drawing particles")
#        time.sleep(0.25)
#    #rotate(right_wheel_strength_multiplier * ninety_deg_turn, left_wheel_strength_multiplier * -ninety_deg_turn)
#    # we use pi/2 because the degrees are in radians.
#    particles.updateR(3*math.pi/2)
#    particles.updateParticles()
#    particles.normalise()
#    particles.resample()
#    particles.updateM(20)
#    particles.updateParticles()
#    particles.normalise()
#    particles.resample()
#    print("Finished drawing particles")
#    #particles = updateRotation(math.pi/2, particles) # only the last set of particles is displayed
#    particles.draw()
#    time.sleep(0.25)
    
#interface.terminate()
