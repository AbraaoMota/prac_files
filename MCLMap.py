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
w2 = (180, 30)
w3 = (180, 54)
w4 = (138, 54)
w5 = (138, 168)
w6 = (114, 168)
w7 = (114, 84)
w8 = (84, 84)
w9 = (84, 30)

# Functions to generate some dummy particles data:
def calcX():
    return random.gauss(80,3) + 70*(math.sin(t)); # in cm

def calcY():
    return random.gauss(70,3) + 60*(math.sin(2*t)); # in cm

def calcW():
    return random.random();

def calcTheta():
    return random.randint(0,360);


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
        display = [(self.__screenX(d[0]),self.__screenY(d[1])) + d[2:] for d in data];
        print "drawParticles:" + str(display);

    def __screenX(self,x):
        return (x + self.margin)*self.scale

    def __screenY(self,y):
        return (self.map_size + self.margin - y)*self.scale

# A Map class containing walls
class Map:
    def __init__(self):
        self.walls = [];

    def add_wall(self,wall):
        self.walls.append(wall);

    def clear(self):
        self.walls = [];

    def draw(self):
        for wall in self.walls:
            canvas.drawLine(wall);

# Simple Particles set
class Particles:
    def __init__(self):
        self.n = 100;    
        self.data = [(w1[0], w1[1], 0, 0.5)]*NUMBER_OF_PARTICLES;

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
        m = (((By - Ay) * (Ax - x) - (Bx - Ax ) * (Ay - y))/((By - Ay ) * cos(theta) - (Bx - Ax ) * sin(theta)))
        #xWall = x + m*cos(theta)
        #yWall = y + m*sin(theta)
        return m

    #returns likelihood based on position and sonar measurement
    def calculate_likelihood(self, x, y, theta, z):
        # Find the wall closest to particle
        # Calculate the distance to wall.
        m = self.calculate_fwd_distance(x, y, theta, self.findWall((x, y, theta)))
        #Check which one works between m and mWall (to compare with z)
        #Calculate difference (sonar compensates for 2cm addition)
        #If incidence angle is too big for sensible readings, skip update
        d = z - m - 2 
        gauss = self.gaussian_estimate(d)
        return gauss


    def gaussian_estimate(self, d):
        # Random pick for now
        sd = 0.025
        # Each particle gets at least 0.5% chance of occurring
        const = 0.005
        return math.exp(-(d**2)/(2 * sd**2)) + const

    def findWall(self, position):
        (x, y, theta) = position
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
        return wall

    
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




particles = Particles();

t = 0;
#print "updating: "
#particles.updateM(20);
#particles.draw();
#print(findWall((84,30,radians(0))))
#print(findWall((84,30,radians(90))))
#print(findWall((84,30,radians(180))))
#print(findWall((84,30,radians(270))))
#print(findWall((42,140,radians(0))))
#print(findWall((100, 140, radians(180))))
t += 0.05;
time.sleep(0.05);



for i in range(0,1):
    for j in range(0,1):
        #rotate(right_wheel_strength_multiplier * ten_cm_length,  left_wheel_strength_multiplier * ten_cm_length)
        particles.updateM(10)
        particles.updateParticles()
        #particles = updateMotion(10, particles) # only the last set of particles is displayed
        particles.draw()
        time.sleep(0.25)
    #rotate(right_wheel_strength_multiplier * ninety_deg_turn, left_wheel_strength_multiplier * -ninety_deg_turn)
    time.sleep(0.25)
    # we use pi/2 because the degrees are in radians.
    particles.updateR(math.pi/2)
    particles.updateParticles()
    #particles = updateRotation(math.pi/2, particles) # only the last set of particles is displayed
    particles.draw()
    
#interface.terminate()
