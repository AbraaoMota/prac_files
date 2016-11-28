#!/usr/bin/env python 

# Some suitable functions and data structures for drawing a map and particles
import os
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
# ROTATION_RADIANS_MULTIPLIER= 2.2969
ROTATION_RADIANS_MULTIPLIER = 3.0625
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

#########################
# SONAR MOTOR
sonar_motor = [2]

interface.motorEnable(sonar_motor[0])

sonar_motor_params = interface.MotorAngleControllerParameters()

sonar_motor_params.maxRotationAcceleration = 6.0
sonar_motor_params.maxRotationSpeed        = 4.0
sonar_motor_params.feedForwardGain         = 400/20.0
sonar_motor_params.minPMW                  = 16.0

sonar_motor_params.pidParameters.minOutput = -255
sonar_motor_params.pidParameters.maxOutput = 255
sonar_motor_params.pidParameters.k_p       = 450
sonar_motor_params.pidParameters.k_i       = 340

k_d = 400
sonar_motor_params.pidParameters.k_d = k_d

interface.setMotorAngleControllerParameters(sonar_motor[0], sonar_motor_params)

sonarPort = 1
interface.sensorEnable(sonarPort, brickpi.SensorType.SENSOR_ULTRASONIC)




###################################################################
###  SIGNATURE STUFF FROM PLACE_REC


# Location signature class: stores a signature characterizing one location
# Location signature stored as an array of size 255 (sonar max range)
# Everytime a depth is recorded, a counter for the element in the array
# at index (recorded depth) is increased, keeping a rotational invariant
# locale signature.

class LocationSignature:
    # def __init__(self, no_bins = 360):
    # def __init__(self, no_bins = 256):
    # def __init__(self, no_bins = 181):
    def __init__(self, no_bins = 37):
        self.sig = [0] * no_bins

    def print_signature(self):
        for i in range(len(self.sig)):
            print self.sig[i]

# --------------------- File management class ---------------
class SignatureContainer():
    def __init__(self, size):
        self.size      = size; # max number of signatures that can be stored
        self.filenames = [];

        # Fills the filenames variable with names like loc_%%.dat
        # where %% are 2 digits (00, 01, 02...) indicating the location number.
        for i in range(self.size):
            self.filenames.append('loc_{0:02d}.dat'.format(i))

    # Get the index of a filename for the new signature. If all filenames are
    # used, it returns -1;
    def get_free_index(self):
        n = 0
        while n < self.size:
            if (os.path.isfile(self.filenames[n]) == False):
                break
            n += 1

        if (n >= self.size):
            return -1;
        else:
            return n;

    # Delete all loc_%%.dat files
    def delete_loc_files(self):
        print "STATUS:  All signature files removed."
        for n in range(self.size):
            if os.path.isfile(self.filenames[n]):
                os.remove(self.filenames[n])

    # Writes the signature to the file identified by index (e.g, if index is 1
    # it will be file loc_01.dat). If file already exists, it will be replaced.
    def save(self, signature, index):
        filename = self.filenames[index]
        if os.path.isfile(filename):
            os.remove(filename)

        f = open(filename, 'w')

        for i in range(len(signature.sig)):
            s = str(signature.sig[i]) + "\n"
            f.write(s)
        f.close();

    # Read signature file identified by index. If the file doesn't exist
    # it returns an empty signature.
    def read(self, index):
        ls = LocationSignature()
        filename = self.filenames[index]
        if os.path.isfile(filename):
            f = open(filename, 'r')
            for i in range(len(ls.sig)):
                s = f.readline()
                if (s != ''):
                    ls.sig[i] = int(s)
            f.close();
        else:
            print "WARNING: Signature does not exist."

        return ls


def spin_motor(motor_rot):
    # spin the motor
    interface.increasemotoranglereferences(sonar_motor, [motor_rot])
    while not interface.motoranglereferencesreached(sonar_motor) :
        # take sonar measurements
        reading      = interface.getsensorvalue(sonarport)
        # record measurements in the signature
        real_angle   = interface.getmotorangles(sonar_motor)[0][0] - init_motor_angle[0][0]
        real_degrees = int(math.degrees(real_angle))

        print("degree bucket is: " + str(real_degrees))
        print("reading is: "       + str(reading[0]))

        if (real_degrees <= 180 and reading[0] > 10):
            ls.sig[int(real_degrees / 5)] += reading[0]
            count[int(real_degrees / 5)]  += 1


# Sonar should spin in small increments (to be decided depending on motor accuracy)
# Should complete a full rotation; on every measurement, increment a counter in each
# element of the signature array at index equal to the depth measured by the sonar.
def characterize_location(ls):
    MOTOR_ROTATION   = math.pi
    init_motor_angle = interface.getMotorAngles(sonar_motor)
    real_angle       = 0
    count            = [0] * 37
    
    # soin the motor
    spin_motor(motor_rotation);
    # unwind the cable 
    spin_motor(-motor_rotation)

    # Print the signature
    for i in range(0, len(ls.sig)):
        if(count[i] != 0):
            ls.sig[i] /= count[i]=
        print("Signature at " + str(i) + " is: " + str(ls.sig[i]))



def compare_signatures(ls1, ls2):
    dist = 0
    s1   = ls1.sig
    s2   = ls2.sig

    for i in range(len(s1)):
        sig1Val = s1[i]
        sig2Val = s2[i]
        
        diff        = sig1Val - sig2Val
        diffSquared = diff * diff
        dist        += diffSquared
    return dist


# This function characterizes the current location, and stores the obtained
# signature into the next available file.
def learn_location():
    ls = LocationSignature()
    characterize_location(ls)
    idx = signatures.get_free_index();

    if (idx == -1): # run out of signature files
        print "\nWARNING:"
        print "No signature file is available. NOTHING NEW will be learned and stored."
        print "Please remove some loc_%%.dat files.\n"
        return

    signatures.save(ls,idx)
    print "STATUS:  Location " + str(idx) + " learned and saved."



# This function tries to recognize the current location.
# 1.   Characterize current location
# 2.   For every learned locations
# 2.1. Read signature of learned location from file
# 2.2. Compare signature to signature coming from actual characterization
# 3.   Retain the learned location whose minimum distance with
#      actual characterization is the smallest.
# 4.   Display the index of the recognized location on the screen
def recognize_location():
    RECOGNITION_THRESHOLD = 13000
    ls_obs = LocationSignature();
    characterize_location(ls_obs);

    sigDists = [0] * signatures.size
    for idx in range(signatures.size):
        ls_read = signatures.read(idx);
        dist    = compare_signatures(ls_obs, ls_read)
        sigDists[idx] = dist

    for i in range(len(sigDists)):
        print("distance is: " + str(sigDists[i]))
        if (sigDists[i] < RECOGNITION_THRESHOLD):
            print("This location is similar to " + str(i))
    print("Tried to recognise")


def find_bottle():
    ACCURATE_THRESHOLD = 160
    MEASURABLE_DIFF = 10

    ls_bottle = LocationSignature()
    characterize_location(ls_bottle)
    bottle_sig = ls_bottle.sig

    # Suppose we're comparing against the first signature
    cached_sig = signatures.read(0)

    contiguous_counter = 0
    bottle_angle = -1

    for i in range(len(bottle_sig)):
        curr = bottle_sig[i]
        cached_curr = cached_sig.sig[i]
        if curr < ACCURATE_THRESHOLD:
            diff = cached_curr - curr
            if diff > MEASURABLE_DIFF:
                contiguous_counter += 1
                if contiguous_counter >= 3:
                    bottle_angle = i - int(contiguous_counter / 2)
            else:
                contiguous_counter = 0

    normalised_angle = bottle_angle * 5
    print("BOTTLE ANGLE IS " + str(bottle_angle) + ", NORMALISED ANGLE IS " + str(normalised_angle))
    angle_to_turn = 90 - normalised_angle

    # answer is in degrees
    # radians variant
    print("WE THINK WE NEED TO TURN " + str(angle_to_turn))
    angle_to_turn = math.radians(angle_to_turn)
    
    #We return -1 if we don't find a contiguous area - i.e. bottle
    if bottle_angle == -1:
	return bottle_angle
    else:
	return angle_to_turn


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

#Definitions of waypoints for OFC
wA1 = w1
wA2 = (115, 30)
wB1 = (124, 73)
wB1b = (124, 133)
wB2 = (94, 94)
wC1 = (73, 94)
wC2 = (40, 53)

objectWaypoints = [wA1, wA2, wB1, wB2, wC1, wC2]

left_touch_port = 3
right_touch_port = 2
reverse_length = 4

interface.sensorEnable(left_touch_port, brickpi.SensorType.SENSOR_TOUCH)
interface.sensorEnable(right_touch_port, brickpi.SensorType.SENSOR_TOUCH)


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

# A Map class containing walls$
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
    print("NORMALISING ANGLE IN ROTATE TO " + str(_angle))
    angle1 = _angle * ROTATION_RADIANS_MULTIPLIER
    angle2 = (_angle * ROTATION_RADIANS_MULTIPLIER) * ANGLE_MULTIPLIER
    interface.increaseMotorAngleReferences(motors,[angle1,-angle2])
    while not interface.motorAngleReferencesReached(motors) :
        motorAngles = interface.getMotorAngles(motors)
        time.sleep(0.1)

    print "Destination reached!"

def not_normalised_rotate(angle):
    angle1 = angle * ROTATION_RADIANS_MULTIPLIER
    angle2 = (angle * ROTATION_RADIANS_MULTIPLIER) * ANGLE_MULTIPLIER
    interface.increaseMotorAngleReferences(motors,[angle1,-angle2])
    while not interface.motorAngleReferencesReached(motors) :
        motorAngles = interface.getMotorAngles(motors)
        time.sleep(0.1)

    print "Destination reached!"

    
        
def move(distance):
    # Get the corresponding motor rotation in radians
    _distance = distanceToRobotRadians(distance)
    interface.increaseMotorAngleReferences(motors,[(_distance),(_distance)])

    while not interface.motorAngleReferencesReached(motors) :
        motorAngles = interface.getMotorAngles(motors)
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
    

    def calculate_fwd_distance(self, x, y, theta, wall):
        (Ax, Ay, Bx, By) = wall
        m = (abs(By - Ay) * abs(Ax - x) - abs(Bx - Ax) * abs(Ay - y))/(abs(By - Ay) * cos(theta) - abs(Bx - Ax) * sin(theta))
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
        SENSOR_ACCURACY = 1
        DIST_SENSOR_CENTER = 2
        SENSOR_UPPER_BOUND = 150
        SENSOR_LOWER_BOUND = 10
        
        d = z - m + SENSOR_ACCURACY + DIST_SENSOR_CENTER
        if(z > SENSOR_UPPER_BOUND or z < SENSOR_LOWER_BOUND):
            gauss = 1
        else:
            gauss = self.gaussian_estimate(d)
        return gauss


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
        
    def runMCL(self):
        self.updateParticles()
        self.normalise()
        self.resample()
        #self.draw()

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
ROTATION_FACTOR = 3.294
def findObject(angle):
    return 0



def bumpObject(currX, currY, currAngle, particles):
    left_touched = 0
    right_touched = 0
    
    # Find angle to object with sonar
    # angle = findObject(currAngle)
    angle = find_bottle()
    print("WE FOUND THE BOTTLE AT ANGLE: " + str(angle))

    #If we do not find the bottle, break free and go to next pitstop
    if angle == -1:
	return (currX, currY, currAngle, particles)

    angle = normaliseAngle(angle)
    print("CURR ANGLE IS: " + str(currAngle) + ", PASSING NORMALISED ANGLE : " + str(angle))
    rotate(angle)
    newAngle = currAngle + angle
    
    # Drive blindly to it - need to store x's and y's
    motorAngles = interface.getMotorAngles(motors)
    orgX = motorAngles[0][0]
    orgY = motorAngles[1][0]
    
    interface.setMotorRotationSpeedReferences(motors, [6.0, 6.0])
        
    while not left_touched and not right_touched :
        left_touched = interface.getSensorValue(left_touch_port)[0]
        right_touched = interface.getSensorValue(right_touch_port)[0]
        
    #if we hit something, stop the motors and reverse.
    if left_touched or right_touched:
        interface.setMotorPwm(motors[0],0)
        interface.setMotorPwm(motors[1],0)
        move(-10)
    
    # At this point, we want to get a sense of our position - odometrically
    motorAngles = interface.getMotorAngles(motors)
    bumpX = motorAngles[0][0] - orgX
    bumpY = motorAngles[1][0] - orgY
    bump = (bumpX + bumpY) / 2
    
    #Get distance travelled by applying factor
    
    dist = bump * ROTATION_FACTOR
    
    newX = currX + dist * cos(newAngle) - 3 #error factor; tbc
    newY = currY + dist * sin(newAngle) - 3 #error factor; tbc
    
    particles.updateM(dist - 3)
    #update with angle or newAngle???
    particles.updateR(angle)
    return (newX, newY, newAngle, particles)


    
# Itinerary:
# Travel from wX1 to wX2.
# MCL with predefined angles to update - should be from PAO #TODO
# Sonar finds object.
# Drive to object.
# Travel from object to wB1.

currX = 0
currY = 0
currAngle = 0
reverse_length = 5

# ang = normaliseAngle(math.radians(360))
# rotate(math.pi * 2)
#not_normalised_rotate(math.pi / 2)

signatures = SignatureContainer(7)
(currX, currY, currAngle, particles) = bumpObject(currX, currY, currAngle, particles)
print("Coords: " + str(currX) + " " + str(currY) + " " + str(currAngle))
'''
motorAngles = interface.getMotorAngles(motors)
x = motorAngles[0][0]
y = motorAngles[1][0]
print "Current pos is our ORIGIN: " + str(x) + "    " + str(y)
move(31)
motorAngles = interface.getMotorAngles(motors)
currX = motorAngles[0][0] - x
currY = motorAngles[1][0] - y
print "CURRENT POS: " + str(currX) + "    " + str(currY)
move(-5)
motorAngles = interface.getMotorAngles(motors)
currX = motorAngles[0][0] - x
currY = motorAngles[1][0] - y
print "CURRENT POS: " + str(currX) + "    " + str(currY)



bumpWaypoints = [wA2, wB1, wB1b, wC1]
# Disclaimer: MCL has to take into consideration the sonar measurements
# from 3 places - -90, 0 and 90 degrees from current angle.

found_object = 0
for (x, y) in objectWaypoints:
    # If we're thinking about going to the second waypoint in B,
    # we check if we've already bumped the bottle in that zone
    if found_object == 1 and (x, y) == wB1b:
	continue
    found_object = 0

    # Calculate distance and angle
    distance = getDistanceToTravel(currX, currY, givenX, givenY)
    angle = (math.atan2(givenY-currY, givenX-currX)) - currAngle
    angle = normaliseAngle(angle)
    rotate(angle)
    currAngle += angle    

    # I'm against MCL after rotation - to discuss.
    # MCL after rotation
    #particles.updateR(angle)
    #particles.runMCL()
    #(currX, currY, currAngle) = particles.updateCurrentValues()
    #particles.draw()
    
    # Move to waypoint
    move(distance)
    
    # not sure if currAngle or angle? what does rotate update?
    # If we are in waypoint wB1, we need to turn to 90 degrees.
    if (x, y) == wB1:
        rotate(90 - currAngle)
	currAngle = 90

    # only done in waypoints: wA2, wB1, wB1b, wC1
     
    if (x, y) in bumpWaypoints:
     
    	# MCL after movement
    	particles.updateM(distance)
    	particles.runMCL()
    	(currX, currY, currAngle) = particles.updateCurrentValues()
    	#particle.draw()
    	# Find object continuously polls the bump sensor
    	(newX, newY, newAngle) = bumpObject(currX, currY, currAngle)
    
    	#If we found an object, we set the boolean to true; useful for zone B
    	# in case we have to take multiple measurements to find the bottle
    	if not(newX == currX and newY == currY and newAngle == currAngle):
	    found_object = 1
    	currX = newX
    	currY = newY
    	currAngle = newAngle

    
    '''
#interface.terminate()
