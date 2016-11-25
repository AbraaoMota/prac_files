#!usr/bin/env python
# By Jacek Zienkiewicz and Andrew Davison, Imperial College London, 2014
# Based on original C code by Adrien Angeli, 2009
import brickpi
import random
import os
import math

# initialise interface

interface=brickpi.Interface()
interface.initialize()

sonar_motor = [2]

interface.motorEnable(sonar_motor[0])

sonar_motor_params = interface.MotorAngleControllerParameters()

sonar_motor_params.maxRotationAcceleration = 6.0
sonar_motor_params.maxRotationSpeed = 4.0
sonar_motor_params.feedForwardGain = 400/20.0
sonar_motor_params.minPMW = 16.0
sonar_motor_params.pidParameters.minOutput = -255
sonar_motor_params.pidParameters.maxOutput = 255
sonar_motor_params.pidParameters.k_p = 450
sonar_motor_params.pidParameters.k_i = 340
k_d = 400
sonar_motor_params.pidParameters.k_d = k_d

interface.setMotorAngleControllerParameters(sonar_motor[0], sonar_motor_params)

sonarPort = 1
interface.sensorEnable(sonarPort, brickpi.SensorType.SENSOR_ULTRASONIC)


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


# Sonar should spin in small increments (to be decided depending on motor accuracy)
# Should complete a full rotation; on every measurement, increment a counter in each
# element of the signature array at index equal to the depth measured by the sonar.
def characterize_location(ls):
    MOTOR_ROTATION = math.pi
    init_motor_angle = interface.getMotorAngles(sonar_motor)
    real_angle = 0

    count = [0] * 37
    # Spin the motor
    interface.increaseMotorAngleReferences(sonar_motor, [MOTOR_ROTATION])
    while not interface.motorAngleReferencesReached(sonar_motor) :
        # Take sonar measurements
        reading = interface.getSensorValue(sonarPort)
        # Record measurements in the signature
        real_angle = interface.getMotorAngles(sonar_motor)[0][0] - init_motor_angle[0][0]
        real_degrees = int(math.degrees(real_angle))
        print("degree bucket is: " + str(real_degrees))
        print("Reading is: " + str(reading[0]))
        if (real_degrees <= 180 and reading[0] > 10):
            ls.sig[int(real_degrees/5)] += reading[0]
            count[int(real_degrees/5)] += 1

    # Unwind the motor due to the cable
    interface.increaseMotorAngleReferences(sonar_motor, [-MOTOR_ROTATION])
    while not interface.motorAngleReferencesReached(sonar_motor):
        # Take sonar measurements
        reading = interface.getSensorValue(sonarPort)
        # Record measurements in the signature
        real_angle = interface.getMotorAngles(sonar_motor)[0][0] - init_motor_angle[0][0]
        real_degrees = int(math.degrees(real_angle))
        print("degree bucket is: " + str(real_degrees))
        print("Reading is: " + str(reading[0]))
        if (real_degrees <= 180 and reading[0] > 10):
            ls.sig[int(real_degrees/5)] += reading[0]
            count[int(real_degrees/5)] += 1

    # Print the signature
    for i in range(0, len(ls.sig)):
        if(count[i] != 0):
            ls.sig[i] /= count[i]
        print("Signature at " + str(i) + " is: " + str(ls.sig[i]))



def compare_signatures(ls1, ls2):
    dist = 0
    s1 = ls1.sig
    s2 = ls2.sig
    for i in range(len(s1)):
        sig1Val = s1[i]
        sig2Val = s2[i]
        diff = sig1Val - sig2Val
        diffSquared = diff * diff
        dist += diffSquared
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
    MEASURABLE_DIFF = 20

    ls_bottle = LocationSignature()
    characterize_location(ls_bottle)
    bottle_sig = ls_bottle.sig

    # Suppose we're comparing against the first signature
    cached_sig = signatures.read(0)

    contiguous_counter = 0
    bottle_angle = -1

    for i in range(bottle_sig):
        curr = bottle_sig[i]
        cached_curr = cached_sig[i]
        if curr < ACCURATE_THRESHOLD:
            diff = cached_curr - curr
            if diff > MEASURABLE_DIFF:
                contiguous_counter += 1
                if contiguous_counter >= 3:
                    bottle_angle = i- int(contiguous_counter / 2)
            else:
                contiguous_counter = 0

    normalised_angle = bottle_angle * 5
    angle_to_turn = (normalised_angle - 90) * -1


    # answer is in degrees
    # radians variant
    # return math.radians(bottle_angle)
    return bottle_angle


# Prior to starting learning the locations, it should delete files from previous
# learning either manually or by calling signatures.delete_loc_files().
# Then, either learn a location, until all the locations are learned, or try to
# recognize one of them, if locations have already been learned.

signatures = SignatureContainer(7);
#signatures.delete_loc_files()

learn_location();
#recognize_location();
#find_bottle()

