import sim
import time
import numpy as np
from shapely.geometry import Point
from shapely.geometry import Polygon
import random as r


def read_sensors(sensors: list) -> list:
    readings = []

    for sens in sensors:
        read = sim.simxReadProximitySensor(clientID, sens, sim.simx_opmode_blocking)
        print(read[2][0], read[2][1])
        readings.append(read)

    return readings


def pso(readings: list) -> list:
    particles = []
    pbest = []
    gbest = [-1000, -1000]
    velocity = []
    prev_velocity = []

    poly = []
    # Construct polygon for final point range
    for read in readings:
        if read[1]:
            poly.append([read[2][0], read[2][1]])

    # initialize particles
    for i in range(5):
        print()
        # point = Point( random coords from robot position )
        # while point not in polygon
            # generate new point
        # particles.append(point)
        # pbest.append(particles[i])

        # if fitness(particles[i]) < fitness(gbest):
            # gbest = particles[i]

        # velocity.append(random coordinates for vector (inside polygon still))

    c = 1
    s = 1
    w = 1

    # update particles
    for _ in range(len(readings)):
        for p in range(len(particles)):
            for k in p:
                print()
                # velocity[p][k] = w * velocity[p][k] + c * rand * (pbest[p][k] - particles[p][k]) + s * rand *
                #                   (gbest[k] * particles[p][k])

            # particles[p] = particles[p] + velocity[p]

            # if fitness(particles[p]) < fitness(pbest[p]):
                # pbest[p] = particles[p]
                # if fitness(particles[p]) < fitness(gbest):
                    # gbest = particles[p]

    return gbest


if __name__ == '__main__':
    # Close any running sims
    sim.simxFinish(-1)

    # Connect to Coppelia client
    clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
    print(clientID)

    # Check connection
    if clientID != -1:
        print("Connected to remote API server")
    else:
        print("Not connected to remote API server")
        quit()

    # Robot Handles
    err, robot = sim.simxGetObjectHandle(clientID, "robot", sim.simx_opmode_blocking)
    err, l_motor = sim.simxGetObjectHandle(clientID, "l_motor", sim.simx_opmode_blocking)
    err, r_motor = sim.simxGetObjectHandle(clientID, "r_motor", sim.simx_opmode_blocking)

    err = sim.simxSetJointTargetVelocity(clientID, l_motor, 0, sim.simx_opmode_streaming)
    err = sim.simxSetJointTargetVelocity(clientID, r_motor, 0, sim.simx_opmode_streaming)
    time.sleep(1)

    # Set up sensor handles
    err, sens0 = sim.simxGetObjectHandle(clientID, "sens0", sim.simx_opmode_blocking)
    print(err)
    err, sens1 = sim.simxGetObjectHandle(clientID, "sens1", sim.simx_opmode_blocking)
    print(err)
    err, sens2 = sim.simxGetObjectHandle(clientID, "sens2", sim.simx_opmode_blocking)
    print(err)
    err, sens3 = sim.simxGetObjectHandle(clientID, "sens3", sim.simx_opmode_blocking)
    print(err)
    err, sens4 = sim.simxGetObjectHandle(clientID, "sens4", sim.simx_opmode_blocking)
    print(err)
    err, sens5 = sim.simxGetObjectHandle(clientID, "sens5", sim.simx_opmode_blocking)
    print(err)
    err, sens6 = sim.simxGetObjectHandle(clientID, "sens6", sim.simx_opmode_blocking)
    print(err)
    err, sens7 = sim.simxGetObjectHandle(clientID, "sens7", sim.simx_opmode_blocking)
    print(err)
    err, sens8 = sim.simxGetObjectHandle(clientID, "sens8", sim.simx_opmode_blocking)
    print(err)
    err, sens9 = sim.simxGetObjectHandle(clientID, "sens9", sim.simx_opmode_blocking)
    print(err)
    err, sens10 = sim.simxGetObjectHandle(clientID, "sens10", sim.simx_opmode_blocking)
    print(err)
    err, sens11 = sim.simxGetObjectHandle(clientID, "sens11", sim.simx_opmode_blocking)
    print(err)
    err, sens12 = sim.simxGetObjectHandle(clientID, "sens12", sim.simx_opmode_blocking)
    print(err)
    err, sens13 = sim.simxGetObjectHandle(clientID, "sens13", sim.simx_opmode_blocking)
    print(err)
    err, sens14 = sim.simxGetObjectHandle(clientID, "sens14", sim.simx_opmode_blocking)
    print(err)
    err, sens15 = sim.simxGetObjectHandle(clientID, "sens15", sim.simx_opmode_blocking)
    print(err)

    sensors = [sens0, sens1, sens2, sens3, sens4, sens5, sens6, sens7, sens8, sens9, sens10, sens11, sens12,
               sens13, sens14, sens15]

    sensors = [sens4]

    for x in range(1000):
        # Update sensors
        readings = read_sensors(sensors)
        pso(readings)
