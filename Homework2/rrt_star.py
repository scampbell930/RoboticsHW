from sympy import Point, Segment
import sim
import time
import matplotlib.pyplot as plt
import random as r
from numpy import Inf
from queue import PriorityQueue
import math
import networkx as nx
import numpy as np
from scipy.spatial.transform import Rotation as R


def find_path(prev, start, end):
    node = end
    path = []
    while node != start:
        path.append(node)
        node = prev[node]
    path.append(node)
    path.reverse()
    return path


def construct_graph(moveable: list, nodes: list) -> nx.Graph:
    graph = nx.Graph()

    for i in range(len(moveable)):
        p1 = [nodes[moveable[i][1] - 1][1], nodes[moveable[i][2] - 1][1]]
        p2 = [nodes[moveable[i][1] - 1][2], nodes[moveable[i][2] - 1][2]]
        distance = math.sqrt(((p1[0] - p2[0]) ** 2) + ((p1[1] - p2[1]) ** 2))
        graph.add_edge(moveable[i][1], moveable[i][2], weight=distance)

    return graph


def drive_along_path():

    err, robot = sim.simxGetObjectHandle(clientID, "robot", sim.simx_opmode_blocking)
    err, p_pos = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_streaming)
    err, euler = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_streaming)
    time.sleep(1)
    err = sim.simxSetJointTargetVelocity(clientID, l_motor, 0, sim.simx_opmode_streaming)
    err = sim.simxSetJointTargetVelocity(clientID, r_motor, 0, sim.simx_opmode_streaming)
    time.sleep(1)

    # remove start node
    path.pop(0)
    print(path)

    # Hopefully move along path
    for p in path:
        print(p)
        node_x = round(graph.nodes[p]['position'][0], 1)
        node_y = round(graph.nodes[p]['position'][1], 1)
        print(node_x, node_y)
        err, p_pos = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_oneshot)
        time.sleep(0.005)

        tick = 0

        # Drive to point
        while not (round(node_x, 2) - 0.1 <= p_pos[0] <= round(node_x, 2) + 0.1) or \
                not (round(node_y, 2) - 0.1 <= p_pos[1] <= round(node_y, 2) + 0.1):

            # Update position
            time.sleep(0.005)
            err, p_pos = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_oneshot)
            rob_x = p_pos[0]
            rob_y = p_pos[1]

            err = sim.simxSetJointTargetVelocity(clientID, l_motor, 1.5, sim.simx_opmode_streaming)
            err = sim.simxSetJointTargetVelocity(clientID, r_motor, 1.5, sim.simx_opmode_streaming)
            time.sleep(0.005)

            tick += 1

            # Every x iterations, recheck position
            if tick % 100 == 0:
                err, euler = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_buffer)
                rob_deg = R.from_rotvec(euler)
                final = [node_x - p_pos[0], node_y - p_pos[1]]
                final_unit = final / np.linalg.norm(final)
                new_angle = R.from_rotvec([final_unit[0], final_unit[1], 0])
                new_angle = new_angle.as_euler('zyx', degrees=True)
                euler = rob_deg.as_euler('zxy', degrees=True)
                heading = [math.cos(euler[0]), math.sin(euler[0])]
                heading = heading / np.linalg.norm(heading)
                diff = np.arccos(np.dot(heading, final_unit))
                calc = math.atan2(final[1], final[0])
                new_angle = euler + new_angle
                err, p_pos = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_oneshot)

                time.sleep(0.005)
                err = sim.simxSetJointTargetVelocity(clientID, l_motor, 0, sim.simx_opmode_streaming)
                err = sim.simxSetJointTargetVelocity(clientID, r_motor, 0, sim.simx_opmode_streaming)

                time.sleep(0.005)

                if p < len(graph.nodes) - 1:

                    first = True
                    # Rotate to next point
                    while not (math.degrees(calc) - 0.5 <= round(euler[0], 2) <= math.degrees(calc) + 0.5):
                        if first:
                            first = False
                            if euler[0] > math.degrees(calc):
                                err = sim.simxSetJointTargetVelocity(clientID, l_motor, 0.3, sim.simx_opmode_streaming)
                                err = sim.simxSetJointTargetVelocity(clientID, r_motor, -0.3, sim.simx_opmode_streaming)
                            else:
                                err = sim.simxSetJointTargetVelocity(clientID, l_motor, -0.3, sim.simx_opmode_streaming)
                                err = sim.simxSetJointTargetVelocity(clientID, r_motor, 0.3, sim.simx_opmode_streaming)

                        time.sleep(0.005)
                        err, euler = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_buffer)
                        rob_deg = R.from_rotvec(euler)
                        euler = rob_deg.as_euler('zxy', degrees=True)

                    err = sim.simxSetJointTargetVelocity(clientID, l_motor, 0, sim.simx_opmode_streaming)
                    err = sim.simxSetJointTargetVelocity(clientID, r_motor, 0, sim.simx_opmode_streaming)
                    print("Found: ", euler[0])
                    time.sleep(0.1)

        err, euler = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_buffer)
        rob_deg = R.from_rotvec(euler)
        final = [node_x - p_pos[0], node_y - p_pos[1]]
        final_unit = final / np.linalg.norm(final)
        new_angle = R.from_rotvec([final_unit[0], final_unit[1], 0])
        new_angle = new_angle.as_euler('zyx', degrees=True)
        euler = rob_deg.as_euler('zxy', degrees=True)
        new_angle = euler + new_angle
        err, p_pos = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_oneshot)

        time.sleep(0.005)
        err = sim.simxSetJointTargetVelocity(clientID, l_motor, 0, sim.simx_opmode_streaming)
        err = sim.simxSetJointTargetVelocity(clientID, r_motor, 0, sim.simx_opmode_streaming)

        time.sleep(0.005)

        if p < len(graph.nodes) - 1:
            print("\n\n\n")
            print("Made it to next node")
            node_x = graph.nodes[p+1]['position'][0]
            node_y = graph.nodes[p+1]['position'][1]

            # Calculate angle to next point
            new_angle = math.degrees(math.atan(math.dist([p_pos[1]], [node_y]) / math.dist([p_pos[0]], [node_x])))

            final = [node_x - p_pos[0], node_y - p_pos[1]]
            final_unit = final / np.linalg.norm(final)

            comp = R.from_rotvec(([final_unit[0], final_unit[1], 0]))
            comp = comp.as_euler('zyx', degrees=True)
            calc = math.atan2(final[1], final[0])

            new_angle = comp[0]

            first = True

            # Rotate to next point
            while not (math.degrees(calc) - 0.5 <= round(euler[0], 2) <= round(math.degrees(calc), 2) + 0.5):
                if first:
                    first = False
                    if euler[0] > math.degrees(calc):
                        err = sim.simxSetJointTargetVelocity(clientID, l_motor, 0.3, sim.simx_opmode_streaming)
                        err = sim.simxSetJointTargetVelocity(clientID, r_motor, -0.3, sim.simx_opmode_streaming)
                    else:
                        err = sim.simxSetJointTargetVelocity(clientID, l_motor, -0.3, sim.simx_opmode_streaming)
                        err = sim.simxSetJointTargetVelocity(clientID, r_motor, 0.3, sim.simx_opmode_streaming)
                time.sleep(0.005)
                err, euler = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_buffer)
                rob_deg = R.from_rotvec(euler)
                euler = rob_deg.as_euler('zxy', degrees=True)
                print(euler[0])

            err = sim.simxSetJointTargetVelocity(clientID, l_motor, 0, sim.simx_opmode_streaming)
            err = sim.simxSetJointTargetVelocity(clientID, r_motor, 0, sim.simx_opmode_streaming)

            time.sleep(0.005)


def rrt_star(iterations: int, end: list, explore: float) -> list:

    i = 0
    graph = nx.Graph()
    graph.add_node(i, position=start)

    for x in range(iterations):
        print("Iteration ", x)
        new = [r.uniform(-12, -2), r.uniform(-10, 9)]
        nearest = find_nearest(graph, new)

        difference = [new[0] - graph.nodes[nearest]['position'][0], new[1] - graph.nodes[nearest]['position'][1]]
        unit_diff = difference / np.linalg.norm(difference)
        new_node = unit_diff * explore
        new_node = [new_node[0] + graph.nodes[nearest]['position'][0], new_node[1] +
                                                                            graph.nodes[nearest]['position'][1]]

        # Make sure new position is not too close to a wall
        too_close = False
        for wall in walls:
            if wall[0] - 0.2 <= new_node[0] <= wall[0] + 0.2:
                if wall[1] - 0.2 <= new_node[0] <= wall[3] + 0.2:
                    too_close = True
                    break

        if not too_close:

            if new_node[0] - 0.5 <= end[0] <= new_node[0] + 0.5:
                if new_node[1] - 0.5 <= end[1] <= new_node[1] + 0.5:
                    i += 1
                    graph.add_node(i, position=end)
                    graph.add_edge(nearest, i)
                    return [True, graph]


            # Check if nearest and new node intersect any walls
            if not intersect_walls(graph, new_node, nearest):
                i += 1
                graph.add_node(i, position=new_node)
                graph.add_edge(nearest, i)

    return [False, graph]


def intersect_walls(graph, new_node, near_node):
    intersect = 0

    # Check if nodes intersect any walls
    for k in range(len(walls)):
        p1, p2 = Point(new_node[0], new_node[1]), Point(graph.nodes[near_node]['position'][0],
                                                        graph.nodes[near_node]['position'][1])
        p3, p4 = Point(walls[k][0], walls[k][1]), Point(walls[k][2], walls[k][3])
        seg1 = Segment(p1, p2)
        seg2 = Segment(p3, p4)

        intersection = seg1.intersection(seg2)

        if intersection:
            intersect = 1
            break

    return intersect == 1


def find_nearest(graph, new):
    min = 0
    min_dist = 100000000
    # find nearest node
    for n in graph.nodes:
        distance = math.sqrt(((new[0] - graph.nodes[n]['position'][0]) ** 2) + ((new[1] - graph.nodes[n]['position'][1]) ** 2))

        if distance < min_dist:
            min_dist = distance
            min = n

    return min


def plot_path():
    print("my path: ", path)
    for j in range(len(walls)):
        x_val = [walls[j][0], walls[j][2]]
        y_val = [walls[j][1], walls[j][3]]
        plt.plot(x_val, y_val, 'r')

    for i in range(len(path)):
        plt.scatter(graph.nodes[i]['position'][0], graph.nodes[i]['position'][1])
    plt.show()


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

    # Make sure robot is not moving
    err, l_motor = sim.simxGetObjectHandle(clientID, "l_motor", sim.simx_opmode_blocking)
    err, r_motor = sim.simxGetObjectHandle(clientID, "r_motor", sim.simx_opmode_blocking)

    err = sim.simxSetJointTargetVelocity(clientID, l_motor, 0, sim.simx_opmode_streaming)
    err = sim.simxSetJointTargetVelocity(clientID, r_motor, 0, sim.simx_opmode_streaming)
    time.sleep(1)

    start = [-11.925, -10.05]

    walls = [[-11.6, -10.675, -11.6, -7.625], [-11.6, -7.625, -9.575, -7.625], [-9.575, -7.625, -9.575, -9.5],
             [-9.575, -9.5, -10.475, -9.5], [-10.475, -9.5, -10.475, -8.575], [-12.4, -6.55, -10.4, -6.55],
             [-9.6, -6.55, -7.7, -6.55], [-7.7, -6.55, -7.7, -7.55], [-8.7, -7.55, -8.7, -9.475],
             [-8.7, -9.475, -6.8249, -9.475], [-6.8249, -9.475, -6.8249, -10.625], [-8.6999, -8.6, -5.7749, -8.6],
             [-6.7999, -8.6, -6.7999, -6.25], [-4.5499, -8.825, -4.5499, -7.8], [-4.5499, -7.8, -5.8499, -7.8],
             [-5.8499, -7.8, -5.8499, -9.675], [-5.8499, -9.675, -3.449, -9.65], [-3.449, -9.65, -3.449, -5.775],
             [-4.5249, -6.95, -5.7499, -6.95], [-5.7499, -6.95, -5.7499, -4.275], [-5.7499, -4.275, -9.4499, -4.275],
             [-9.5249, -4.65, -9.5249, -2.8], [-9.5249, -2.8, -11.55, -2.8], [-11.55, -2.8, -11.55, -5.65],
             [-11.55, -5.65, -8.899, -5.65], [-8.899, -5.65, -8.899, -6.575], [-12.45, -3.775, -11.475, -3.775],
             [-11.525, -4.625, -10.35, -4.625], [-10.35, -4.625, -10.35, -3.75], [-5.7249, -5.25, -7.7249, -5.25],
             [-7.7249, -5.25, -7.7249, -5.65], [-8.399, -3.175, -7.5249, -3.175], [-7.5249, -3.175, -7.5249, -1.225],
             [-11.45, -1.75, -7.4749, -1.75], [-2.3749, -4.475, -4.5749, -4.475], [-4.5749, -5.975, -4.5749, -3.175],
             [-4.5749, -3.175, -6.5999, -3.175], [-6.5999, -3.175, -6.5999, 1.8], [-6.5999, -0.2, -8.4749, -0.2],
             [-8.4749, -0.2, -8.4749, -0.2], [-6.5999, -1.975, -4.5249, -1.975], [-3.4999, -3, -3.4999, -0.22499],
             [-4.4999, -0.74999, -4.4999, -0.1999], [-5.4499, -0.9749, -5.4999, -0.1999],
             [-5.4999, -0.1999, -2.399, -0.1999], [-12.45, 1.2, -11.325, 1.2], [-11.325, 1.2, -11.325, -0.72499],
             [-11.325, -0.72499, -9.4499, -0.72499], [-9.4499, -0.72499, -9.4499, 1.175],
             [-10.375, 0.425, -10.375, 2.275], [-10.375, 2.275, -8.499, 2.275], [-8.499, 2.275, -8.499, 0.825],
             [-12.475, 4.175, -11.425, 4.175], [-11.425, 4.175, -11.425, 2.3], [-11.425, 3.35, -9.375, 3.35],
             [-11.425, 5.257, -11.425, 8.05], [-11.425, 8.05, -9.5, 8.05], [-10.4, 4.45, -10.4, 6.85],
             [-8.6, 9.25, -8.6, 6.4], [-8.6, 6.4, -9.45, 6.4], [-9.45, 6.4, -9.45, 4.425],
             [-9.45, 4.425, -8.475, 4.425], [-8.475, 5.3, -8.475, 3.4], [-8.475, 3.9, -6.475, 3.9],
             [-6.475, 3.9, -6.475, 5.825], [-7.525, 4.975, -7.525, 5.9], [-8.575, 6.85, -7.55, 6.85],
             [-7.55, 6.85, -7.55, 8.2], [-7.6, 0.825, -7.6, 2.725], [-7.6, 2.725, -5.525, 2.725],
             [-3.525, 0.775, -5.5, 0.775], [-5.5, 0.775, -5.5, 6.875], [-2.4, 1.875, -5.5, 1.875],
             [-4.65, 1.875, -4.65, 3.8], [-4.65, 3.8, -3.625, 3.8], [-3.625, 3.8, -3.625, 2.925],
             [-3.65, 6.875, -3.65, 4.95], [-3.65, 4.95, -4.675, 4.95], [-4.675, 4.95, -4.675, 6.85],
             [-4.675, 6.85, -6.4, 6.85], [-6.4, 6.85, -6.4, 8.225], [-5.5, 7.95, -5.5, 9.275],
             [-4.425, 8.1, -2.425, 8.1]]
    nodes = []
    moveable = []

    [outcome, graph] = rrt_star(30, [-9.95, -7.1], 1.5)

    print("\n\n\n\n\n\n")
    print(outcome, graph)

    path = graph.nodes
    plot_path()

    if outcome:
        print(path)
        path = nx.shortest_path(graph, source=0, target=len(graph.nodes) - 1)
        drive_along_path()
