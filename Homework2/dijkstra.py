import sim
import time
import matplotlib.pyplot as plt
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


def dijkstra(graph: nx.Graph, source: int, final: int):
    def cost(u, v):
        return graph[u][v]["weight"]

    n = graph.number_of_nodes()

    dist = {i: Inf for i in list(graph.nodes)}
    previous = {}
    dist[source] = 0

    visited = set()

    q = PriorityQueue()

    q.put((dist[source], source))

    while q.qsize() > 0:
        distance, node = q.get()
        visited.add(node)

        for neighbor in dict(graph.adjacency()).get(node):
            alt = dist[node] + cost(node, neighbor)

            if alt < dist[neighbor]:
                dist[neighbor] = alt
                previous[neighbor] = node

                if neighbor not in visited:
                    visited.add(neighbor)
                    q.put((dist[neighbor], neighbor))
                else:
                    out = q.get((dist[neighbor], neighbor))
                    q.put((dist[neighbor], neighbor))

    return dist, previous


def drive_along_path(final: int):

    err, robot = sim.simxGetObjectHandle(clientID, "robot", sim.simx_opmode_blocking)
    err, p_pos = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_streaming)
    err, euler = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_streaming)
    time.sleep(1)
    err = sim.simxSetJointTargetVelocity(clientID, l_motor, 0, sim.simx_opmode_streaming)
    err = sim.simxSetJointTargetVelocity(clientID, r_motor, 0, sim.simx_opmode_streaming)
    time.sleep(1)

    final_x = round(nodes[final - 1][1], 1)
    final_y = round(nodes[final - 1][2], 1)

    # remove start node
    path.pop(0)
    print(path)

    # Hopefully move along path
    for p in path:
        print(p)
        node_x = round(nodes[p - 1][1], 1)
        node_y = round(nodes[p - 1][2], 1)
        print(node_x, node_y)
        err, p_pos = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_oneshot)
        time.sleep(0.005)

        tick = 0

        # Drive to point
        while not (round(node_x, 2) - 0.05 <= p_pos[0] <= round(node_x, 2) + 0.05) or \
                not (round(node_y, 2) - 0.05 <= p_pos[1] <= round(node_y, 2) + 0.05):

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

                if p < len(nodes) - 1:

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

        if p < len(nodes) - 1:
            print("\n\n\n")
            print("Made it to next node")
            node_x = nodes[p][1]
            node_y = nodes[p][2]

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


def plot_path():
    print("my path: ", path)
    for i in range(len(moveable)):
        x_val = [nodes[moveable[i][1] - 1][1], nodes[moveable[i][2] - 1][1]]
        y_val = [nodes[moveable[i][1] - 1][2], nodes[moveable[i][2] - 1][2]]
        plt.plot(x_val, y_val, 'b')
    for j in range(len(walls)):
        x_val = [walls[j][0], walls[j][2]]
        y_val = [walls[j][1], walls[j][3]]
        plt.plot(x_val, y_val, 'r')

        for i in range(len(path) - 1):
            j = i + 1
            a = nodes[path[i] - 1][1]
            b = nodes[path[j] - 1][1]
            c = nodes[path[i] - 1][2]
            d = nodes[path[j] - 1][2]

            x_val = [a, b]
            y_val = [c, d]
            plt.plot(x_val, y_val, 'g', linewidth=2.5)
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

    nodes = [[1, -11.925, -10.05], [2, -11.925, -7.125], [3, -9.95, -7.1], [4, -9.975, -6.225], [5, -11.925, -6.15],
             [6, -11.975, -4.4], [7, -9.075, -7.125], [8, -9.125, -10.15], [9, -7.35, -10.15], [10, -10.9, -10.125],
             [11, -10.9, -8.15], [12, -8.17, -7.125], [13, -8.125, -8.025], [14, -7.257, -8.1], [15, -7.25, -6.15],
             [16, -8.25, -6.1], [17, -8.4, -5.125], [18, -9.95, -5.125], [19, -10, -3.35], [20, -10.9, -3.4],
             [21, -10.9, -4.025], [22, -7.125, -5.8], [23, -6.275, -5.85], [24, -6.2, -7.35], [25, -4, -7.35],
             [26, -4, -9.2], [27, -5.125, -9.2], [28, -5.125, -8.45], [29, -4.05, -6.45], [30, -4.05, -5.175],
             [31, -2.97, -5.2], [32, -2.97, -10.175], [33, -6.25, -10.175], [34, -6.25, -9.025], [35, -8.1, -9.025],
             [36, -5.075, -6.425], [37, -5.075, -3.775], [38, -6.975, -3.775], [39, -6.975, -0.7249], [40, -8, -0.7249],
             [41, -8, -1.225], [42, -8.975, -1.225], [43, -8.95, -3.725], [44, -8.95, -2.3], [45, -11.95, -2.275],
             [46, -11.95, -3.225], [47, -11.95, -1.15], [48, -11.95, 0.67501], [49, -8.95, 0.35001],
             [50, -7.95, 0.35001], [51, -7.125, 0.35001], [52, -7.075, 2.225], [53, -6.05, 2.225],
             [54, -6.05, 0.325601], [55, -2.975, 0.325601], [56, -2.975, 1.35], [57, -4.95, 1.4], [58, -6.05, -1.45],
             [59, -4.025, -1.45], [60, -4.025, -2.575], [61, -5.875, -2.575], [62, -4.025, -3.72], [63, -2.975, -3.72],
             [64, -2.975, -0.92499], [65, -8.975, 1.825], [66, -9.9, 1.8], [67, -9.9, -0.049992],
             [68, -10.9, -0.049992], [69, -10.9, 1.825], [70, -11.925, 1.825], [71, -11.925, 3.5], [72, -10.9, 2.875],
             [73, -8.925, 2.875], [74, -7.95, 2.85], [75, -8.925, 3.85], [76, -9.9249, 3.85], [77, -10.875, 3.85],
             [78, -10.875, 4.775], [79, -10.875, 7.325], [80, -9.85, 7.325], [81, -9.075, 7.325], [82, -9.075, 8.625],
             [83, -11.9, 8.625], [84, -11.9, 4.775], [85, -7.9, 3.3], [86, -6, 3.3], [87, -6, 6.3], [88, -6.975, 6.3],
             [89, -6.975, 4.475], [90, -7.95, 4.475], [91, -7.95, 5.9], [92, -8.875, 5.9], [93, -6.975, 8.7],
             [94, -8.05, 8.7], [95, -8.05, 7.3], [96, -5.925, 8.7], [97, -5.925, 7.4], [98, -4.95, 7.4],
             [99, -4.95, 8.65], [100, -3, 8.65], [101, -4.05, 7.4], [102, -4.05, 5.45], [103, -3.025, 7.4],
             [104, -3.025, 4.3], [105, -3.025, 2.45], [106, -4.025, 2.45], [107, -4.025, 3.3], [108, -5.05, 4.3],
             [109, -5.05, 6.3], [110, -5.05, 2.35]]

    walls = [[-11.475, -10.675, -11.475, -7.625], [-11.475, -7.625, -9.575, -7.625], [-9.575, -7.625, -9.575, -9.5],
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

    # This list was generated with the commented out code beneath. It took too long to generate every run so I save it
    # in a file to use over again
    moveable = [[0, 1, 2], [1, 2, 7], [2, 3, 4], [3, 3, 7], [4, 4, 7], [5, 5, 6], [6, 5, 12], [7, 7, 8], [8, 7, 12],
                [9, 7, 13], [10, 7, 14], [11, 8, 10], [12, 9, 10], [13, 12, 13], [14, 12, 14], [15, 14, 15],
                [16, 14, 22], [17, 15, 17], [18, 15, 18], [19, 15, 22], [20, 16, 17], [21, 16, 22], [22, 18, 19],
                [23, 18, 22], [24, 19, 21], [25, 20, 21], [26, 22, 23], [27, 25, 26], [28, 25, 29], [29, 25, 30],
                [30, 26, 28], [31, 26, 29], [32, 27, 28], [33, 29, 30], [34, 29, 31], [35, 31, 32], [36, 33, 34],
                [37, 36, 37], [38, 37, 43], [39, 38, 39], [40, 38, 43], [41, 39, 41], [42, 39, 42], [43, 39, 47],
                [44, 40, 41], [45, 40, 47], [46, 41, 47], [47, 41, 49], [48, 42, 47], [49, 42, 49], [50, 43, 44],
                [51, 45, 46], [52, 45, 47], [53, 45, 48], [54, 47, 48], [55, 49, 50], [56, 49, 51], [57, 51, 52],
                [58, 53, 54], [59, 53, 58], [60, 54, 58], [61, 55, 56], [62, 59, 60], [63, 59, 62],
                [65, 60, 62], [67, 62, 63], [68, 65, 66], [69, 67, 68], [70, 67, 70], [71, 68, 72],
                [72, 69, 70], [73, 69, 72], [74, 72, 73], [75, 72, 74], [76, 74, 75], [77, 74, 76], [78, 76, 77],
                [79, 76, 78], [80, 76, 80], [81, 76, 82], [82, 76, 84], [83, 77, 84], [84, 78, 79], [85, 78, 84],
                [86, 79, 81], [87, 80, 81], [88, 82, 83], [89, 85, 86], [90, 86, 93], [91, 87, 88], [92, 87, 90],
                [93, 87, 91], [94, 87, 92], [95, 89, 90], [96, 89, 92], [97, 89, 93], [98, 91, 92], [99, 93, 94],
                [100, 93, 96], [101, 93, 98], [102, 94, 96], [103, 96, 97], [104, 97, 99], [105, 97, 101],
                [106, 97, 103], [107, 98, 99], [108, 98, 101], [109, 98, 103], [110, 99, 101], [111, 101, 102],
                [112, 101, 103], [113, 103, 104], [114, 103, 105], [115, 104, 108], [116, 105, 106], [117, 108, 109],
                [118, 108, 110], [119, 54, 55], [120, 58, 59], [121, 32, 33], [122, 24, 25], [123, 23, 24],
                [124, 29, 36], [125, 63, 64], [126, 60, 61], [127, 44, 45], [128, 52, 53], [129, 50, 74],
                [130, 74, 85], [131, 49, 65], [132, 66, 67], [133, 37, 38], [134, 2, 3]]

    graph = construct_graph(moveable, nodes)

    # Used for checking is node paths intersect walls
    # was too slow to run everytime, so I wrote the total to a file and
    # hardcoded those values into the program
    '''
    travel_edges = []
    # Calculate non-intersecting walls and edges
    for i in range(len(nodes)):
        print("\n\n", nodes[i][0], "\n\n")
        j = i - 1
        while j < len(nodes) - 1 and j < (i + 9):
            j += 1
            if i != j:
                intersect = 0

                # Check if nodes intersect any walls
                for k in range(len(walls)):
                    p1, p2 = Point(nodes[i][1], nodes[i][2]), Point(nodes[j][1], nodes[j][2]),
                    p3, p4 = Point(walls[k][0], walls[k][1]), Point(walls[k][2], walls[k][3])
                    seg1 = Segment(p1, p2)
                    seg2 = Segment(p3, p4)

                    intersection = seg1.intersection(seg2)

                    if intersection:
                        intersect = 1
                        break

                if intersect == 0:
                    print("Nodes with no intersection: ", i + 1, j + 1)
                else:
                    print("Intersection")

                for l in range(len(travel_edges)):
                    if (travel_edges[l][1] == i and travel_edges[l][2] == j) or (
                            travel_edges[l][1] == j and travel_edges[l][2] == i):
                        intersect = 1
                        break

                if intersect == 0:
                    travel_edges.append([len(travel_edges), i + 1, j + 1])

    with open('edges.txt', 'w') as f:
        f.write(str(travel_edges))
    '''
    final = 24
    dist, previous = dijkstra(graph, 1, final)
    path = find_path(previous, 1, final)

    plot_path()

    drive_along_path(final)

    sim.simxFinish(clientID)
