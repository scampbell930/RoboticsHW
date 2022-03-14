import sim


if __name__ == '__main__':
    # Close any running sims
    sim.simxFinish(-1)

    # Connect to Coppelia client
    clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

    # Check connection
    if clientID != -1:
        print("Connected to remote API server")
    else:
        print("Not connected to remote API server")
