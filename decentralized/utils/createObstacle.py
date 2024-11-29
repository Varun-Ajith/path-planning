import numpy as np

def create_obstacle(simuTime, num_timeStep):
    v = -2 #obstacle 1
    p0 = np.array([5, 12])
    obst = create_robot(p0, v, np.pi / 2, simuTime, num_timeStep).reshape(4, num_timeStep, 1)
    obstacles = obst
    v = 2 #obstacle 2
    p0 = np.array([0, 5])
    obst = create_robot(p0, v, 0, simuTime, num_timeStep).reshape(4, num_timeStep, 1)
    obstacles = np.dstack((obstacles, obst))
    v = 2 #obstacle 3
    p0 = np.array([10, 10])
    obst = create_robot(p0, v, -np.pi * 3 / 4, simuTime, num_timeStep).reshape(4, num_timeStep, 1)
    obstacles = np.dstack((obstacles, obst))
    v = 2 #obstcle 4
    p0 = np.array([7.5, 2.5])
    obst = create_robot(p0, v, np.pi * 3 / 4, simuTime, num_timeStep).reshape(4, num_timeStep, 1)
    obstacles = np.dstack((obstacles, obst))
    return obstacles


def create_robot(p0, v, theta, simTime, numTimeStep):
    t = np.linspace(0, simTime, numTimeStep)
    theta = theta * np.ones(np.shape(t))
    vx = v * np.cos(theta)
    vy = v * np.sin(theta)
    v = np.stack([vx, vy])
    p0 = p0.reshape((2, 1))
    p = p0 + np.cumsum(v, axis = 1) * (simTime / numTimeStep)
    p = np.concatenate((p, v))
    return p 