from utils.multiRobot_plot import plotRandO
from utils.createObstacle import create_obstacle
import numpy as np
from scipy.optimize import minimize, Bounds
import time


# PARAMETRES
SIM_TIME = 8
TIMESTEP = 0.1
NUMBER_OF_TIMESTEPS = int(SIM_TIME/TIMESTEP)
ROBOT_RADIUS = 0.5
V_MAX = 2
VMIN = 0.2
Q_c = 5
kappa = 4
HRZN_l = int(4)
NMPC_TIMESTEP = 0.3
upper_bound = [(1/np.sqrt(2)) * V_MAX] * HRZN_l * 2
lower_bound = [-(1/np.sqrt(2)) * V_MAX] * HRZN_l * 2


def simulate(filename):
    obstacles = create_obstacle(SIM_TIME, NUMBER_OF_TIMESTEPS)

    start = np.array([7, 6])
    p_desired = np.array([7, 6])

    robot_state = start
    robot_state_history = np.empty((4, NUMBER_OF_TIMESTEPS))

    for i in range(NUMBER_OF_TIMESTEPS):
        obstacle_predictions = predict_obstacle_positions(obstacles[:, i, :]) # # predict the obstacles' position in future
        xref = compute_xref(robot_state, p_desired,
                            HRZN_l, NMPC_TIMESTEP)
        vel, velocity_profile = compute_velocity(
            robot_state, obstacle_predictions, xref) # # compute velocity using nmpc
        robot_state = update_state(robot_state, vel, TIMESTEP)
        robot_state_history[:2, i] = robot_state

    plotRandO(
        robot_state_history, obstacles, ROBOT_RADIUS, NUMBER_OF_TIMESTEPS, SIM_TIME, filename)


def compute_velocity(robot_state, obstacle_predictions, xref): #Computes control velocity of the copter
    u0 = np.random.rand(2*HRZN_l)
    def cost_fn(u): return total_cost(
        u, robot_state, obstacle_predictions, xref)

    bounds = Bounds(lower_bound, upper_bound)

    res = minimize(cost_fn, u0, method='SLSQP', bounds=bounds)
    velocity = res.x[:2]
    return velocity, res.x


def compute_xref(start, goal, number_of_steps, timestep):
    dir_vec = (goal - start)
    norm = np.linalg.norm(dir_vec)
    if norm < 0.1:
        new_goal = start
    else:
        dir_vec = dir_vec / norm
        new_goal = start + dir_vec * V_MAX * timestep * number_of_steps
    return np.linspace(start, new_goal, number_of_steps).reshape((2*number_of_steps))


def total_cost(u, robot_state, obstacle_predictions, xref):
    x_robot = update_state(robot_state, u, NMPC_TIMESTEP)
    c1 = tracking_cost(x_robot, xref)
    c2 = total_collision_cost(x_robot, obstacle_predictions)
    total = c1 + c2
    return total


def tracking_cost(x, xref):
    return np.linalg.norm(x-xref)


def total_collision_cost(robot, obstacles):
    total_cost = 0
    for i in range(HRZN_l):
        for j in range(len(obstacles)):
            obstacle = obstacles[j]
            rob = robot[2 * i: 2 * i + 2]
            obs = obstacle[2 * i: 2 * i + 2]
            total_cost += collision_cost(rob, obs)
    return total_cost


def collision_cost(x0, x1): #Cost of collision between two robot_state
    d = np.linalg.norm(x0 - x1)
    cost = Q_c / (1 + np.exp(kappa * (d - 2*ROBOT_RADIUS)))
    return cost


def predict_obstacle_positions(obstacles):
    obstacle_predictions = []
    for i in range(np.shape(obstacles)[1]):
        obstacle = obstacles[:, i]
        obstacle_position = obstacle[:2]
        obstacle_vel = obstacle[2:]
        u = np.vstack([np.eye(2)] * HRZN_l) @ obstacle_vel
        obstacle_prediction = update_state(obstacle_position, u, NMPC_TIMESTEP)
        obstacle_predictions.append(obstacle_prediction)
    return obstacle_predictions


def update_state(x0, u, timestep):
    N = int(len(u) / 2)
    lower_triangular_ones_matrix = np.tril(np.ones((N, N)))
    kron = np.kron(lower_triangular_ones_matrix, np.eye(2))

    new_state = np.vstack([np.eye(2)] * int(N)) @ x0 + kron @ u * timestep

    return new_state