from utils.multiRobot_plot import plotRandO
from utils.createObstacle import create_obstacle
from utils.cntrl import cmp_desiredVeclocity
import numpy as np

SIM_TIME = 5
TIMESTEP = 0.1
NUMBER_OF_TIMESTEP = int(SIM_TIME / TIMESTEP)
ROBOT_RADIUS = 0.5
VMAX = 2
VMIN = 0.2

def simulate(filename):
    obstacles = create_obstacle(SIM_TIME, NUMBER_OF_TIMESTEP)
    start = np.array([0, 1, 0, 0])
    goal = np.array([7, 8, 0, 0])
    robot_state = start
    robot_state_history = np.empty((4, NUMBER_OF_TIMESTEP))
    for i in range(NUMBER_OF_TIMESTEP):
        v_desired = cmp_desiredVeclocity(robot_state, goal, ROBOT_RADIUS, VMAX)
        control_vel = compute_velocity(robot_state, obstacles[:, i, :], v_desired)
        robot_state = update_state(robot_state, control_vel)
        robot_state_history[:4, i] = robot_state

    plotRandO(robot_state_history, obstacles, ROBOT_RADIUS, NUMBER_OF_TIMESTEP, SIM_TIME, filename)

def compute_velocity(robot, obstacles, v_desired):
    pA = robot[:2]
    vA = robot[-2:]
    number_of_obstacles = np.shape(obstacles)[1]
    Amat = np.empty((number_of_obstacles * 2, 2))
    bvec = np.empty((number_of_obstacles * 2))
    for i in range(number_of_obstacles):
        obstacle = obstacles[:, i]
        pB = obstacle[:2]
        vB = obstacle[2:]
        dispBA = pA - pB
        distBA = np.linalg.norm(dispBA)
        thetaBA = np.arctan2(dispBA[1], dispBA[0])
        if 2.2 * ROBOT_RADIUS > distBA:
            distBA = 2.2 * ROBOT_RADIUS
        phi_obs = np.arcsin(2.2 * ROBOT_RADIUS / distBA)
        phi_left = thetaBA + phi_obs
        phi_right = thetaBA - phi_obs
        translation = vB
        Atemp, btemp = create_constraints(translation, phi_left, "left")
        Amat[i * 2, :] = Atemp
        bvec[i * 2] = btemp
        Atemp, btemp = create_constraints(translation, phi_right, "right")
        Amat[i * 2 + 1, :] = Atemp
        bvec[i * 2 + 1] = btemp

    th = np.linspace(0, 2 * np.pi, 20)
    vel = np.linspace(0, VMAX, 5)
    vv, thth = np.meshgrid(vel, th)
    vx_sample = (vv * np.cos(thth)).flatten()
    vy_sample = (vv * np.sin(thth)).flatten()
    v_sample = np.stack((vx_sample, vy_sample))
    v_satisfying_cons = check_constraints(v_sample, Amat, bvec)
    size = np.shape(v_satisfying_cons)[1]
    diffs = v_satisfying_cons - ((v_desired).reshape(2, 1) @ np.ones(size).reshape(1, size))
    norm = np.linalg.norm(diffs, axis = 0)
    min_index = np.where(norm == np.amin(norm))[0][0]
    cmd_vel = (v_satisfying_cons[:, min_index])
    return cmd_vel

def check_constraints(v_sample, Amat, bvec):
    length = np.shape(bvec)[0]
    for i in range(int(length / 2)):
        v_sample = check_inside(v_sample, Amat[2 * i : 2 * i + 2, :], bvec[2 * i : 2 * i + 2])
    return v_sample

def check_inside(v, Amat, bvec):
    v_out = []
    for i in range(np.shape(v)[1]):
        if not ((Amat @ v[:, i] < bvec).all()):
            v_out.append(v[:, i])
    return np.array(v_out).T

def create_constraints(translation, angle, side):
    origin = np.array([0, 0, 1])
    point = np.array([np.cos(angle), np.sin(angle)])
    line = np.cross(origin, point)
    line = translate_line(line, translation)

    if side == 'left':
        line *= -1
    A = line[:2]
    b = -line[2]
    return A, b

def translate_line(line, translation):
    matrix = np.eye(3)
    matrix[2, :2] = -translation[:2]
    return matrix @ line

def update_state(x, v):
    new_state = np.empty((4))
    new_state[:2] = x[:2] + v * TIMESTEP
    new_state[-2:] = v
    return new_state
