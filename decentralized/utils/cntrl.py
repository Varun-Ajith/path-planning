import numpy as np

def cmp_desiredVeclocity(current_pos, goal_pos, robot_radius, vmax):
    disp_velocity = (goal_pos - current_pos)[:2]
    norm = np.linalg.norm(disp_velocity)
    if norm < robot_radius / 5:
        return np.zeros(2)
    disp_velocity = disp_velocity / norm
    np.shape(disp_velocity)
    desiredVelocity = vmax * disp_velocity
    return desiredVelocity