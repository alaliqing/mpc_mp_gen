import casadi
import pickle
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Polygon, PathPatch
from matplotlib.collections import PatchCollection
import matplotlib.path as mpath
import numpy as np
import sys
import os
import time
import math

from scipy.spatial.transform import Rotation as Rotation
from sympy import *
from casadi import *

NX = 4  # x = x, y, yaw, yawt
NU = 2  # u = [v, w]
T = 10  # horizon length

R = np.diag([5, 0.01])  # input cost matrix
Rd = np.diag([0.1, 0.1])  # ([0.01, 1.0])  # input difference cost matrix
Q = np.diag([0.1, 0, 0, 0]) # final matrix
Qa = np.diag([1, 1, 1, 1])
Qb = 1
GOAL_DIS = 1.1  # goal distance
STOP_SPEED = 0.5 / 3.6  # stop speed
MAX_TIME = 100.0  # max simulation time

# iterative paramter
MAX_ITER = 100 # Max iteration
DU_TH = 0.3  # 0.1  # iteration finish param

DT = 0.2  # [s] time tick 0.1 for 45degree

# Vehicle parameters
LENGTH = 1.12 # 0.72   # [m]
LENGTH_T = 1.14 #0.36  # [m]
WIDTH = 0.745 # 0.48  # [m]
WIDTH_T = 1.18
BACKTOWHEEL = 0.36  # [m]
WHEEL_LEN = 0.1  # [m]
WHEEL_WIDTH = 0.07  # [m]
TREAD = 0.2  # [m]
WB = 0.3  # [m]
ROD_LEN = 1.55 #0.5 # [m]
CP_OFFSET = 0 # 0.1   # [m]

RES = 0.03

MAX_OMEGA = 2 # maximum rotation speed [rad/s]

MAX_SPEED = 2.5  # 55.0 / 3.6  # maximum speed [m/s]
MIN_SPEED = 0.0  # minimum speed [m/s]
JACKKNIFE_CON = 45.0  # [degrees]

CIR_RAD = 5  # radius of circular path [m]

def get_nparray_from_matrix(x):
    return np.array(x).flatten()

def predict_motion_mpc(x, u):
    """
    Predict motion for forward model.
    """
    x1 = x
    x1[0] = x[0] + u[0] * cos(x[2]) * DT
    x1[1] = x[1] + u[0] * sin(x[2]) * DT
    x1[2] = x[2] + u[1] * DT
    x1[3] = x[3] + u[0] / ROD_LEN * sin(x[2] - x[3]) * DT - CP_OFFSET * u[1] * cos(x[2] - x[3]) / ROD_LEN * DT
    return x1

def predict_motion_mpc_1(x, u):
    """"
    Predict motion for backward model
    """
    x1 = x
    x1[0] = x[0] + u[0] * cos(x[2] - x[3]) * cos(x[3]) * DT
    x1[1] = x[1] + u[0] * cos(x[2] - x[3]) * sin(x[3]) * DT
    x1[2] = x[2] + u[1] * DT
    x1[3] = x[3] + u[0] / ROD_LEN * sin(x[2] - x[3]) * DT - CP_OFFSET * u[1] * cos(x[2] - x[3]) / ROD_LEN * DT
    return x1

def interpose(xls, yls, yawls, yawtls, lengthls):
    """
    Interpolate into 10 states based on path length.
    """
    arc = lengthls[-1] / 9
    if lengthls[-1] < 0:
        lengthls.reverse()
        xls.reverse()
        yls.reverse()
        yawls.reverse()
        yawtls.reverse()
    xlsn_ = []
    ylsn_ = []
    yawlsn_ = []
    yawtlsn_ = []
    for i in range(10):
        xlsn_.append(np.interp(arc*i, lengthls, xls))
        ylsn_.append(np.interp(arc*i, lengthls, yls))
        yawlsn_.append(np.interp(arc*i, lengthls, yawls))
        yawtlsn_.append(np.interp(arc*i, lengthls, yawtls))
    # for i in range(10):
        # ylsn_.append(np.interp(xlsn_[i], xls, yls))
        # yawlsn_.append(np.interp(xlsn_[i], xls, yawls))
        # yawtlsn_.append(np.interp(xlsn_[i], xls, yawtls))
    if lengthls[-1] < 0:
        xlsn_.reverse()
        ylsn_.reverse()
        yawlsn_.reverse()
        yawtlsn_.reverse()
    return xlsn_, ylsn_, yawlsn_, yawtlsn_

def rectangle_vertex(x, y, a, b, yaw1, color):
    rect = Rectangle((x, y), a, b, angle=yaw1)
    rect.set_edgecolor(color)
    rect.set_facecolor('None')
    return rect

def draw_rotated_box(x, y, yaw1, yawt, color, width=0.04, height=0.02):
    """
    Plot tractor and trailer.
    """
    margin = 20
    import numpy as np
    a = LENGTH # length
    b = WIDTH # width
    c = a/2 # length2

    yaw1rad = np.deg2rad(yaw1)
    yawtrad = np.deg2rad(yawt)

    plot_car(x, y, yaw1rad, LENGTH, WIDTH)

    # for tractor
    plot_car(x - np.cos(yawtrad) * c - np.cos(yaw1rad) * c,
                y - np.sin(yawtrad) * c - np.sin(yaw1rad) * c, yawtrad, LENGTH_T, WIDTH_T, truckcolor="-r")

    # x1 = x - c * cos(yaw1rad) - c * cos(yawtrad)
    # y1 = y - c * sin(yaw1rad) - c * sin(yawtrad)

    # # x = x - a/2 * cos(yaw1) - b/2 * sin(yaw1)
    # # y = y - a/2 * sin(yaw1) + b/2 * cos(yaw1)

    # # x1 = x1 - c/2 * cos(yawt) - b/2 * sin(yawt)
    # # y1 = y1 - c/2 * sin(yawt) + b/2 * cos(yawt)
    # print(yaw1)

    # r = Rotation.from_euler('z', yaw1rad)
    # print(r)


    # pos2 = r.apply([x-a/2, y-b/2, 0])
    # print(yaw1, yaw1rad, "x", x, pos2[0],x-a*cos(yaw1rad)/2-b*sin(yaw1rad)/2, "y", y, pos2[1], y+a*sin(yaw1rad)/2-b*cos(yaw1rad)/2)

    #rect1 = rectangle_vertex(x-a*cos(yaw1rad)/2-b*sin(yaw1rad)/2, y+a*sin(yaw1rad)/2-b*cos(yaw1rad)/2, a, b, yaw1, 'blue') #x-a*cos(yaw1)/2-b*sin(yaw1)/2, y+a*sin(yaw1)/2-b*cos(yaw1)/2
    #rect2 = rectangle_vertex(x1, y1, c, b, yawt, 'green')


    ax = plt.gca()
    # ax.add_patch(rect1)
    # ax.add_patch(rect2)
    # ax.plot(xls1, yls1, 'g-')
    # ax.plot(xls2, yls2, 'b-')

def plot_car(x, y, yaw, length, width, steer=0.0, cabcolor="-r", truckcolor="-k"):  # pragma: no cover

    outline = np.array([[-length / 2, (length - length / 2), (length - length / 2), -length / 2, -length / 2],
                        [width / 2, width / 2, - width / 2, -width / 2, width / 2]])

    rr_wheel = np.array([[WHEEL_LEN, -WHEEL_LEN, -WHEEL_LEN, WHEEL_LEN, WHEEL_LEN],
                         [-WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD,
                          -WHEEL_WIDTH - TREAD]])

    rl_wheel = np.copy(rr_wheel)
    rl_wheel[1, :] *= -1

    Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                     [-math.sin(yaw), math.cos(yaw)]])

    outline = (outline.T.dot(Rot1)).T
    rr_wheel = (rr_wheel.T.dot(Rot1)).T
    rl_wheel = (rl_wheel.T.dot(Rot1)).T

    outline[0, :] += x
    outline[1, :] += y

    rr_wheel[0, :] += x
    rr_wheel[1, :] += y

    rl_wheel[0, :] += x
    rl_wheel[1, :] += y

    plt.plot(np.array(outline[0, :]).flatten(),
             np.array(outline[1, :]).flatten(), truckcolor)

    # plt.plot(np.array(rr_wheel[0, :]).flatten(),
    #          np.array(rr_wheel[1, :]).flatten(), truckcolor)

    # plt.plot(np.array(rl_wheel[0, :]).flatten(),
    #          np.array(rl_wheel[1, :]).flatten(), truckcolor)
    # plt.plot(x, y, "*")


def mpc_solver(start, goal, direction):

    """
    Solve MPC problem, direction: -1 backwards, 1 forwards.
    """

    xref = np.empty((NX,2))
    for i in range(NX):
        xref[i,0] = start[i]
        xref[i,1] = goal[i]

    opti = casadi.Opti()
    x = opti.variable(NX, T+1)
    u = opti.variable(NU, T)
    p = opti.parameter(NX, 2)
    opti.set_value(p, xref)
    obj = 0

    for t in range(T):
        # obj += u[:, t].T @ R @ u[:, t]  # control(velocity) cost

        if t < (T - 1):
            obj += (u[:, t + 1] - u[:, t]).T @ Rd @ (u[:, t + 1] - u[:, t])  # acceleration cost
            obj += (x[:, t + 1] - x[:, t]).T @ Qa @ (x[:, t + 1] - x[:, t])

        # if t != 0:
        #     obj += (p[:, -1] - x[:, t]).T @ Q @ (p[:, -1] - x[:, t])
    
    # obj += (x[2, T] - x[3, T]).T @ Qb @ (x[2, T] - x[3, T])

    obj += (p[:, 1] - x[:, T]).T @ Q @ (p[:, 1] - x[:, T])
    
    for t in range(T):
        opti.subject_to(fabs(x[3, t] - x[2, t]) <= np.deg2rad(JACKKNIFE_CON))
        # opti.subject_to(u[0, t + 1] * u[0, t] > 0)

        # Dynamic Constraints
        if direction == 1:
            opti.subject_to(x[0, t+1] == x[0, t] + u[0, t] * cos(x[2, t]) * DT)
            opti.subject_to(x[1, t+1] == x[1, t] + u[0, t] * sin(x[2, t]) * DT)
            opti.subject_to(x[2, t+1] == x[2, t] + u[1, t] * DT)
            opti.subject_to(x[3, t+1] == x[3, t] + u[0, t] / ROD_LEN * sin(x[2, t] - x[3, t]) * DT - CP_OFFSET * u[1, t] * cos(
                x[2, t] - x[3, t]) / ROD_LEN * DT)
        else:
            opti.subject_to(x[0, t+1] == x[0, t] + u[0, t] * cos(x[2, t] - x[3, t]) * cos(x[3, t]) * DT)
            opti.subject_to(x[1, t+1] == x[1, t] + u[0, t] * cos(x[2, t] - x[3, t]) * sin(x[3, t]) * DT)
            opti.subject_to(x[2, t+1] == x[2, t] + u[1, t] * DT)
            opti.subject_to(x[3, t+1] == x[3, t] + u[0, t] / ROD_LEN * sin(x[2, t] - x[3, t]) * DT - CP_OFFSET * u[1, t] * cos(
                x[2, t] - x[3, t]) / ROD_LEN * DT)


    opti.subject_to(x[:, 0] == start)
    # opti.subject_to(x[:, T] == goal)
    # opti.subject_to(x[0, T] == goal[0])
    # opti.subject_to(x[1, T] == goal[1])
    opti.subject_to(x[2, T] == goal[2])
    opti.subject_to(x[3, T] == goal[3])
    opti.subject_to(fabs(u[0, :]) <= MAX_SPEED)
    if direction == -1:
        opti.subject_to(u[0, :] <= MIN_SPEED)
    else:
        opti.subject_to(u[0, :] >= MIN_SPEED)
    # opti.subject_to(u[0, :] < -0.01)
    opti.subject_to(fabs(u[1, :]) <= MAX_OMEGA)

    opti.minimize(obj)

    p_opts = dict(print_time=False, verbose=False)
    s_opts = dict(print_level=5, linear_solver='ma57')
    opti.solver("ipopt", p_opts, s_opts)
    sol = opti.solve()

    ox = get_nparray_from_matrix(sol.value(x)[0, :])
    oy = get_nparray_from_matrix(sol.value(x)[1, :])
    oyaw = get_nparray_from_matrix(sol.value(x)[2, :])
    oyawt = get_nparray_from_matrix(sol.value(x)[3, :])
    ov = get_nparray_from_matrix(sol.value(u)[0, :])
    odyaw = get_nparray_from_matrix(sol.value(u)[1, :])

    return ox, oy, oyaw, oyawt, ov, odyaw

def mp_generator(start, goal, subxlss, subylss, subyawlss, subyawtlss, ind, direction):
    """
    Iterately solve MPC problems to calculate path from start to goal.
    """
    fig, ax = plt.subplots()
    motion_primitive = np.empty((0,4))
    init_state = start.copy()
    current_state = init_state
    # new_row = np.array(start)
    motion_primitive = np.vstack([motion_primitive, start.copy()])
    length = 0
    lengthls = [length]
    ite = 0
    while ite < MAX_ITER:
        try:
            ox, oy, oyaw, oyawt, ov, odyaw = mpc_solver(current_state, goal, direction)
        except:
            print("Can not solve this mpc problem!")
            break
        u = [ov[0], odyaw[0]]
        print("current velocity: ", u)
        if direction == 1:
            current_state = predict_motion_mpc(current_state, u)
        else:
            current_state = predict_motion_mpc_1(current_state, u)
        new_row = np.array(current_state)
        motion_primitive = np.vstack([motion_primitive, new_row])
        length += ov[0] * DT
        lengthls.append(length)
        ite += 1
        # if (init_state[3] == 0 and goal[3] == 0 and fabs(current_state[0] - goal[0]) < 0.01 and fabs(current_state[1] - goal[1]) < 0.01) or ((init_state[3] != 0 or goal[3] != 0) and fabs(current_state[2] - goal[2]) < 0.01 and fabs(current_state[3] - goal[3]) < 0.01):
        print(fabs(current_state[2] - goal[2]), fabs(current_state[3] - goal[3]))
        if fabs(current_state[2] - goal[2]) < RES and fabs(current_state[3] - goal[3]) < RES:
            
            # Condition 1 for straight path, condition 2 for curve path.
            if goal[2] == 0 and start[3] == 0:
                if fabs(current_state[0] - goal[0]) < 0.01 and fabs(current_state[1] - goal[1]) < 0.01:
                    print("satisfied stop condition 1")
                    break
                continue
            else:
                # if fabs(current_state[0] - goal[0]) < 0.01 and fabs(current_state[1] - goal[1]) < 0.01:
                # ite_w = 0
                # while True:
                #     if fabs(current_state[2] - current_state[3]) < 0.15: # or current_state[0] >= 1.2 or current_state[1] >= 1.2:
                #         break
                #     if direction == 1:
                #         temp_state = predict_motion_mpc(current_state, [0.01,0])
                #     else:
                #         temp_state = predict_motion_mpc_1(current_state, [0.01,0])
                #     temp_state = predict_motion_mpc(current_state, [0.01,0])
                #     current_state = temp_state
                #     ite_w += 1
                # print("Satisfy break condition!")
                # new_row = np.array(current_state)
                # motion_primitive = np.vstack([motion_primitive, new_row])
                # length += 0.01 * ite_w * DT
                # lengthls.append(length)
                # ite += 1
                print("satisfied stop condition 2")
                break
        

    xlsn = [i[0] for i in motion_primitive]
    ylsn = [i[1] for i in motion_primitive]
    yawlsn = [i[2] for i in motion_primitive]
    yawtlsn = [i[3] for i in motion_primitive]

    # print("Check while stop conditions: ", current_state[0], goal[0], current_state[1], goal[1])
    print("Iteration ", ite)
    print("Goal: ", goal)

    xlsn, ylsn, yawlsn, yawtlsn = interpose(xlsn, ylsn, yawlsn, yawtlsn, lengthls)

    # Adjust final position to fit resolution.
    xlsn[-1] = round(xlsn[-1] / RES) * RES
    ylsn[-1] = round(ylsn[-1] / RES) * RES
    yawlsn[-1] = goal[2]
    yawtlsn[-1] = goal[3]

    # if direction == -1:
    #     xlsn.reverse()
    #     ylsn.reverse()
    #     yawlsn.reverse()
    #     yawtlsn.reverse()
    #     yawlsn[-1] = start[2]
    #     yawtlsn[-1] = start[2]
    # a = xlsn[0]
    # for i in range(len(xlsn)):
    #     xlsn[i] -= a

    subxlss.append(xlsn)
    subylss.append(ylsn)
    subyawlss.append(yawlsn)
    subyawtlss.append(yawtlsn)

    ax = plt.gca()
    ax.plot(xlsn, ylsn, label=ind)
    for xi, yi, yawi, yawti in zip(xlsn, ylsn, yawlsn, yawtlsn):
        print("x, y, yaw, yawt: ",xi, yi, yawi, yawti)
        draw_rotated_box(xi, yi, np.rad2deg(yawi), np.rad2deg(yawti), 'blue')
    print(current_state[1])
    plt.legend()
    plt.axis('equal')
    plt.show()
    
def main():
    start_state = [[0, 0, 0, 0]]

    # start_state = [[0, 0, 0, np.deg2rad(-45)],
    #                [0, 0, 0, np.deg2rad(-22.5)],
    #                [0, 0, 0, 0],
    #                [0, 0, 0, np.deg2rad(22.5)],
    #                [0, 0, 0, np.deg2rad(45)]]
    
    # Forwards goal states
    goal_state = [[0.3, 0, 0, 0], # possible: downsampling Jing 17.05.2023
                  [0.45, 0, 0, 0],
                  [2, 0, np.deg2rad(90), np.deg2rad(90)],
                  [2, 0, np.deg2rad(67.5), np.deg2rad(67.5)],
                  [2, 0, np.deg2rad(45), np.deg2rad(45)],
                  [2, 0, np.deg2rad(22.5), np.deg2rad(22.5)]]
    
    # goal_state = [[0.15, 0, 0, 0],
    #               [0.3, 0, 0, 0],
    #               [-0.3, 0, 0, 0],
    #               [2, 0, np.deg2rad(90), np.deg2rad(45)],
    #               [2, 0, np.deg2rad(67.5), np.deg2rad(22.5)],
    #               [2, 0, np.deg2rad(45), np.deg2rad(22.5)],
    #               [2, 0, np.deg2rad(22.5), np.deg2rad(0)]]
    
    # Calculate end pose angle index
    cal_goal_ind = []
    for goal in goal_state:
        cal_goal_ind.append(int(np.rad2deg(goal[2])/22.5))

    # This subls include all mps of one start state.
    subxlss = []
    subylss = []
    subyawlss = []
    subyawtlss = []

    for start in start_state:
        # fig, ax = plt.subplots()
        ind = 0
        direction = 1
        for goal in goal_state:
            mp_generator(start, goal, subxlss, subylss, subyawlss, subyawtlss, ind, direction)
            ind += 1


    goal_state1 = goal_state.copy()

    # for i in range(16):
    #     start_state1[0][i][0], start_state1[0][i][1], start_state1[0][i][2], start_state1[0][i][3] = -start_state1[0][i][1], start_state1[0][i][0], -start_state1[0][i][2], -start_state1[0][i][3]
    # for i in range(4):
    #     start_state1[0][i+16][0], start_state1[0][i+16][1], start_state1[0][i+16][2], start_state1[0][i+16][3] = -start_state1[0][i+16][0] - 0.3, start_state1[0][i+16][1], -start_state1[0][i+16][2], -start_state1[0][i+16][3]
    
    # Backwards goal states goal_state1
    for i in range(6):
        goal_state1[i][0], goal_state1[i][1], goal_state1[i][2], goal_state1[i][3] = -goal_state1[i][0], goal_state1[i][1], -goal_state1[i][2], -goal_state1[i][3]

    for goal_b in goal_state1:
        angle_ind = int(np.rad2deg(goal_b[2])/22.5)
        if angle_ind < 0:
            angle_ind += 16
        cal_goal_ind.append(angle_ind)

    for start in start_state:
        # fig, ax = plt.subplots()
        ind = 0
        direction = -1
        for goal in goal_state1:
            mp_generator(start, goal, subxlss, subylss, subyawlss, subyawtlss, ind, direction)
            ind += 1
    
    # Add motion primitives in Quadrants 3 and 4
    subxlss_2 = subxlss[2:6]
    subylss_2 = subylss[2:6]
    subyawlss_2 = subyawlss[2:6]
    subyawtlss_2 = subyawtlss[2:6]

    subxlss_3 = subxlss[8:]
    subylss_3 = subylss[8:]
    subyawlss_3 = subyawlss[8:]
    subyawtlss_3 = subyawtlss[8:]

    for xls, yls, yawls, yawtls in zip(subxlss_2, subylss_2, subyawlss_2, subyawtlss_2):
        subxlss.append([x for x in xls])
        subylss.append([-y for y in yls])
        subyawlss.append([-yaw for yaw in yawls])
        subyawtlss.append([-yawt for yawt in yawtls])

    for xls, yls, yawls, yawtls in zip(subxlss_3, subylss_3, subyawlss_3, subyawtlss_3):
        subxlss.append([x for x in xls])
        subylss.append([-y for y in yls])
        subyawlss.append([-yaw for yaw in yawls])
        subyawtlss.append([-yawt for yawt in yawtls])

    print("mp size: ", len(subxlss))
    print(cal_goal_ind)
    mpls = [subxlss, subylss, subyawlss, subyawtlss]
    # plt.legend()
    # plt.axis('equal')
    # plt.show()
    
    #  Save generated motion primitives to file mp.pkl.
    with open('mp.pkl', 'wb') as file:
        pickle.dump(mpls, file)

if __name__ == "__main__":
    main()