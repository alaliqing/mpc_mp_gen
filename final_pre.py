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
Rd = np.diag([1, 1])  # ([0.01, 1.0])  # input difference cost matrix
Q = np.diag([0.3, 0.3, 0, 0]) # final matrix
Qa = np.diag([10, 10, 10, 10])  # state difference matrix
GOAL_DIS = 1.1  # goal distance
STOP_SPEED = 0.5 / 3.6  # stop speed
MAX_TIME = 100.0  # max simulation time

# iterative paramter
MAX_ITER = 100 # Max iteration
DU_TH = 0.1  # 0.1  # iteration finish param

DT = 0.6  # [s] time tick 0.1 for 45degree

# Vehicle parameters
LENGTH = 0.72 # / 20 # [m]
LENGTH_T = 0.36 # / 20  # [m]
WIDTH = 0.48 # / 20  # [m]
BACKTOWHEEL = 0.36  # [m]
WHEEL_LEN = 0.1  # [m]
WHEEL_WIDTH = 0.07  # [m]
TREAD = 0.2  # [m]
WB = 0.3  # [m]
ROD_LEN = 0.5 # / 20  # [m]
CP_OFFSET = 0.1 # / 20  # [m]

MAX_OMEGA = 1.5 # maximum rotation speed [rad/s]

MAX_SPEED = 5  # 55.0 / 3.6  # maximum speed [m/s]
MIN_SPEED = 0.01  # minimum speed [m/s]
JACKKNIFE_CON = 45.0  # [degrees]

CIR_RAD = 5  # radius of circular path [m]

def get_nparray_from_matrix(x):
    return np.array(x).flatten()

def predict_motion_mpc(x, u, ind):
    x1 = x
    x1[0] = x[0] + u[0] * cos(x[2]) * DT
    x1[1] = x[1] + u[0] * sin(x[2]) * DT
    x1[2] = x[2] + u[1] * DT
    if ind < 6:
        x1[3] = x[3] + u[0] / ROD_LEN * sin(x[2] - x[3]) * DT - CP_OFFSET * u[1] * cos(x[2] - x[3]) / ROD_LEN * DT
    else:
        x1[3] = x[3] + u[0] / ROD_LEN * sin(x[2] - x[3]) * DT - CP_OFFSET * u[1] * cos(x[2] - x[3]) / ROD_LEN * DT
    return x1

def interpose(xls, yls, yawls, yawtls, lengthls):
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
    for i in range(10):
        ylsn_.append(np.interp(xlsn_[i], xls, yls))
        yawlsn_.append(np.interp(xlsn_[i], xls, yawls))
        yawtlsn_.append(np.interp(xlsn_[i], xls, yawtls))
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
    margin = 20
    import numpy as np
    a = LENGTH/20 # length
    b = WIDTH/20 # width
    c = a/2 # length2

    yaw1rad = np.deg2rad(yaw1)
    yawtrad = np.deg2rad(yawt)

    x1 = x - c * cos(yaw1rad) - c * cos(yawtrad)
    y1 = y - c * sin(yaw1rad) - c * sin(yawtrad)

    # x = x - a/2 * cos(yaw1) - b/2 * sin(yaw1)
    # y = y - a/2 * sin(yaw1) + b/2 * cos(yaw1)

    # x1 = x1 - c/2 * cos(yawt) - b/2 * sin(yawt)
    # y1 = y1 - c/2 * sin(yawt) + b/2 * cos(yawt)

    rect1 = rectangle_vertex(x, y, a, b, yaw1, 'blue')
    rect2 = rectangle_vertex(x1, y1, c, b, yawt, 'green')


    ax = plt.gca()
    ax.add_patch(rect1)
    ax.add_patch(rect2)
    # ax.plot(xls1, yls1, 'g-')
    # ax.plot(xls2, yls2, 'b-')

# def draw_rotated_box(x, y, yaw1, yawt, color, width=0.04, height=0.02):
#     margin = 20
#     import numpy as np
#     a = LENGTH # length
#     b = WIDTH # width
#     c = a/2 # length2

#     yaw1rad = np.deg2rad(yaw1)
#     yawtrad = np.deg2rad(yawt)

#     plot_car(x, y, yaw1rad, LENGTH)

#     # for tractor
#     plot_car(x - np.cos(yawtrad) * c - np.cos(yaw1rad) * c,
#                 y - np.sin(yawtrad) * c - np.sin(yaw1rad) * c, yawtrad, LENGTH/2, truckcolor="-r")

#     # x1 = x - c * cos(yaw1rad) - c * cos(yawtrad)
#     # y1 = y - c * sin(yaw1rad) - c * sin(yawtrad)

#     # # x = x - a/2 * cos(yaw1) - b/2 * sin(yaw1)
#     # # y = y - a/2 * sin(yaw1) + b/2 * cos(yaw1)

#     # # x1 = x1 - c/2 * cos(yawt) - b/2 * sin(yawt)
#     # # y1 = y1 - c/2 * sin(yawt) + b/2 * cos(yawt)
#     # print(yaw1)

#     # r = Rotation.from_euler('z', yaw1rad)
#     # print(r)


#     # pos2 = r.apply([x-a/2, y-b/2, 0])
#     # print(yaw1, yaw1rad, "x", x, pos2[0],x-a*cos(yaw1rad)/2-b*sin(yaw1rad)/2, "y", y, pos2[1], y+a*sin(yaw1rad)/2-b*cos(yaw1rad)/2)

#     #rect1 = rectangle_vertex(x-a*cos(yaw1rad)/2-b*sin(yaw1rad)/2, y+a*sin(yaw1rad)/2-b*cos(yaw1rad)/2, a, b, yaw1, 'blue') #x-a*cos(yaw1)/2-b*sin(yaw1)/2, y+a*sin(yaw1)/2-b*cos(yaw1)/2
#     #rect2 = rectangle_vertex(x1, y1, c, b, yawt, 'green')


#     ax = plt.gca()
#     # ax.add_patch(rect1)
#     # ax.add_patch(rect2)
#     # ax.plot(xls1, yls1, 'g-')
#     # ax.plot(xls2, yls2, 'b-')

# def plot_car(x, y, yaw, length, steer=0.0, cabcolor="-r", truckcolor="-k"):  # pragma: no cover

#     outline = np.array([[-length / 2, (length - length / 2), (length - length / 2), -length / 2, -length / 2],
#                         [WIDTH / 2, WIDTH / 2, - WIDTH / 2, -WIDTH / 2, WIDTH / 2]])

#     rr_wheel = np.array([[WHEEL_LEN, -WHEEL_LEN, -WHEEL_LEN, WHEEL_LEN, WHEEL_LEN],
#                          [-WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD,
#                           -WHEEL_WIDTH - TREAD]])

#     rl_wheel = np.copy(rr_wheel)
#     rl_wheel[1, :] *= -1

#     Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
#                      [-math.sin(yaw), math.cos(yaw)]])

#     outline = (outline.T.dot(Rot1)).T
#     rr_wheel = (rr_wheel.T.dot(Rot1)).T
#     rl_wheel = (rl_wheel.T.dot(Rot1)).T

#     outline[0, :] += x
#     outline[1, :] += y

#     rr_wheel[0, :] += x
#     rr_wheel[1, :] += y

#     rl_wheel[0, :] += x
#     rl_wheel[1, :] += y

#     plt.plot(np.array(outline[0, :]).flatten(),
#              np.array(outline[1, :]).flatten(), truckcolor)

#     # plt.plot(np.array(rr_wheel[0, :]).flatten(),
#     #          np.array(rr_wheel[1, :]).flatten(), truckcolor)

#     # plt.plot(np.array(rl_wheel[0, :]).flatten(),
#     #          np.array(rl_wheel[1, :]).flatten(), truckcolor)
#     # plt.plot(x, y, "*")

def mpc_solver(start, goal):

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
            # obj += (p[:, -1] - x[:, t]).T @ Q @ (p[:, -1] - x[:, t])
    
    # obj += (p[:, 1] - x[:, T]).T @ Q @ (p[:, 1] - x[:, T])

    for t in range(T):
        opti.subject_to(fabs(x[3, t] - x[2, t]) <= np.deg2rad(JACKKNIFE_CON))
        # opti.subject_to(u[0, t + 1] * u[0, t] > 0)

        # Dynamic Constraints
        opti.subject_to(x[0, t+1] == x[0, t] + u[0, t] * cos(x[2, t]) * DT)
        opti.subject_to(x[1, t+1] == x[1, t] + u[0, t] * sin(x[2, t]) * DT)
        opti.subject_to(x[2, t+1] == x[2, t] + u[1, t] * DT)
        opti.subject_to(x[3, t+1] == x[3, t] + u[0, t] / ROD_LEN * sin(x[2, t] - x[3, t]) * DT - CP_OFFSET * u[1, t] * cos(
            x[2, t] - x[3, t]) / ROD_LEN * DT)
    

    opti.subject_to(x[:, 0] == start)
    # opti.subject_to(x[:, T] == goal)
    opti.subject_to(x[0, T] == goal[0])
    opti.subject_to(x[1, T] == goal[1])
    opti.subject_to(x[2, T] == goal[2])
    # opti.subject_to(x[3, T] == goal[3])
    opti.subject_to(fabs(u[0, :]) <= MAX_SPEED)
    # opti.subject_to(fabs(u[0, :]) >= MIN_SPEED)
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

def mpc_solver1(start, goal):

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
            # obj += (p[:, -1] - x[:, t]).T @ Q @ (p[:, -1] - x[:, t])
    
    # obj += (p[:, 1] - x[:, T]).T @ Q @ (p[:, 1] - x[:, T])

    for t in range(T):
        opti.subject_to(fabs(x[3, t] - x[2, t]) <= np.deg2rad(JACKKNIFE_CON))
        # opti.subject_to(u[0, t + 1] * u[0, t] > 0)

        # Dynamic Constraints
        opti.subject_to(x[0, t+1] == x[0, t] + u[0, t] * cos(x[2, t]) * DT)
        opti.subject_to(x[1, t+1] == x[1, t] + u[0, t] * sin(x[2, t]) * DT)
        opti.subject_to(x[2, t+1] == x[2, t] + u[1, t] * DT)
        opti.subject_to(x[3, t+1] == x[3, t] + u[0, t] / ROD_LEN * sin(x[2, t] - x[3, t]) * DT - CP_OFFSET * u[1, t] * cos(
            x[2, t] - x[3, t]) / ROD_LEN * DT)
    

    opti.subject_to(x[:, 0] == start)
    # opti.subject_to(x[:, T] == goal)
    opti.subject_to(x[0, T] == goal[0])
    opti.subject_to(x[1, T] == goal[1])
    opti.subject_to(x[2, T] == goal[2])
    # opti.subject_to(x[3, T] == goal[3])
    opti.subject_to(fabs(u[0, :]) <= MAX_SPEED)
    # opti.subject_to(fabs(u[0, :]) >= MIN_SPEED)
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
    
def main():
    start_state = [#[0, 0, 0, np.deg2rad(-45)],
                #    [0, 0, 0, np.deg2rad(-22.5)],
                   [0, 0, 0, 0],
                   [0, 0, 0, np.deg2rad(22.5)],
                   [0, 0, 0, np.deg2rad(45)]]
                   
                   
                   
    
    goal_state = [#[[0, 0, 0, 0], # -45
                #    [0, 0, 0, np.deg2rad(-22.5)],
                #    [0, 0, np.deg2rad(22.5), np.deg2rad(-22.5)],
                #    [0, 0, np.deg2rad(22.5), np.deg2rad(0)],
                #    [0, 0, np.deg2rad(45.0), np.deg2rad(0)],
                #    [0, 0, np.deg2rad(-22.5), np.deg2rad(-67.5)],
                # #    [0, 0, np.deg2rad(-22.5), np.deg2rad(-45.0)], # 1
                #    [0, 0, np.deg2rad(-22.5), np.deg2rad(-22.5)],
                #    [0, 0, np.deg2rad(-22.5), np.deg2rad(0)],
                #    [0, 0, np.deg2rad(-45.0), np.deg2rad(-90)],
                #    [0, 0, np.deg2rad(-45.0), np.deg2rad(-67.5)],
                #    [0, 0, np.deg2rad(-45.0), np.deg2rad(-45)]],
                
                #   [[0, 0, 0, 0], # -225
                #    [0, 0, 0, np.deg2rad(-45.0)],
                #    [0, 0, np.deg2rad(22.5), np.deg2rad(-22.5)],
                #    [0, 0, np.deg2rad(22.5), np.deg2rad(0)],
                #    [0, 0, np.deg2rad(45.0), np.deg2rad(0)],
                #    [0, 0, np.deg2rad(45.0), np.deg2rad(22.5)],
                #    [0, 0, np.deg2rad(-22.5), np.deg2rad(-67.5)],
                #    [0, 0, np.deg2rad(-22.5), np.deg2rad(-45.0)],
                #    [0, 0, np.deg2rad(-22.5), np.deg2rad(-22.5)],
                #    [0, 0, np.deg2rad(-45.0), np.deg2rad(-67.5)],
                #    [0, 0, np.deg2rad(-45.0), np.deg2rad(-45)]],
    
                  [[0.6, 0.9, np.deg2rad(90), np.deg2rad(67.5)],
                   [0.6, 0, 0, 0], # 0
                   [1.2, 0, 0, 0],
                   [-0.6, 0, 0, 0],
                #    [0, 0, 0, np.deg2rad(45.0)],
                   [0, 0, 0, np.deg2rad(22.5)],
                #    [0, 0, 0, np.deg2rad(-45.0)],
                   [0, 0, 0, np.deg2rad(-22.5)],
                   [0, 0, np.deg2rad(22.5), np.deg2rad(-22.5)],
                   [0, 0, np.deg2rad(22.5), np.deg2rad(22.5)],
                   [0, 0, np.deg2rad(45.0), np.deg2rad(22.5)],
                   [0, 0, np.deg2rad(-22.5), np.deg2rad(-22.5)],
                   [0, 0, np.deg2rad(-22.5), np.deg2rad(22.5)],
                   [0, 0, np.deg2rad(-45.0), np.deg2rad(-22.5)]],
                #    [0, 0, np.deg2rad(-45.0), np.deg2rad(0)]]]

                  [[0, 0, 0, 0], # 225
                   [0, 0, 0, np.deg2rad(45.0)],
                   [0, 0, np.deg2rad(22.5), np.deg2rad(45.0)],
                   [0, 0, np.deg2rad(22.5), np.deg2rad(22.5)],
                   [0, 0, np.deg2rad(22.5), np.deg2rad(67.5)],
                   [0, 0, np.deg2rad(45.0), np.deg2rad(67.5)],
                   [0, 0, np.deg2rad(45.0), np.deg2rad(90.0)],
                   [0, 0, np.deg2rad(-22.5), np.deg2rad(0)], # 1
                   [0, 0, np.deg2rad(-22.5), np.deg2rad(22.5)],
                   [0, 0, np.deg2rad(-45.0), np.deg2rad(-22.5)],
                   [0, 0, np.deg2rad(-45.0), np.deg2rad(0)]],

                  [[0, 0, 0, np.deg2rad(22.5)],  # 45
                   [0, 0, 0, 0],
                   [0, 0, np.deg2rad(22.5), np.deg2rad(22.5)],
                   [0, 0, np.deg2rad(22.5), np.deg2rad(45)],
                   [0, 0, np.deg2rad(22.5), np.deg2rad(67.5)],
                   [0, 0, np.deg2rad(-22.5), np.deg2rad(0)],
                   [0, 0, np.deg2rad(-22.5), np.deg2rad(22.5)],
                   [0, 0, np.deg2rad(45.0), np.deg2rad(45.0)],
                   [0, 0, np.deg2rad(45.0), np.deg2rad(67.5)],
                   [0, 0, np.deg2rad(45.0), np.deg2rad(90.0)],
                   [0, 0, np.deg2rad(-45.0), np.deg2rad(0)]]]

    xlss = []
    ylss = []
    yawlss = []
    yawtlss = []

    for start, goalset in zip(start_state, goal_state):
        subxlss = []
        subylss = []
        subyawlss = []
        subyawtlss = []
        fig, ax = plt.subplots()
        ind = 0
        for goal in goalset:
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
                    if init_state[3] == 0 and goal[3] == 0:
                        ox, oy, oyaw, oyawt, ov, odyaw = mpc_solver1(current_state, goal)
                    else:
                        ox, oy, oyaw, oyawt, ov, odyaw = mpc_solver(current_state, goal)
                except:
                    print("Can not solve this mpc problem!")
                    break
                u = [ov[0], odyaw[0]]
                print("current velocity: ", u)
                current_state = predict_motion_mpc(current_state, u, ind)
                
                if (init_state[3] == 0 and goal[3] == 0 and fabs(current_state[0] - goal[0]) < 0.01 and fabs(current_state[1] - goal[1]) < 0.01) or ((init_state[3] != 0 or goal[3] != 0) and fabs(current_state[2] - goal[2]) < 0.01 and fabs(current_state[3] - goal[3]) < 0.01):
                # if fabs(current_state[0] - goal[0]) < 0.01 and fabs(current_state[1] - goal[1]) < 0.01:
                    print("Satisfy break condition!")
                    break
                new_row = np.array(current_state)
                motion_primitive = np.vstack([motion_primitive, new_row])
                length += ov[0] * DT
                lengthls.append(length)
                ite += 1

            xlsn = [i[0] for i in motion_primitive]
            ylsn = [i[1] for i in motion_primitive]
            yawlsn = [i[2] for i in motion_primitive]
            yawtlsn = [i[3] for i in motion_primitive]

            ind += 1

            print("Check while stop conditions: ", current_state[0], goal[0], current_state[1], goal[1])
            print("Iteration ", ite)

            xlsn, ylsn, yawlsn, yawtlsn = interpose(xlsn, ylsn, yawlsn, yawtlsn, lengthls)
            xlsn[-1] = round(xlsn[-1] / 0.03) * 0.03
            ylsn[-1] = round(ylsn[-1] / 0.03) * 0.03
            yawlsn[-1] = goal[2]
            yawtlsn[-1] = goal[3]

            subxlss.append(xlsn)
            subylss.append(ylsn)
            subyawlss.append(yawlsn)
            subyawtlss.append(yawtlsn)

            ax.plot(xlsn, ylsn, label=ind)
            for xi, yi, yawi, yawti in zip(xlsn, ylsn, yawlsn, yawtlsn):
                print("yaw, yawt: ", yawi, yawti)
                draw_rotated_box(xi, yi, np.rad2deg(yawi), np.rad2deg(yawti), 'blue')
            plt.legend()
            plt.axis('equal')
            plt.show()
        xlss.append(subxlss)
        ylss.append(subylss)
        yawlss.append(subyawlss)
        yawtlss.append(subyawtlss)
        mpls = [xlss, ylss, yawlss, yawtlss]
        # plt.legend()
        # plt.axis('equal')
        # plt.show()
    with open('mp.pkl', 'wb') as file:
        pickle.dump(mpls, file)

if __name__ == "__main__":
    main()