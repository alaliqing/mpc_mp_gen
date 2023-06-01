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
MAX_ITER = 300 # Max iteration
DU_TH = 0.1  # 0.1  # iteration finish param

DT = 0.3  # [s] time tick 0.1 for 45degree

# Vehicle parameters
LENGTH = 0.72 / 10  # [m]
LENGTH_T = 0.36 / 10  # [m]
WIDTH = 0.48 / 10  # [m]
BACKTOWHEEL = 0.36  # [m]
WHEEL_LEN = 0.1  # [m]
WHEEL_WIDTH = 0.07  # [m]
TREAD = 0.2  # [m]
WB = 0.3  # [m]
ROD_LEN = 0.5 / 10  # [m]
CP_OFFSET = 0.1 / 10  # [m]

RES = 0.03

MAX_OMEGA = 0.5 # maximum rotation speed [rad/s]

MAX_SPEED = 0.5  # 55.0 / 3.6  # maximum speed [m/s]
MIN_SPEED = 0  # minimum speed [m/s]
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
    a = LENGTH # length
    b = WIDTH # width
    c = a/2 # length2

    yaw1rad = np.deg2rad(yaw1)
    yawtrad = np.deg2rad(yawt)

    plot_car(x, y, yaw1rad, LENGTH)

    # for tractor
    plot_car(x - np.cos(yawtrad) * c - np.cos(yaw1rad) * c,
                y - np.sin(yawtrad) * c - np.sin(yaw1rad) * c, yawtrad, LENGTH/2, truckcolor="-r")

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

def plot_car(x, y, yaw, length, steer=0.0, cabcolor="-r", truckcolor="-k"):  # pragma: no cover

    outline = np.array([[-length / 2, (length - length / 2), (length - length / 2), -length / 2, -length / 2],
                        [WIDTH / 2, WIDTH / 2, - WIDTH / 2, -WIDTH / 2, WIDTH / 2]])

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


def main():
    """
    Iterate over all start angles(16)
    """
    with open('mp.pkl', 'rb') as file:
        mpls = pickle.load(file)
    xlss = mpls[0]
    ylss = mpls[1]
    yawlss = mpls[2]
    yawtlss = mpls[3]
    # # Start to calculate other directions
    numberofangles = 16
    numberofprimsperangle = 24
    numberofjointangles = 1

    # Multipliers
    forwardcostmult = 1
    backwardcostmult = 1
    forwardandturncostmult225 = 1
    forwardandturncostmult450 = 3
    backwardandturncostmult225 = 1
    backwardandturncostmult450 = 3
    jackknifecostmult = 100000

    # Eleven prims
    fout = open("newmp.mprim", 'w')
    fout.write('resolution_m: %f\n' % RES)
    fout.write('numberofangles: %d\n' % numberofangles)
    fout.write('numberofjointangles: %d\n' % numberofjointangles)
    fout.write('totalnumberofprimitives: %d\n' % (numberofprimsperangle * numberofangles * numberofjointangles))

    # goal_angles = [0, 0, 0, 4, 4, 4, 3, 3, 3, 4, 4, 4, 4, 3, 3, 3, 0, 0, 1, 1, 
    #                12, 12, 12, 13, 13, 13, 12, 12, 12, 12, 13, 13, 13, 0, 0, 15, 15,
    #                12, 12, 12, 13, 13, 13, 12, 12, 12, 12, 13, 13, 13, 0, 0, 15, 15,
    #                4, 4, 4, 3, 3, 3, 4, 4, 4, 4, 3, 3, 3, 0, 0, 1, 1]

    goal_angles= [0, 0, 8, 4, 3, 2, 1, 0, 0, 8, 12, 13, 14, 15,
                  8, 12, 13, 14, 15, 8, 4, 3, 2, 1]

    for angleind in range(numberofangles):
        current_angle = (angleind) * 2 * math.pi / numberofangles
        r = Rotation.from_euler('z', current_angle)
        fig, ax = plt.subplots()
        # for joint in range(numberofjointangles):
        for primind in range(numberofprimsperangle):
            # fig, ax = plt.subplots()
            fout.write('primID: %d\n' % (primind))
            fout.write('startangle_c: %d\n' % (angleind))
            fout.write('startjointangle_c: %d\n' % (0))
            each_angle_path = []
            # print("xlss size: ", len(xlss), len(ylss), len(yawlss), len(yawtlss))
            
            # Rotation and fit to resolution
            for x, y, yaw, yawt in zip(xlss[primind], ylss[primind], yawlss[primind], yawtlss[primind]):
                inter_coord = r.apply([x, y, 0])
                yaw += current_angle
                yawt += current_angle
                inter_coord[2] = yaw
                inter_coord = np.append(inter_coord, yawt)
                each_angle_path.append(inter_coord)
            each_angle_path[-1][0] = round(each_angle_path[-1][0] / RES) * RES
            each_angle_path[-1][1] = round(each_angle_path[-1][1] / RES) * RES
            start_state = [0, 0, current_angle, current_angle]
            goal_state = each_angle_path[-1]

            xls1 = [i[0] for i in each_angle_path]
            yls1 = [i[1] for i in each_angle_path]
            yawls1 = [i[2] for i in each_angle_path]
            yawtls1 = [i[3] for i in each_angle_path]

            ax.plot(xls1, yls1, label=primind)
            for xi, yi, yawi, yawti in zip(xls1, yls1, yawls1, yawtls1):
                draw_rotated_box(xi, yi, np.rad2deg(yawi), np.rad2deg(yawti), 'blue')

            # cur_joint_angle = 0
            
            # if cur_joint_angle == 0 or cur_joint_angle == 4:
            #     multcost = 3
            # elif joint == 2 and primind == 0 or primind == 1 or primind == 2:
            #     multcost = 1
            # else:
            multcost = 2

            # Set straight mp cost to 1
            if primind == 0 or primind == 1 or primind == 6 or primind == 7:
                multcost = 1
            
            # if fabs(round(goal_state[0]/RES)) <= 5:
            #     multcost = 1000000

            fout.write(f"endpose_c: {round(goal_state[0]/RES)} {round(goal_state[1]/RES)} {goal_angles[primind]+angleind} {0}\n")
            fout.write(f"additionalactioncostmult: {multcost}\n")
            fout.write(f"intermediateposes: {10}\n")
            for interind in range(10):
                fout.write(f"{round(each_angle_path[interind][0], 4):.4f} {round(each_angle_path[interind][1], 4):.4f} {round(each_angle_path[interind][2], 4):.4f} {round((each_angle_path[interind][3] - each_angle_path[interind][2]), 4):.4f}\n")
        plt.axis('equal')
        # plt.show()
    print("Finish generating motion primitives!")
    fout.close()

if __name__ == "__main__":
    main()

