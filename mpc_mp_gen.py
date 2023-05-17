"""

Path tracking simulation with iterative linear model predictive control

Robot model: differential driven mobile robot with passive trailer, reference point on the trailer

State variable: (x2, y2, theta1, theta2)
Control input variable: (v1, w1)

author: Yucheng Tang (@Yucheng-Tang)

Citation: Atsushi Sakai (@Atsushi_twi)
"""
import casadi
import pandas as pd
import matplotlib.pyplot as plt
import cvxpy
import math
import numpy as np
import sys
import os
import time
from sympy import *

# import dccp

from casadi import *

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../PathPlanning/CubicSpline/")

try:
    import cubic_spline_planner
except:
    raise

NX = 4  # x = x, y, yaw, yawt
NU = 2  # u = [v, w]
T = 10  # horizon length 5

lam = 0.05  # barrier function param
r_f = 80  # reachable set param
circle_robot_limit = 70  # determine if the robot is tangent to a circle

# mpc parameters
R = np.diag([0.01, 0.01])  # input cost matrix
Rd = np.diag([0.01, 1.0])  # ([0.01, 1.0])  # input difference cost matrix
Q = np.diag([1.0, 1.0, 0.001, 0.1])
Qf = np.diag([100.0, 100.0, 0.01, 1])  # state final matrix
GOAL_DIS = 1.1  # goal distance
STOP_SPEED = 0.5 / 3.6  # stop speed
MAX_TIME = 500.0  # max simulation time

# iterative paramter
MAX_ITER = 5  # Max iteration
DU_TH = 0.1  # 0.1  # iteration finish param

TARGET_SPEED = 0.25  # [m/s] target speed
N_IND_SEARCH = 10  # Search index number

DT = 0.01  # [s] time tick

# Vehicle parameters
LENGTH = 0.72  # [m]
LENGTH_T = 0.36  # [m]
WIDTH = 0.48  # [m]
BACKTOWHEEL = 0.36  # [m]
WHEEL_LEN = 0.1  # [m]
WHEEL_WIDTH = 0.05  # [m]
TREAD = 0.2  # [m]
WB = 0.3  # [m]
ROD_LEN = 0.5  # [m]
CP_OFFSET = 0.1  # [m]

# Obstacle parameter
O_X = [-3]  # [-3]# , -4.5] # -2
O_Y = [-0.3]  # [4.2]# , -2.2] # 0.1
O_R = [1.0]  # , 0.5] # 0.5
O_D = [100000000]

MAX_OMEGA = np.deg2rad(90.0)  # maximum rotation speed [rad/s]

MAX_SPEED = 0.2  # 55.0 / 3.6  # maximum speed [m/s]
MIN_SPEED = 0.2  # minimum speed [m/s]
JACKKNIFE_CON = 45.0  # [degrees]

CIR_RAD = 5  # radius of circular path [m]

TIME_LIST = []

PX_SET = []
PY_SET = []
X_SET = []
Y_SET = []

BARRIER_LIST = []

# TODO: add MODE parameter for path selection
MODE = ""

show_animation = True


class State:
    """
    vehicle state class
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, yawt=0.0, v=0.0, dyaw=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.yawt = yawt
        self.predelta = None


def pi_2_pi(angle):
    while (angle > math.pi):
        angle = angle - 2.0 * math.pi

    while (angle < -math.pi):
        angle = angle + 2.0 * math.pi

    return angle


def get_linear_model_matrix(xref_t, xref_t1, x_t):
    rp_t = np.array([xref_t[0], xref_t[1]])
    rp_t1 = np.array([xref_t1[0], xref_t1[1]])
    v_r = np.linalg.norm(rp_t1 - rp_t)
    w_r = xref_t1[2] - xref_t[2]

    A = np.zeros((NX, NX))
    A[0, 0] = 1.0
    A[1, 1] = 1.0
    A[2, 2] = 1.0
    # A[3, 3] = 1.0

    # model from pythonrobotic
    A[0, 2] = v_r * math.sin(x_t[3] - x_t[2]) * math.cos(x_t[3])
    A[0, 3] = - v_r * math.sin(2 * x_t[3] - x_t[2])
    A[1, 2] = v_r * math.sin(x_t[3] - x_t[2]) * math.sin(x_t[3])
    A[1, 3] = v_r * math.cos(2 * x_t[3] - x_t[2])
    A[3, 2] = (v_r * math.cos(x_t[3] - x_t[2]) - CP_OFFSET * w_r * math.sin(x_t[3] - x_t[2])) / ROD_LEN
    A[3, 3] = 1.0 - (v_r * math.cos(x_t[3] - x_t[2]) - CP_OFFSET * w_r * math.sin(x_t[3] - x_t[2])) / ROD_LEN

    B = np.zeros((NX, NU))

    # model from pythonrobotics
    B[0, 0] = DT * math.cos(x_t[3] - x_t[2]) * math.cos(x_t[3])
    B[1, 0] = DT * math.cos(x_t[3] - x_t[2]) * math.sin(x_t[3])
    B[2, 1] = DT
    B[3, 0] = - DT * math.sin(x_t[3] - x_t[2]) / ROD_LEN
    B[3, 1] = - DT * CP_OFFSET * math.cos(x_t[3] - x_t[2]) / ROD_LEN

    C = np.zeros(NX)

    # model from pythonrobotics
    C[0] = - v_r * (x_t[2] * math.sin(x_t[3] - x_t[2]) * math.cos(x_t[3]) - x_t[3] * math.sin(2 * x_t[3] - x_t[2]))
    C[1] = - v_r * (x_t[2] * math.sin(x_t[3] - x_t[2]) * math.sin(x_t[3]) + x_t[3] * math.cos(2 * x_t[3] - x_t[2]))
    C[3] = (x_t[3] - x_t[2]) * (v_r * math.cos(x_t[3] - x_t[2]) - CP_OFFSET * w_r * math.sin(x_t[3] - x_t[2])) / ROD_LEN

    return A, B, C


# TODO: modify the plot_car to use x2, y2, yaw and yawt, plot tractor-trailer system in one function?
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

    plt.plot(np.array(rr_wheel[0, :]).flatten(),
             np.array(rr_wheel[1, :]).flatten(), truckcolor)

    plt.plot(np.array(rl_wheel[0, :]).flatten(),
             np.array(rl_wheel[1, :]).flatten(), truckcolor)
    plt.plot(x, y, "*")


def update_state(state, v, dyaw):
    # input check
    if dyaw >= MAX_OMEGA:
        dyaw = MAX_OMEGA
    elif dyaw <= -MAX_OMEGA:
        dyaw = -MAX_OMEGA

    # model from paper
    state_new = State(x=state.x, y=state.y, yaw=state.yaw, yawt=state.yawt)

    state_new.x = state.x + v * math.cos(state.yaw - state.yawt) * math.cos(state.yawt) * DT
    state_new.y = state.y + v * math.cos(state.yaw - state.yawt) * math.sin(state.yawt) * DT
    state_new.yaw = state.yaw + dyaw * DT
    state_new.yawt = state.yawt + v / ROD_LEN * math.sin(state.yaw - state.yawt) * DT \
                     - CP_OFFSET * dyaw * math.cos(state.yaw - state.yawt) / ROD_LEN * DT
    return state_new


def get_nparray_from_matrix(x):
    return np.array(x).flatten()


def calc_nearest_index(state, cx, cy, cyaw, pind):
    dx = [state.x - icx for icx in cx[pind:(pind + N_IND_SEARCH)]]
    dy = [state.y - icy for icy in cy[pind:(pind + N_IND_SEARCH)]]

    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

    mind = min(d)

    ind = d.index(mind) + pind

    mind = math.sqrt(mind)

    dxl = cx[ind] - state.x
    dyl = cy[ind] - state.y

    angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
    if angle < 0:
        mind *= -1

    return ind, mind


def predict_motion(x0, ov, odyaw, xref):
    xbar = xref * 0.0
    # print("x_bar size, ", xbar.shape, "x_ref size, ", xref.shape)
    for i, _ in enumerate(x0):
        xbar[i, 0] = x0[i]

    state = State(x=x0[0], y=x0[1], yaw=x0[2], yawt=x0[3])
    for (vi, dyawi, i) in zip(ov, odyaw, range(1, T + 1)):
        state = update_state(state, vi, dyawi)
        xbar[0, i] = state.x
        xbar[1, i] = state.y
        xbar[2, i] = state.yaw
        xbar[3, i] = state.yawt

    return xbar


def predict_motion_mpc(x, u):
    x1 = x
    x1[0] = x[0] + u[0] * cos(x[2] - x[3]) * cos(x[3]) * DT
    x1[1] = x[1] + u[0] * cos(x[2] - x[3]) * sin(x[3]) * DT
    x1[2] = x[2] + u[1] * DT
    x1[3] = x[3] + u[0] / ROD_LEN * sin(x[2] - x[3]) * DT - CP_OFFSET * u[1] * cos(
        x[2] - x[3]) / ROD_LEN * DT
    return x1


def iterative_linear_mpc_control(xref, x0, ov, odyaw):
    """
    MPC contorl with updating operational point iteraitvely
    """
    # print("xref, ", xref, "x0, ", x0)
    if ov is None or odyaw is None:
        ov = [0.0] * T
        odyaw = [0.0] * T

    for i in range(MAX_ITER):
        pov, podyaw = ov[:], odyaw[:]
        ov, odyaw, ox, oy, oyaw, oyawt = linear_mpc_control(xref, x0)
        # print("ov_size: ", ov.shape)
        du = sum(abs(ov - pov)) + sum(abs(odyaw - podyaw))  # calc u change value
        # print("U change value: ", sum(abs(ov - pov)), sum(abs(odyaw - podyaw)))
        if du <= DU_TH:  # iteration finish param
            break
    else:
        print("Iterative is max iter")

    return ov, odyaw, ox, oy, oyaw, oyawt


# TODO: check control model
def linear_mpc_control(xref, x0):
    """
    linear mpc control

    xref: reference point
    xbar: operational point
    x0: initial state
    dref: reference steer angle
    """

    global O_D

    opti = casadi.Opti()

    x = opti.variable(NX, T + 1)
    u = opti.variable(NU, T)
    y = opti.variable(NX, 1000)
    p = opti.parameter(NX, T + 1)
    opti.set_value(p, xref)
    obj = 0

    stage_cost = 0

    # for t in range(T):
    #     # obj += u[:, t].T @ R @ u[:, t]  # control(velocity) cost

    #     # if t < (T - 1):
    #     #     obj += (u[:, t + 1] - u[:, t]).T @ Rd @ (u[:, t + 1] - u[:, t])  # acceleration cost

    #     if t != 0:
    #         obj += (p[:, t] - x[:, t]).T @ Q @ (p[:, t] - x[:, t])  # stage cost
    #         stage_cost += (p[:, t] - x[:, t]).T @ Q @ (p[:, t] - x[:, t])

    obj += (p[:, T] - x[:, T]).T @ Qf @ (p[:, T] - x[:, T])  # terminal cost

    # obj += 0.1 * slack.T @ slack

    # A, B, C = get_linear_model_matrix(
    #     xref[:, t], xref[:, t + 1], xbar[:, t])
    t_ref = 5

    # Reachable set
    for t in range(T):
        # jackknife constraint
        opti.subject_to(fabs(x[3, t + 1] - x[2, t + 1]) <= np.deg2rad(JACKKNIFE_CON))

    # initial pose constraint
    opti.subject_to(x[:, 0] == x0)
    # opti.subject_to(fabs(u[0, :]) >= MIN_SPEED)
    opti.subject_to(fabs(u[0, :]) <= MAX_SPEED)
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
    # print("slack!!!!!!!!", oslack[0], oslack[1], oslack[2])
    BARRIER_LIST.clear()
    for j in range(len(O_X)):
        O_D[j] = (x0[0] - O_X[j]) ** 2 + (x0[1] - O_Y[j]) ** 2
        # print("O_D!", O_D[i])
    t_ref = 6
    # print("t_ref", t_ref - 1)
    x1 = ox[t_ref - 1]
    y1 = oy[t_ref - 1]
    yaw1 = oyaw[t_ref - 1]
    yawt1 = oyawt[t_ref - 1]
    x11 = ox[t_ref - 1]
    y11 = oy[t_ref - 1]
    yaw11 = oyaw[t_ref - 1]
    yawt11 = oyawt[t_ref - 1]
    x12 = ox[t_ref - 1]
    y12 = oy[t_ref - 1]
    yaw12 = oyaw[t_ref - 1]
    yawt12 = oyawt[t_ref - 1]
    oangle = -(oyaw[t_ref - 1] - oyawt[t_ref - 1])

    return ov, odyaw, ox, oy, oyaw, oyawt


def calc_ref_trajectory(state, cx, cy, cyaw, cyawt, pind):
    xref = np.zeros((NX, T + 1))
    dref = np.zeros((1, T + 1))
    ncourse = len(cx)

    ind, _ = calc_nearest_index(state, cx, cy, cyawt, pind)

    if pind >= ind:
        ind = pind

    xref[0, 0] = cx[ind]
    xref[1, 0] = cy[ind]
    # xref[2, 0] = sp[ind]
    # TODO: if the pose of tractor could be calculate in this case, when yes, define the function cal_tractor_pose
    xref[2, 0] = cyaw[ind]  # cal_tractor_pose(cyawt[ind])
    xref[3, 0] = cyawt[ind]

    for i in range(T + 1):
        # TODO: what will happen when v is not state variable and should be negative?
        if (ind + i) < ncourse:
            xref[0, i] = cx[ind + i]
            xref[1, i] = cy[ind + i]
            xref[2, i] = cyaw[ind + i]  # cal_tractor_pose(cyawt[ind + dind])
            xref[3, i] = cyawt[ind + i]
        else:
            xref[0, i] = cx[ncourse - 1]
            xref[1, i] = cy[ncourse - 1]
            # xref[2, i] = sp[ncourse - 1]
            # TODO: traget yaw position should be pre-defined!!!
            xref[2, i] = cyaw[ncourse - 1]  # cal_tractor_pose(cyawt[ncourse - 1])
            xref[3, i] = cyawt[ncourse - 1]

    return xref, ind


def check_goal(state, goal, tind, nind):
    # check goal
    dx = state.x - goal[0]
    dy = state.y - goal[1]
    d = math.hypot(dx, dy)

    # print(d, GOAL_DIS)

    isgoal = (d <= GOAL_DIS)

    if abs(tind - nind) >= 10:
        isgoal = False

    if isgoal:
        return True

    return False


def do_simulation(cx, cy, cyaw, cyawt, initial_state):
    """
    Simulation

    cx: course x position list
    cy: course y position list
    cyawt: course yaw position of the trailer list
    ck: course curvature list
    sp: speed profile
    dl: course tick [m]

    """

    goal = [cx[-1], cy[-1]]

    state = initial_state

    # initial yaw compensation
    if state.yawt - cyawt[0] >= math.pi:
        state.yawt -= math.pi * 2.0
    elif state.yawt - cyawt[0] <= -math.pi:
        state.yawt += math.pi * 2.0

    Time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    yawt = [state.yawt]
    t = [0.0]
    dyaw = [0.0]
    v = [0.0]
    diff_tar_act = []
    target_ind, _ = calc_nearest_index(state, cx, cy, cyawt, 0)

    txt_list = [state.x + math.cos(state.yawt) * ROD_LEN]
    tyt_list = [state.y + math.sin(state.yawt) * ROD_LEN]

    O_DIST = 100000000000000

    ov, odyaw = None, None

    # TODO: check the function smooth_yaw
    cyawt = smooth_yaw(cyawt)

    start = time.time()
    state_ls = [initial_state]

    while MAX_TIME >= Time:
        xref, target_ind = calc_ref_trajectory(
            state, cx, cy, cyaw, cyawt, target_ind)
        
        print("x ref: ", xref)

        x0 = [state.x, state.y, state.yaw, state.yawt]  # current state

        ov, odyaw, ox, oy, oyaw, oyawt = iterative_linear_mpc_control(
            xref, x0, ov, odyaw)

        if abs(ov[0]) <= 0.0001:
            print("WARNING!!!!!!", ov[0])

        if odyaw is not None:
            dyawi, vi = odyaw[0], ov[0]

        state = update_state(state, vi, dyawi)
        state_ls.append(state)

        if state.x - goal[0] < 0.03 and state.y - goal[1] < 0.03:
            break

        Time = Time + DT

    x.append(state.x)
    y.append(state.y)
    yaw.append(state.yaw)
    yawt.append(state.yawt)
    t.append(Time)
    dyaw.append(dyawi)
    v.append(vi)

        
    return t, x, y, yaw, yawt, dyaw, v, txt_list, tyt_list, diff_tar_act, state_ls


def calc_speed_profile(cx, cy, cyaw, target_speed):
    # Clculate the moving direction of each time step

    speed_profile = [target_speed] * len(cx)
    direction = 1.0  # forward

    # Set stop point
    for i in range(len(cx) - 1):
        dx = cx[i + 1] - cx[i]
        dy = cy[i + 1] - cy[i]

        move_direction = math.atan2(dy, dx)
        # print(cyaw[i], move_direction, dy, dx)

        if dx != 0.0 and dy != 0.0:
            dangle = abs(pi_2_pi(move_direction - cyaw[i]))
            if dangle >= math.pi / 4.0:
                direction = -1.0
            else:
                direction = 1.0

        if direction != 1.0:
            speed_profile[i] = - target_speed
        else:
            speed_profile[i] = target_speed
        # speed_profile[i] = -target_speed

    speed_profile[-1] = 0.0

    return speed_profile


def smooth_yaw(yaw):
    for i in range(len(yaw) - 1):
        dyaw = yaw[i + 1] - yaw[i]

        while dyaw >= math.pi / 2.0:
            yaw[i + 1] -= math.pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]

        while dyaw <= -math.pi / 2.0:
            yaw[i + 1] += math.pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]

    return yaw


def get_straight_course(dl):
    ax = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.0]
    ay = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)

    return cx, cy, cyaw, ck


def get_straight_course2(dl):
    ax = [0.0, -2.0, -4.0, -8.0, -10.0, -12.0, -14.0]
    ay = [0.0, -0.2, 0.2, 0.0, -0.2, 0.2, 0.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)

    return cx, cy, cyaw, ck


def get_straight_course3(dl):
    ax = [0.0, -10.0, -20.0, -40.0, -50.0, -60.0, -70.0]
    ay = [0.0, -1.0, 1.0, 0.0, -1.0, 1.0, 0.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)

    cyaw = [pi_2_pi(i - math.pi) for i in cyaw]

    return cx, cy, cyaw, ck


def get_forward_course(dl):
    ax = [0.0, 60.0, 125.0, 50.0, 75.0, 30.0, -10.0]
    ay = [0.0, 0.0, 50.0, 65.0, 30.0, 50.0, -20.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)

    return cx, cy, cyaw, ck


def get_switch_back_course(dl):
    ax = [0.0, 30.0, 6.0, 20.0, 35.0]
    ay = [0.0, 0.0, 20.0, 35.0, 20.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)
    ax = [35.0, 10.0, 0.0, 0.0]
    ay = [20.0, 30.0, 5.0, 0.0]
    cx2, cy2, cyaw2, ck2, s2 = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)
    cyaw2 = [i - math.pi for i in cyaw2]
    cx.extend(cx2)
    cy.extend(cy2)
    cyaw.extend(cyaw2)
    ck.extend(ck2)

    return cx, cy, cyaw, ck


def get_reverse_parking_course(dl):
    ax = [0.0, -1.0, -2.0, -4.0, -6.0, -6.0, -6.0]
    ay = [0.0, 0.0, 0.0, 0.0, -2.0, -4.0, -6.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)

    cyaw = [pi_2_pi(i - math.pi) for i in cyaw]

    return cx, cy, cyaw, ck


def get_circle_course_forward(r):
    t = np.linspace(0, 2 * math.pi, num=200)
    ax = [r * math.sin(i) for i in t]
    ay = [r * math.cos(i) for i in t]
    ck = np.zeros(200)

    return ax, ay, -t, ck


def get_circle_course_backward(r):
    t = np.linspace(0, 0.5 * math.pi, num=50)
    ax = [- r * math.sin(i) for i in t]
    ay = [r * math.cos(i) for i in t]
    ck = np.zeros(200)

    return ax, ay, t, ck


def main():
    print(__file__ + " start!!")

    dl = 0.05  # course tick
    # cx, cy, cyawt, ck = get_straight_course(dl)
    # cyawt = np.zeros(len(cyawt))

    # sp = calc_speed_profile(cx, cy, cyawt, TARGET_SPEED)
    # cyaw = np.copy(cyawt)

    cx = [0, 3]
    cy = [0, 2]
    cyaw = [0, 45]
    cyawt = [0, 45]


    initial_state = State(x=cx[0], y=cy[0], yaw=cyaw[0], yawt=cyawt[0])

    t, x, y, yaw, yawt, dyawt, v, txt, tyt, diff, state_list = do_simulation(
        cx, cy, cyaw, cyawt, initial_state)
    
    x_ls = [i.x for i in state_list]
    y_ls = [i.y for i in state_list]
    yaw_ls = [i.yaw for i in state_list]
    yawt_ls = [i.yawt for i in state_list]


    if show_animation:  # pragma: no cover
        plt.close("all")
        plt.subplots()
        plt.plot(x_ls, y_ls, "-r", label="motion primitive")
        # plt.plot(x, y, "-g", label="trailer")
        # plt.plot(txt, tyt, "-b", label="tractor")
        theta = np.linspace(0, 2 * np.pi, 100)
        circle_x = O_X + O_R * np.cos(theta)
        circle_y = O_Y + O_R * np.sin(theta)
        # plt.plot(circle_x, circle_y)
        # plt.plot(txm, tym, "-y", label="tracking")
        plt.grid(True)
        plt.axis("equal")
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.legend()

        # plt.subplots()
        # plt.plot(t, v, "-r", label="speed")
        # plt.grid(True)
        # plt.xlabel("Time [s]")
        # plt.ylabel("Speed [kmh]")

        # plt.subplots()
        # t = np.linspace(0, diff.shape[0], num=diff.shape[0])
        # plt.plot(t, diff.T[0], "-g", label="x_axis")
        # plt.plot(t, diff.T[1], "-b", label="y_axis")
        # plt.plot(t, [math.sqrt(i[0] ** 2 + i[1] ** 2) for i in diff], "-r", label="distance")
        # plt.grid(True)
        # plt.xlabel("Time [s]")
        # plt.ylabel("difference [m]")

        plt.show()


if __name__ == '__main__':
    main()