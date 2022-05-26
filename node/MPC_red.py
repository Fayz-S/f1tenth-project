#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math
from casadi import *
import rospkg
import pandas as pd

accelerate = 0
steering_angle_velocity = 0


class dynamic_bicycle_model():

    def __init__(self, wheelbase, friction_coeff, h_cg, l_f, l_r, cs_f, cs_r, mass, Iz, Cm1, Cm2, Cm3, B_f, C_f, D_f,
                 B_r, C_r, D_r, max_accel, max_decel, max_speed, max_steering_vel, max_steering_angle):
        self.wheelbase = wheelbase
        self.friction_coeff = friction_coeff
        self.h_cg = h_cg
        self.l_f = l_f
        self.l_r = l_r
        self.cs_f = cs_f
        self.cs_r = cs_r
        self.mass = mass
        self.Iz = Iz
        self.Cm1 = Cm1
        self.Cm2 = Cm2
        self.Cm3 = Cm3
        self.B_f = B_f
        self.C_f = C_f
        self.D_f = D_f
        self.B_r = B_r
        self.C_r = C_r
        self.D_r = D_r
        self.max_accel = max_accel
        self.max_decel = max_decel
        self.max_speed = max_speed
        self.max_steering_vel = max_steering_vel
        self.max_steering_angle = max_steering_angle

    def update_model_dynamic(self, start_state, velocity, steering_angle, dt):
        thresh = 0.5
        error = 0.03
        if not start_state[-1]:
            thresh += error

        if start_state[3] < thresh:
            return self.update_model_kinematic(start_state, velocity, steering_angle, dt)

        self.compute_accel(velocity, start_state[3])
        self.compute_steer_vel(steering_angle, start_state[5])

        d = accelerate / 7.51
        Fx_fr = self.Cm1 * d / self.Cm2 - start_state[3] / self.Cm3

        alpha_f = -math.atan((start_state[6] * self.l_f + start_state[4]) / start_state[3]) + start_state[5]
        alpha_r = math.atan((start_state[6] * self.l_r - start_state[4]) / start_state[3])

        Fy_f = self.D_f * math.sin(self.C_f * math.atan(self.B_f * alpha_f)) * 25
        Fy_r = self.D_r * math.sin(self.C_r * math.atan(self.B_r * alpha_r)) * 25

        x_dot = start_state[3] * math.cos(start_state[2]) - start_state[4] * math.sin(start_state[2])
        y_dot = start_state[3] * math.sin(start_state[2]) + start_state[4] * math.sin(start_state[2])
        theta_dot = start_state[6]
        velocity_x_dot = (Fx_fr * math.cos(start_state[5]) - Fy_f * math.sin(start_state[5]) + self.mass * start_state[
            4] * start_state[6]) / self.mass
        velocity_y_dot = (Fy_r + Fy_f * math.sin(start_state[2]) + Fx_fr * math.sin(start_state[2]) - self.mass *
                          start_state[3] * start_state[6]) / self.mass
        angular_velocity_dot = (Fy_f * self.l_f * math.cos(start_state[2]) - Fy_r * self.l_r) / self.Iz

        # [0: x, 1: y, 2: theta, 3: velocity x, 4: velocity y, 5: steering_angle, 6: angular_velocity, 7: slip_angle]
        end_state = []
        end_state.append(start_state[0] + x_dot * dt)
        end_state.append(start_state[1] + y_dot * dt)
        end_state.append(start_state[2] + theta_dot * dt)
        end_state.append(start_state[3] + velocity_x_dot * dt)
        end_state.append(start_state[4] + velocity_y_dot * dt)
        end_state.append(start_state[5] + steering_angle_velocity * dt)
        if abs(start_state[5]) < 0.005 and abs(start_state[6] + angular_velocity_dot * dt) < 1:
            end_state.append(0)
        else:
            end_state.append(min(max(-2.0, start_state[6] + dt * angular_velocity_dot), 2.0))
        end_state.append(alpha_f)
        end_state.append(1)

        return end_state

    def update_model_kinematic(self, start_state, velocity, steering_angle, dt):
        self.compute_accel(velocity, start_state[3])
        self.compute_steer_vel(steering_angle, start_state[5])

        x_dot = start_state[3] * math.cos(start_state[2])
        y_dot = start_state[3] * math.sin(start_state[2])
        theta_dot = start_state[3] / self.wheelbase * math.tan(start_state[5])
        v_dot = accelerate
        steer_angle_dot = steering_angle_velocity

        end_state = []
        end_state.append(start_state[0] + x_dot * dt)
        end_state.append(start_state[1] + y_dot * dt)
        end_state.append(start_state[2] + theta_dot * dt)
        end_state.append(min(max(start_state[3] + v_dot * dt, -2.0), 2.0))
        end_state.append(0)
        end_state.append(start_state[5] + steer_angle_dot * dt)
        end_state.append(0)
        end_state.append(0)
        end_state.append(0)

        return end_state

    def compute_accel(self, desired_velocity, current_velocity):
        dif = desired_velocity - current_velocity

        global accelerate

        if current_velocity > 0:
            if dif > 0:
                kp = 2.0 * self.max_accel / self.max_speed
                accelerate = min(max(kp * dif, -self.max_accel), self.max_accel)
            else:
                accelerate = -self.max_decel

        elif current_velocity < 0:
            if dif > 0:
                accelerate = self.max_decel
            else:
                kp = 2.0 * self.max_accel / self.max_speed
                accelerate = min(max(kp * dif, -self.max_accel), self.max_accel)

        else:
            kp = 2.0 * self.max_accel / self.max_speed
            accelerate = min(max(kp * dif, -self.max_accel), self.max_accel)

    def compute_steer_vel(self, desired_angle, current_steering_angle):
        global steering_angle_velocity

        dif = (desired_angle - current_steering_angle)

        if abs(dif) > .0001:
            steering_angle_velocity = dif / abs(dif) * self.max_steering_vel
        else:
            steering_angle_velocity = 0


class NFTOCPNLP(object):

    def __init__(self, N, Q, R, Qf, goal, bx, bu, NonLinearBicycleModel):
        # Define variables
        self.N = N
        self.n = Q.shape[1]
        self.d = R.shape[1]
        self.bx = bx
        self.bu = bu
        self.Q = Q
        self.Qf = Qf
        self.R = R
        self.goal = goal
        self.dt = dt
        self.optCost = np.inf
        self.bx = bx
        self.bu = bu
        self.NonLinearBicycleModel = NonLinearBicycleModel
        self.buildFTOCP()
        self.solverTime = []

    def solve(self, x0, verbose=False):
        # Set initial condition + state and input box constraints
        self.lbx = x0.tolist() + (-self.bx).tolist() * (self.N) + (-self.bu).tolist() * self.N
        self.ubx = x0.tolist() + (self.bx).tolist() * (self.N) + (self.bu).tolist() * self.N

        # Solve the NLP
        start = rospy.get_time()
        sol = self.solver(lbx=self.lbx, ubx=self.ubx, lbg=self.lbg_dyanmics, ubg=self.ubg_dyanmics)
        end = rospy.get_time()
        delta = end - start
        self.solverTime.append(delta)

        # Check if the solution is feasible
        if (self.solver.stats()['success']):
            self.feasible = 1
            x = sol["x"]
            self.qcost = sol["f"]
            self.xPred = np.array(x[0:(self.N + 1) * self.n].reshape((self.n, self.N + 1))).T
            self.uPred = np.array(
                x[(self.N + 1) * self.n:((self.N + 1) * self.n + self.d * self.N)].reshape((self.d, self.N))).T
            self.mpcInput = self.uPred[0][0]

            print("xPredicted:")
            print(self.xPred)
            print("uPredicted:")
            print(self.uPred)
            print("Cost:")
            print(self.qcost)

            print("NLP Solver Time: ", delta, " seconds.")
        else:
            self.xPred = np.zeros((self.N + 1, self.n))
            self.uPred = np.zeros((self.N, self.d))
            self.mpcInput = []
            self.feasible = 0
            print("Unfeasible")

        return self.uPred[0]

    def buildFTOCP(self):

        n = self.n
        d = self.d

        # Define variables
        X = SX.sym('X', n * (self.N + 1))
        U = SX.sym('U', d * self.N)

        # Define dynamic constraints
        self.constraint = []

        for i in range(0, self.N):
            X_next = self.NonLinearBicycleModel.update_model_dynamic(X[n * i:n * (i + 1)], U[d * i:d * (i + 1)], dt)[0:-1]
            for j in range(0, self.n):
                self.constraint = vertcat(self.constraint, X_next[j] - X[n * (i + 1) + j])

        self.cost = 0
        for i in range(0, self.N):
            self.cost = self.cost + (X[n * i:n * (i + 1)] - self.goal).T @ self.Q @ (X[n * i:n * (i + 1)] - self.goal)
            self.cost = self.cost + U[d * i:d * (i + 1)].T @ self.R @ U[d * i:d * (i + 1)]

        self.cost = self.cost + (X[n * self.N:n * (self.N + 1)] - self.goal).T @ self.Qf @ (
                    X[n * self.N:n * (self.N + 1)] - self.goal)

        # Standard NLP
        opts = {"verbose": False, "ipopt.print_level": 0, "print_time": 0}
        nlp = {'x': vertcat(X, U), 'f': self.cost, 'g': self.constraint}
        self.solver = nlpsol('solver', 'ipopt', nlp, opts)

        # Set lower bound of inequality constraint to zero to force n*N state dynamics
        self.lbg_dyanmics = [0] * (n * self.N)
        self.ubg_dyanmics = [0] * (n * self.N)

# [0: x, 1: y, 2: theta, 3: velocity x, 4: velocity y, 5: steering_angle, 6: angular_velocity, 7: slip_angle]
x_cl_nlp_dy = np.zeros(8)
LiDAR_distance_min = None
carState_topic = ""
scan_topic_red = ""
MPC_drive_topic = ""

# rate = rospy.Rate(10)

def carState_callback(data):
    global x_cl_nlp_dy
    x_cl_nlp_dy = np.asarray(data.data.split(","), dtype=float)



def LiDAR_callback(data):
    global LiDAR_distance_min
    LiDAR_distance_min = data.ranges


if __name__ == '__main__':
    # anonymous=True flag means that rospy will choose a unique name for our 'listener' node so that multiple
    # listeners can run simultaneously.
    rospy.init_node("MPC_red", anonymous=True)
    # need to add ~ before key name
    MPC_drive_topic = rospy.get_param("~MPC_drive_topic")
    scan_topic_red = rospy.get_param("~scan_topic_red")
    carState_topic = rospy.get_param("~carState_topic_red")

    wheelbase = rospy.get_param("~wheelbase")
    friction_coeff = rospy.get_param("~friction_coeff")
    h_cg = rospy.get_param("~height_cg")
    l_f = rospy.get_param("~l_cg2front")
    l_r = rospy.get_param("~l_cg2rear")
    cs_f = rospy.get_param("~C_S_front")
    cs_r = rospy.get_param("~C_S_rear")
    mass = rospy.get_param("~mass")
    Iz = rospy.get_param("~moment_inertia")
    Cm1 = rospy.get_param("~empirical_drivetrain_parameters_1")
    Cm2 = rospy.get_param("~empirical_drivetrain_parameters_2")
    Cm3 = rospy.get_param("~empirical_drivetrain_parameters_3")
    B_f = rospy.get_param("~empirical_Pacejka_parameters_B_f")
    C_f = rospy.get_param("~empirical_Pacejka_parameters_C_f")
    D_f = rospy.get_param("~empirical_Pacejka_parameters_D_f")
    B_r = rospy.get_param("~empirical_Pacejka_parameters_B_r")
    C_r = rospy.get_param("~empirical_Pacejka_parameters_C_r")
    D_r = rospy.get_param("~empirical_Pacejka_parameters_D_r")
    max_accel = rospy.get_param("~max_accel")
    max_decel = rospy.get_param("~max_decel")
    max_speed = rospy.get_param("~max_speed")
    max_steering_vel = rospy.get_param("~max_steering_vel")
    max_steering_angle = rospy.get_param("~max_steering_angle")
    map_name = rospy.get_param("~map_name")
    map_topic = rospy.get_param("~map_topic")

    model = dynamic_bicycle_model(wheelbase, friction_coeff, h_cg, l_f, l_r, cs_f, cs_r, mass, Iz, Cm1, Cm2, Cm3, B_f,
                                  C_f, D_f,
                                  B_r, C_r, D_r, max_accel, max_decel, max_speed, max_steering_vel, max_steering_angle)

    # https://wiki.ros.org/Packages#Client_Library_Support
    # get an instance of RosPack with the default search paths
    rospack = rospkg.RosPack()
    # list all packages, equivalent to rospack list
    rospack.list()

    reference_line_raw = pd.read_csv(rospack.get_path("f1tenth_simulator") + "/maps/" + map_name + "_minTime.csv", sep=";")

    map_msg = OccupancyGrid
    map_ptr = rospy.wait_for_message(map_topic, OccupancyGrid)
    if map_ptr is not None:
        map_msg = map_ptr
    map_width = map_msg.info.width
    map_height = map_msg.info.height
    map_origin_x = map_msg.info.origin.position.x
    map_origin_y = map_msg.info.origin.position.y
    map_resolution = map_msg.info.resolution



    rospy.Subscriber(carState_topic, String, carState_callback)
    rospy.Subscriber(scan_topic_red, LaserScan, LiDAR_callback)
    drive_pub_red = rospy.Publisher(MPC_drive_topic, AckermannDriveStamped, queue_size=10)

    ## Parameters initialization
    N = 20  # 20
    # number of states of the model
    n_dy = 8
    # number of controlling command
    d = 2


    dt = 0.1
    # sys_dy = systemdy(x0_dy, dt)
    maxTime = 40
    # xRef = np.array([10, 10, 0, np.pi/2])
    # initial state
    x0_dy = x_cl_nlp_dy
    current_x = (x0_dy[0]+map_origin_x)/(5*map_resolution)
    current_y = (-1*x0_dy[1]+map_origin_y)/(5*map_resolution)

    size = reference_line_raw.shape[0]
    start = 0
    end = size - 1
    while start < end:
        mid = int((start + end)/2)
        distance_to_start = pow(reference_line_raw.iloc[start, 1] - current_x, 2) + pow(reference_line_raw.iloc[start, 2] - current_y, 2)
        distance_to_end = pow(reference_line_raw.iloc[end, 1] - current_x, 2) + pow(reference_line_raw.iloc[end, 2] - current_y, 2)
        if distance_to_start < distance_to_end:
            end = mid - 1
        else:
            start = mid

    goal_index = (start+N)%size
    goal_x = reference_line_raw.iloc[goal_index, 1] * 5 * map_resolution - map_origin_x
    goal_y = -(reference_line_raw.iloc[goal_index, 2] * 5 * map_resolution - map_origin_y)
    goal_theta = -(reference_line_raw.iloc[goal_index, 3] + math.pi / 2)
    # goals
    xRef_dy = np.array([goal_x, goal_y, goal_theta, 10, 0, 0, 0, 0])

    # for loss function
    R = 1 * np.eye(d)
    Q_dy = 1 * np.eye(n_dy)
    # Qf_dy = 1000*np.eye(n_dy)
    Qf_dy = np.diag([100.8, 100.0, 50.0, 280.0, 0.0, 0.0, 0.0, 0.0])

    rospy.loginfo(min(LiDAR_distance_min))
    # car state constrains
    bx_dy = np.array([1000, 15, 4, 10, 5, 1, 1, 1])
    #input constrains
    bu = np.array([max_accel, max_steering_angle])

    ## Solving the problem
    nlp_dy = NFTOCPNLP(N, Q_dy, R, Qf_dy, xRef_dy, bx_dy, bu, model)

    ut_dy = nlp_dy.solve(x0_dy)

    # sys_dy.reset_IC()
    xPredNLP_dy = []
    uPredNLP_dy = []
    CostSolved_dy = []
    for t in range(0, maxTime):
        # latest state
        xt_dy = x_cl_nlp_dy

        ut_dy = nlp_dy.solve(xt_dy)
        xPredNLP_dy.append(nlp_dy.xPred)
        uPredNLP_dy.append(nlp_dy.uPred)
        CostSolved_dy.append(nlp_dy.qcost)

        drive_pub_red.publish(ut_dy)

    # try:
    #     # while not rospy.is_shutdown():
    #     #     hello_str = "hello world %s" % rospy.get_time()
    #     #     rospy.loginfo(hello_str)
    #     #     pub.publish(hello_str)
    #     #     rate.sleep()
    # except rospy.ROSInterruptException:
    #     pass
    rospy.spin()
