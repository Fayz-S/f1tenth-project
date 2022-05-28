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


    def update_model(self, start_state, velocity, steering_angle, dt):
        global accelerate
        global steering_angle_velocity
        self.compute_accel(velocity, start_state[3])
        self.compute_steer_vel(steering_angle, start_state[5])

        d = accelerate / 7.51
        Fx_fr = self.Cm1 * d / self.Cm2 - start_state[3] / self.Cm3

        alpha_f = -atan2((start_state[6] * self.l_f + start_state[4]) , start_state[3]) + start_state[5]
        alpha_r = atan2((start_state[6] * self.l_r - start_state[4]) , start_state[3])

        Fy_f = self.D_f * sin(self.C_f * atan(self.B_f * alpha_f)) * 25
        Fy_r = self.D_r * sin(self.C_r * atan(self.B_r * alpha_r)) * 25

        x_dot = start_state[3] * cos(start_state[2]) - start_state[4] * sin(start_state[2])
        y_dot = start_state[3] * sin(start_state[2]) + start_state[4] * sin(start_state[2])
        theta_dot = start_state[6]
        velocity_x_dot = (Fx_fr * cos(start_state[5]) - Fy_f * sin(start_state[5]) + self.mass * start_state[4] * start_state[6]) / self.mass
        velocity_y_dot = (Fy_r + Fy_f * sin(start_state[2]) + Fx_fr * sin(start_state[2]) - self.mass *
                          start_state[3] * start_state[6]) / self.mass
        angular_velocity_dot = (Fy_f * self.l_f * cos(start_state[2]) - Fy_r * self.l_r) / self.Iz

        # [0: x, 1: y, 2: theta, 3: velocity x, 4: velocity y, 5: steering_angle, 6: angular_velocity, 7: slip_angle]

        x_next = start_state[0] + x_dot * dt
        y_next = start_state[1] + y_dot * dt
        theta_next = start_state[2] + theta_dot * dt
        vx_next = start_state[3] + velocity_x_dot * dt
        vy_next = start_state[4] + velocity_y_dot * dt
        steering_angle_next = start_state[5] + steering_angle_velocity * dt
        yaw_next = if_else(
            logic_and(fabs(start_state[5]) < 0.005, fabs(start_state[6] + angular_velocity_dot * dt) < 1), 0,
            fmin(fmax(-2.0, start_state[6] + dt * angular_velocity_dot), 2.0))

        slip_angle_next = start_state[7] * 0
        # end_state.append(1)
        end_state = [x_next, y_next, theta_next, vx_next, vy_next, steering_angle_next, yaw_next, slip_angle_next]
        return end_state

    def compute_accel(self, desired_velocity, current_velocity):
        dif = desired_velocity - current_velocity

        global accelerate
        kp = 2.0 * self.max_accel / self.max_speed
        accelerate = if_else(current_velocity > 0,
                             if_else(dif > 0, fmin(fmax(kp * dif, -self.max_accel), self.max_accel), -self.max_decel),
                             if_else(current_velocity < 0, if_else(dif > 0, self.max_decel,
                                                                   fmin(fmax(kp * dif, -self.max_accel),
                                                                        self.max_accel)),
                                     fmin(fmax(kp * dif, -self.max_accel), self.max_accel)))

    def compute_steer_vel(self, desired_angle, current_steering_angle):
        global steering_angle_velocity

        dif = (desired_angle - current_steering_angle)

        steering_angle_velocity = if_else(fabs(dif) > 0.0001, dif / fabs(dif) * self.max_steering_vel, 0)

class kinematic_bicycle_model():

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

    def update_model(self, start_state, velocity, steering_angle, dt):
        global accelerate
        global steering_angle_velocity
        self.compute_accel(velocity, start_state[3])
        # self.compute_steer_vel(steering_angle, start_state[5])

        # x_dot = start_state[3] * np.cos(start_state[2])
        # y_dot = start_state[3] * np.sin(start_state[2])
        # theta_dot = start_state[3] / self.wheelbase * np.tan(start_state[5])
        # v_dot = accelerate
        # steer_angle_dot = steering_angle_velocity

        # x_next = start_state[0] + x_dot * dt
        # y_next = start_state[1] + y_dot * dt
        # theta_next = start_state[2] + theta_dot * dt
        # vx_next = fmin(fmax(start_state[3] + v_dot * dt, -2.0), 2.0)
        # vx_next = start_state[3] + v_dot*dt
        # vy_next = start_state[4] * 0
        # steering_angle_next = start_state[5] - steering_angle
        # yaw_next = start_state[6] * 0
        # slip_angle_next = start_state[7] * 0
        # end_state.append(0), yaw_next, slip_angle_next

        start_state[3] = start_state[3] + 0.00001
        alpha_f = -np.arctan2((start_state[5] * self.l_f + start_state[4]) , start_state[3]) + steering_angle
        alpha_r = np.arctan2((start_state[5] * self.l_r - start_state[4]) , start_state[3])

        Fyf = self.D_f * np.sin(self.C_f * np.arctan(self.B_f * alpha_f))*25
        Fyr = self.D_r * np.sin(self.C_r * np.arctan(self.B_r * alpha_r))*25

        x_next = start_state[0] + dt * (start_state[3] * np.cos(start_state[2]) - start_state[4] * np.sin(start_state[2]))
        y_next = start_state[1] + dt * (start_state[3] * np.sin(start_state[2]) + start_state[4] * np.cos(start_state[2]))
        theta_next = start_state[2] + dt * start_state[5]

        vx_next = start_state[3] + dt * (accelerate - 1 / self.mass * Fyf * np.sin(steering_angle) + start_state[4] * start_state[5])
        vy_next = start_state[4] + dt * (1 / self.mass * (Fyf * np.cos(steering_angle) + Fyr) - start_state[3] * start_state[5])
        yaw_next = start_state[5] + dt * (1 / self.Iz * (self.l_f * Fyf * np.cos(steering_angle) - Fyr * self.l_r))

        end_state = [x_next, y_next, theta_next, vx_next, vy_next, yaw_next]
        return end_state

    def compute_accel(self, desired_velocity, current_velocity):
        dif = desired_velocity - current_velocity

        global accelerate
        kp = 2.0 * self.max_accel / self.max_speed
        # accelerate = if_else(current_velocity > 0,
        #                      if_else(dif > 0,
        #                              fmin(fmax(kp * dif, -self.max_accel), self.max_accel),
        #                              -self.max_decel),
        #                      if_else(current_velocity < 0,
        #                              if_else(dif > 0,
        #                                      self.max_decel,
        #                                      fmin(fmax(kp * dif, -self.max_accel), self.max_accel)),
        #                              fmin(fmax(kp * dif, -self.max_accel), self.max_accel)))
        accelerate = kp*dif

    def compute_steer_vel(self, desired_angle, current_steering_angle):
        global steering_angle_velocity

        dif = (desired_angle - current_steering_angle)

        steering_angle_velocity = if_else(fabs(dif) > 0.0001, dif / fabs(dif) * self.max_steering_vel, 0)


class NFTOCPNLP(object):

    def __init__(self, N, Q, R, Qf, goal, upx, lowx, bu, NonLinearBicycleModel):
        # Define variables
        self.N = N
        self.n = Q.shape[1]
        self.d = R.shape[1]
        self.upx = upx
        self.lowx = lowx
        self.bu = bu
        self.Q = Q
        self.Qf = Qf
        self.R = R
        self.goal = goal
        self.dt = dt
        self.optCost = np.inf
        self.NonLinearBicycleModel = NonLinearBicycleModel
        self.buildFTOCP()
        self.solverTime = []

    def solve(self, x0, verbose=False):
        # Set initial condition + state and input box constraints
        self.lbx = x0.tolist() + (self.lowx).tolist() * (self.N) + (-self.bu).tolist() * self.N
        self.ubx = x0.tolist() + (self.upx).tolist() * (self.N) + (self.bu).tolist() * self.N

        # Solve the NLP
        start = rospy.get_time()
        sol = self.solver(lbx=self.lbx, ubx=self.ubx, lbg=self.lbg_dyanmics, ubg=self.ubg_dyanmics)
        # rospy.loginfo(sol)
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

            rospy.loginfo("xPredicted:")
            rospy.loginfo(self.xPred)
            rospy.loginfo("uPredicted:")
            rospy.loginfo(self.uPred)
            # rospy.loginfo("Cost:")
            # rospy.loginfo(self.qcost)

            rospy.loginfo("NLP Solver Time: %s%s", delta, " seconds.")
        else:
            self.xPred = np.zeros((self.N + 1, self.n))
            self.uPred = np.zeros((self.N, self.d))
            self.mpcInput = []
            self.feasible = 0
            rospy.logwarn("Unfeasible")

        return self.uPred[1]

    def buildFTOCP(self):

        n = self.n
        d = self.d

        # Define variables
        X = SX.sym('X', n * (self.N + 1))
        U = SX.sym('U', d * self.N)

        # Define dynamic constraints
        self.constraint = []

        for i in range(self.N):
            X_next = self.NonLinearBicycleModel.update_model(X[n * i:n * (i + 1)],
                                                             U[d * i],
                                                             U[d * i + 1], dt)
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
LiDAR_raw = ()
carState_topic = ""
scan_topic_red = ""
MPC_drive_topic = ""


# rate = rospy.Rate(10)

def carState_callback(data):
    global x_cl_nlp_dy
    x_cl_nlp_dy = np.asarray(data.data.split(","), dtype=float)


def LiDAR_callback(data):
    global LiDAR_raw
    LiDAR_raw = data.ranges
    # rospy.loginfo(LiDAR_raw)


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
    field_of_view = rospy.get_param("~scan_field_of_view")
    scan_beams = rospy.get_param("~scan_beams")

    dynamic_model = dynamic_bicycle_model(wheelbase, friction_coeff, h_cg, l_f, l_r, cs_f, cs_r, mass, Iz, Cm1, Cm2, Cm3, B_f,
                                  C_f, D_f,
                                  B_r, C_r, D_r, max_accel, max_decel, max_speed, max_steering_vel, max_steering_angle)

    kinematic_model = kinematic_bicycle_model(wheelbase, friction_coeff, h_cg, l_f, l_r, cs_f, cs_r, mass, Iz, Cm1, Cm2, Cm3, B_f,
                              C_f, D_f,
                              B_r, C_r, D_r, max_accel, max_decel, max_speed, max_steering_vel, max_steering_angle)
    # https://wiki.ros.org/Packages#Client_Library_Support
    # get an instance of RosPack with the default search paths
    rospack = rospkg.RosPack()
    # list all packages, equivalent to rospack list
    rospack.list()

    map_msg = OccupancyGrid
    map_ptr = rospy.wait_for_message(map_topic, OccupancyGrid)
    if map_ptr is not None:
        map_msg = map_ptr
    map_width = map_msg.info.width
    map_height = map_msg.info.height
    map_origin_x = map_msg.info.origin.position.x
    map_origin_y = map_msg.info.origin.position.y
    map_resolution = map_msg.info.resolution

    reference_line_raw = pd.read_csv(rospack.get_path("f1tenth_simulator") + "/maps/" + map_name + "_minTime.csv",
                                     sep=";")

    reference_line_x_y_theta = pd.DataFrame()
    reference_line_x_y_theta['x'] = reference_line_raw.iloc[:, 1] * 5 * map_resolution - map_origin_x
    reference_line_x_y_theta['y'] = - (reference_line_raw.iloc[:, 2] * 5 * map_resolution - map_origin_y)
    reference_line_x_y_theta['theta'] = -(reference_line_raw.iloc[:, 3] + math.pi / 2)

    rospy.Subscriber(carState_topic, String, carState_callback)
    rospy.Subscriber(scan_topic_red, LaserScan, LiDAR_callback)
    drive_pub_red = rospy.Publisher(MPC_drive_topic, AckermannDriveStamped, queue_size=10)

    ## Parameters initialization
    N = 15  # 20
    # number of states of the model
    n_dy = 6
    # number of controlling command
    d = 2

    dt = 0.005
    # sys_dy = systemdy(x0_dy, dt)
    maxTime = 10
    # xRef = np.array([10, 10, 0, np.pi/2])
    # initial state
    while not rospy.is_shutdown():
        x0_dy = x_cl_nlp_dy
        current_x = x0_dy[0]
        current_y = x0_dy[1]
        current_theta = x0_dy[2]

        size = reference_line_raw.shape[0]
        # first line is same as last line
        start = 0
        end = size - 2
        while start < end:
            mid = int((start + end) / 2)
            distance_to_start = pow(reference_line_x_y_theta.iloc[start, 0] - current_x, 2) + pow(
                reference_line_x_y_theta.iloc[start, 1] - current_y, 2)
            distance_to_end = pow(reference_line_x_y_theta.iloc[end, 0] - current_x, 2) + pow(
                reference_line_x_y_theta.iloc[end, 1] - current_y, 2)
            if distance_to_start < distance_to_end:
                end = mid
            else:
                start = mid + 1

        goal_index = (start + 20) % size
        goal_x = reference_line_x_y_theta.iloc[goal_index, 0]
        goal_y = reference_line_x_y_theta.iloc[goal_index, 1]
        goal_theta = reference_line_x_y_theta.iloc[goal_index, 2]
        rospy.loginfo("goal"+str(goal_x))
        rospy.loginfo("goal"+str(goal_y))
        rospy.loginfo("goal"+str(goal_theta))
        # goals
        xRef_dy = [goal_x, goal_y, goal_theta, 20, 0, 0]

        # for loss function
        R = 1 * np.eye(d)
        Q_dy = 1 * np.eye(n_dy)
        # Qf_dy = 1000*np.eye(n_dy)
        Qf_dy = np.diag([2000.0, 2000.0, 500.0, 100.0, 0.0, 0.0])

        # current_LiDAR = ()
        # while len(current_LiDAR) == 0:
        #     current_LiDAR = LiDAR_raw
        #
        # index = -1
        # LiDAR_min_distance = 0x3F3F3F3F
        # for i in range(scan_beams):
        #     if LiDAR_min_distance > current_LiDAR[i]:
        #         LiDAR_min_distance = current_LiDAR[i]
        #         index = i
        #
        # theta_to_nearest = current_theta - field_of_view / 2 + field_of_view / scan_beams * index
        # distance_to_nearest_x = LiDAR_min_distance * math.cos(theta_to_nearest) + current_x
        # distance_to_nearest_y = LiDAR_min_distance * math.sin(theta_to_nearest) + current_y
        # rospy.loginfo(distance_to_nearest_x)
        # rospy.loginfo(distance_to_nearest_y)

        # car state constrains
        upx_dy = np.array([1000, 1000, 40, 20, 10, 10])
        lowx_dy = np.array([-1000, -1000, -30, 0, -10, -10])
        # input constrains
        bu = np.array([max_speed, max_steering_angle+0.5])

        thresh = 0.5
        ## Solving the problem
        #nlp_dynamic = NFTOCPNLP(N, Q_dy, R, Qf_dy, xRef_dy, upx_dy, lowx_dy, bu, dynamic_model)
        nlp_kinematic = NFTOCPNLP(N, Q_dy, R, Qf_dy, xRef_dy, upx_dy, lowx_dy, bu, kinematic_model)

        # ut_dy = nlp_kinematic.solve(x0_dy)

        # sys_dy.reset_IC()
        xPredNLP_dy = []
        uPredNLP_dy = []
        CostSolved_dy = []
        for t in range(0, maxTime):
            # latest state
            xt_dy = x_cl_nlp_dy[0:6]
            xt_dy[5] = x_cl_nlp_dy[6]
            # if xt_dy[3] < thresh:
            # rospy.loginfo("kinematic")
            ut_dy = nlp_kinematic.solve(xt_dy)
                # xPredNLP_dy.append(nlp_kinematic.xPred)
                # uPredNLP_dy.append(nlp_kinematic.uPred)
                # CostSolved_dy.append(nlp_kinematic.qcost)
            # else:
            #     rospy.loginfo("dynamic")
            #     ut_dy = nlp_dynamic.solve(xt_dy)
                # xPredNLP_dy.append(nlp_dynamic.xPred)
                # uPredNLP_dy.append(nlp_dynamic.uPred)
                # CostSolved_dy.append(nlp_dynamic.qcost)
            rospy.loginfo(ut_dy)
            ack_msg = AckermannDriveStamped()
            ack_msg.header.stamp = rospy.Time.now()
            ack_msg.drive.steering_angle = ut_dy[1]
            ack_msg.drive.speed = ut_dy[0]
            drive_pub_red.publish(ack_msg)

    # try:
    #     # while not rospy.is_shutdown():
    #     #     hello_str = "hello world %s" % rospy.get_time()
    #     #     rospy.loginfo(hello_str)
    #     #     pub.publish(hello_str)
    #     #     rate.sleep()
    # except rospy.ROSInterruptException:
    #     pass
    rospy.spin()
