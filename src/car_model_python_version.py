#!/usr/bin/env python

import numpy as np
import math

accelerate = 0
steering_angle_velocity = 0


class dynamic_bicycle_model(object):

    def __initial__(self, wheelbase, friction_coeff, h_cg, l_f, l_r, cs_f, cs_r, mass, Iz, Cm1, Cm2, Cm3, B_f, C_f, D_f,
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
