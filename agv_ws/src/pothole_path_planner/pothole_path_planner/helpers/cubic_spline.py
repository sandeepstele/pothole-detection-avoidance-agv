#! /usr/bin/python
# -*- coding: utf-8 -*-
u"""
Cubic Spline library on python
author Atsushi Sakai
usage: see test codes as below
license: MIT
"""
import math
import numpy as np
import bisect

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String, Byte
from rclpy.qos import QoSProfile


class Spline:
    u"""
    Cubic Spline class
    """

    def __init__(self, x, y):
        self.b, self.c, self.d, self.w = [], [], [], []

        self.x = x
        self.y = y

        self.nx = len(x)  # dimension of x
        h = np.diff(x)

        # calc coefficient c
        self.a = [iy for iy in y]

        # calc coefficient c
        A = self.__calc_A(h)
        B = self.__calc_B(h)
        self.c = np.linalg.solve(A, B)
        #  print(self.c1)

        # calc spline coefficient b and d
        for i in range(self.nx - 1):
            self.d.append((self.c[i + 1] - self.c[i]) / (3.0 * h[i]))
            tb = (self.a[i + 1] - self.a[i]) / h[i] - h[i] * \
                (self.c[i + 1] + 2.0 * self.c[i]) / 3.0
            self.b.append(tb)

    def calc(self, t):
        u"""
        Calc position
        if t is outside of the input x, return None
        """

        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None

        i = self.__search_index(t)
        dx = t - self.x[i]
        result = self.a[i] + self.b[i] * dx + \
            self.c[i] * dx ** 2.0 + self.d[i] * dx ** 3.0

        return result

    def calcd(self, t):
        u"""
        Calc first derivative
        if t is outside of the input x, return None
        """

        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None

        i = self.__search_index(t)
        dx = t - self.x[i]
        result = self.b[i] + 2.0 * self.c[i] * dx + 3.0 * self.d[i] * dx ** 2.0
        return result

    def calcdd(self, t):
        u"""
        Calc second derivative
        """

        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None

        i = self.__search_index(t)
        dx = t - self.x[i]
        result = 2.0 * self.c[i] + 6.0 * self.d[i] * dx
        return result

    def __search_index(self, x):
        u"""
        search data segment index
        """
        return bisect.bisect(self.x, x) - 1

    def __calc_A(self, h):
        u"""
        calc matrix A for spline coefficient c
        """
        A = np.zeros((self.nx, self.nx))
        A[0, 0] = 1.0
        for i in range(self.nx - 1):
            if i != (self.nx - 2):
                A[i + 1, i + 1] = 2.0 * (h[i] + h[i + 1])
            A[i + 1, i] = h[i]
            A[i, i + 1] = h[i]

        A[0, 1] = 0.0
        A[self.nx - 1, self.nx - 2] = 0.0
        A[self.nx - 1, self.nx - 1] = 1.0
        #  print(A)
        return A

    def __calc_B(self, h):
        u"""
        calc matrix B for spline coefficient c
        """
        B = np.zeros(self.nx)
        for i in range(self.nx - 2):
            B[i + 1] = 3.0 * (self.a[i + 2] - self.a[i + 1]) / \
                h[i + 1] - 3.0 * (self.a[i + 1] - self.a[i]) / h[i]
        #  print(B)
        return B


class Spline2D:
    u"""
    2D Cubic Spline class
    """

    def __init__(self, x, y):
        self.s = self.__calc_s(x, y)
        self.sx = Spline(self.s, x)
        self.sy = Spline(self.s, y)

    def __calc_s(self, x, y):
        dx = np.diff(x)
        dy = np.diff(y)
        self.ds = [math.sqrt(idx ** 2 + idy ** 2)
                   for (idx, idy) in zip(dx, dy)]
        s = [0]
        s.extend(np.cumsum(self.ds))
        return s

    def calc_position(self, s):
        u"""
        calc position
        """
        x = self.sx.calc(s)
        y = self.sy.calc(s)

        return x, y

    def calc_curvature(self, s):
        u"""
        calc curvature
        """
        dx = self.sx.calcd(s)
        ddx = self.sx.calcdd(s)
        dy = self.sy.calcd(s)
        ddy = self.sy.calcdd(s)
        k = (ddy * dx - ddx * dy) / (dx ** 2 + dy ** 2)
        return k

    def calc_yaw(self, s):
        u"""
        calc yaw
        """
        dx = self.sx.calcd(s)
        dy = self.sy.calcd(s)
        yaw = math.atan2(dy, dx)
        return yaw


def interpolate_trajectory_with_spline(x, y, ds=0.1):
    try:
        sp = Spline2D(x, y)
        s = np.arange(0, sp.s[-1], ds)

        rx, ry, ryaw, rk = [], [], [], []
        for i_s in s:
            ix, iy = sp.calc_position(i_s)
            rx.append(ix)
            ry.append(iy)
            ryaw.append(sp.calc_yaw(i_s))
            rk.append(sp.calc_curvature(i_s))

        # return rx, ry, ryaw, rk, s

        spline_msg_data = []
        for i in range(len(rx)):
            spline_msg_data.append(rx[i])
            spline_msg_data.append(ry[i])

        spline_msg_data = np.array(spline_msg_data).reshape(-1, 2)
        return spline_msg_data
    except:
        return np.array([[x[i], y[i]] for i in range(len(x))])


def interpolate_velocity(path, vel, smooth_path):
    smooth_path = np.array(smooth_path)
    smooth_vel = [vel[0]]
    indices = [0]

    x_path = path[:, 0]

    x_smooth = smooth_path[:, 0]

    i = 1
    j = 1

    while i < len(x_path) and j < len(x_smooth):
        if x_path[i] < x_smooth[j]:
            indices.append(i+j-1)
            i += 1
        else:
            j += 1
    while i < len(x_path):
        indices.append(i+j-1)
        i += 1
    while j < len(x_smooth):
        j += 1

    # self.print(f'Unsmoothed: {x_path}, Smoothed: {x_smooth}, indices: {indices}')

    slope = []
    for i in range(len(vel)-1):
        slope.append((vel[i+1]-vel[i])/(indices[i+1]-indices[i]))

    for i in range(len(slope)):
        for j in range(indices[i+1]-indices[i]):
            smooth_vel.append(slope[i]*(j+1) + vel[i])
    return np.array(smooth_vel)
