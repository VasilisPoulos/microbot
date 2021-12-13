#!/usr/bin/env python

import math 
import numpy as np

def angle_trunc(a):
    while a < 0.0:
        a += np.pi * 2
    return a

def get_angle_between_points(x_orig, y_orig, x_landmark, y_landmark):
    deltaY = y_landmark - y_orig
    deltaX = x_landmark - x_orig
    return angle_trunc(math.atan2(deltaY, deltaX))

class LinearEquation:
    def __init__(self, point_a, point_b) -> None:
        self.starting_point = point_a
        self.finishing_point = point_b
        self.a, self.b, self.c \
            = self.calculate_linear_equation_coefficients()

    def calculate_linear_equation_coefficients(self):
        """Returns a, b, c coefficients a linear equation for two points."""
        a = self.starting_point[1] - self.finishing_point[1]
        b = self.finishing_point[0] - self.starting_point[0]
        c = self.starting_point[0] * self.finishing_point[1] \
            - self.starting_point[1] * self.finishing_point[0]
        return a, b, c

    def find_symmetrical_point_of(self, point_x):
        """Returns the symmetric coordinates of a point with respect to a line that 
        passes through the control point [C] and the intersection point of S1 and S2 
        [P]."""
        temp = -2 * (self.a * point_x[0] + self.b * point_x[1] + self.c) \
                / (self.a * self.a + self.b * self.b)
        x = temp * self.a + point_x[0]
        y = temp * self.b + point_x[1]
        return np.array([x, y]) 
