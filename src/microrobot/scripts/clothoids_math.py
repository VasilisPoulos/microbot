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

def angle_between_lines(point_a, intersection, point_b) -> float:
    '''Finds the angle between two lines that intersect.
    
    The lines begin form point a and point b respectively and intersect at the 
    intersection point. The function converts the each line's points to vectors 
    so that we can find the cosine of that angle using the formula of the dot 
    product. Finally, in order to find the angle between two vectors, a and b, 
    we will solve with respect to the angle θ.
    '''
    # TODO: this function converts to vectors and finds an angle, maybe it 
    # could be splitted in two.
    vector_a = point_a - intersection
    vector_b = point_b - intersection
    cos_theta = np.dot(vector_a, vector_b) / (np.linalg.norm(vector_a) * \
        np.linalg.norm(vector_b))
    angle_in_rad = np.arccos(cos_theta)
    return angle_in_rad

def euler_distance(point_a, point_b) -> float:
    return np.linalg.norm(point_a - point_b)

def affine_rotation(angle, point):

    if len(point) == 3:
        rotation = np.array([[math.cos(angle), - math.sin(angle), 0],
                            [math.sin(angle),    math.cos(angle), 0],
                            [0,                  0,               1]])
    elif len(point) == 2:
        rotation = np.array([[math.cos(angle), - math.sin(angle)],
                            [math.sin(angle),    math.cos(angle)]])

    new_point = np.matmul(rotation, point)
    return new_point

def affine_reflection(point, axis=1):
    ''' -1 for y axis 1 for x
    '''

    if len(point) == 3:
        reflection = np.array([ [axis, 0, 0],
                                [0,-axis, 0],
                                [0,    0, 1]])
    elif len(point) == 2:
        reflection = np.array([ [axis, 0],
                                [0,-axis]])

    new_point = np.matmul(reflection, point)
    return new_point

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

