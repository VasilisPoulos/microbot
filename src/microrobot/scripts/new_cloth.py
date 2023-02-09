#!/usr/bin/env python

from typing import Tuple as tuple
import numpy as np
import scipy.special as sc
import matplotlib.pyplot as plt
from clothoids_math import *

def calculate_trajectory(intersection, finish, max_dev=0.003, points_on_curve=6, \
    points_on_straights=4, view=True):
    '''
    Transform start, intersection (p) and finish to x-axis using affine transformations so that 
    the problem can be solved using the paper's theory.    
    '''

    # 1.1 Rotation
    start = np.array([0., 0., 1])
    angle_p = \
        - get_angle_between_points(start[0], start[1], intersection[0], intersection[1])

    # print('Angle: {}'.format(np.degrees(angle_p)))

    temp_p = [affine_rotation(angle_p, intersection)[:-2][0], 0., 1.]
    temp_finish = affine_rotation(angle_p, finish)
    
    # 1.2 Reflection 
    reflected = False
    if temp_finish[1] < 0:
        temp_finish = affine_reflection(temp_finish)
        reflected = True

    # Sanity check, see if point p is on x-axis.
    # print('Transformed Point P: {}'.format(temp_p))
    # print('Transformed Point F: {}'.format(temp_finish))

    # 2. Calculate temp clothoid curve
    c1 = Clothoid(start, temp_p, temp_finish, max_dev, points_on_curve, points_on_straights)
    # c1.plot()

    # 3. Transform path back and plot
    if reflected:
        c1.path = [affine_reflection(point) for point in c1.path]
        c1.control_point = affine_reflection(c1.control_point)  
        c1.intersection = affine_reflection(c1.intersection) 
        c1.finish = affine_reflection(c1.finish)
        c1.curve_start = affine_reflection(c1.curve_start) 

    actual_path = [affine_rotation(-angle_p, point) for point in c1.path]
    c1.control_point = affine_rotation(-angle_p, c1.control_point)  
    c1.intersection = affine_rotation(-angle_p, c1.intersection) 
    c1.finish = affine_rotation(-angle_p, c1.finish)
    c1.curve_start = affine_rotation(-angle_p, c1.curve_start) 

    c1.path = actual_path
    c1.intersection

    if view:
        c1.plot()

    return c1.path

class Clothoid:
    """ Calculates right hand turn path and direction for a trajectory that 
    contains a clothoid curve.

    This class implements the theory of the Primitives for Smoothing Mobile 
    Robot Trajectories by Sara Fleury, Philippe Soukres, Jean-Paul Laumond, 
    Raja Chatila. 
    
    Notations: Trajectory start and intersection must be on x-axis, finish 
    should be on the first or second quadrant. Angle must be between 0 < a < pi.
    Every other case is deduced by symmetry.
    """

    def __init__(self, start, intersection, finish, max_dev=0.004, \
        points_on_curve=6, points_on_straights=4, wheel_axis=0.025) -> None:
        self.start = start
        self.intersection = intersection
        self.finish = finish
        self.max_dev = max_dev
        self.wheel_axis = wheel_axis
        self.points_on_curve = points_on_curve 
        self.points_on_straights = points_on_straights
        self.vector_length = self.__calculate_vector_length()
        self.angle = self.__calculate_angle()
        self.characterizing_shape = self.__calculate_characterizing_shape()
        self.curve_start = self.__calculate_clothoid_starting_point()
        self.control_point = self.__calculate_control_point()
        self.path, self.direction = self.__calculate_clothoid()

    def __calculate_vector_length(self) -> float:
        vector_a = self.start - self.intersection
        vector_a_length = np.linalg.norm(vector_a)

        return vector_a_length

    def __calculate_angle(self) -> float:     
        return angle_between_lines(self.start, self.intersection, self.finish)

    def __calculate_characterizing_shape(self) -> float:
        theta_c = (np.pi - self.angle) / 2
        SF, _ = sc.fresnel(np.sqrt((2 * theta_c) / np.pi))
        characterizing_shape = (np.pi / (self.max_dev**2)) * SF**2
        return characterizing_shape

    def __calculate_clothoid_starting_point(self) -> list:
        
        SF, CF = sc.fresnel(np.sqrt((np.pi - self.angle) / np.pi))
        x0 = self.vector_length - self.max_dev * ((1 / np.tan(self.angle / 2)) + (CF / SF))    

        if x0 < self.start[0]:
            # set max_dev to max value
            self.max_dev = -(self.start[0] - self.vector_length)/ ((1 / np.tan(self.angle / 2)) \
            + (CF / SF))
            x0 = self.start[0]
            print('Warning: clothoid starts before starting position,' + \
                'reducing self.max_dev set to {}'.format(self.max_dev))
            self.characterizing_shape = self.__calculate_characterizing_shape()

        return [x0, 0]
        
    def __calculate_control_point(self) -> list:
        theta_c = (np.pi - self.angle) / 2
        SF, CF = sc.fresnel(np.sqrt((2 * theta_c) / np.pi))
        x_c = np.sqrt(np.pi / self.characterizing_shape) * CF + self.curve_start[0]
        y_c = np.sqrt(np.pi / self.characterizing_shape) * SF
        return np.array([x_c , y_c]) 

    def __calculate_curve_to_point_c(self) -> tuple[list, list]:
        curve_path = []
        direction = np.array([])
        theta_c = (np.pi - self.angle) / 2
     
        for theta in np.linspace(0, theta_c, int(np.ceil(self.points_on_curve/2))):
            fresnel_variable = np.sqrt(2*theta/ np.pi)
            SF, CF  = sc.fresnel(fresnel_variable) 

            x = round(np.sqrt(np.pi/self.characterizing_shape) * CF, 6)
            y = round(np.sqrt(np.pi/self.characterizing_shape) * SF, 6)
            curve_path.append([self.curve_start[0] + x, self.curve_start[1] + y])
            
            #if(x > self.control_point[0] or y > self.control_point[1]): break

            # Robot controller didn't require direction information to function, 
            # so the direction calculations provided in the paper were left to
            # be implemented on a later date.
            # theta = (self.angle / self.wheel_axis) * (theta**2)
            direction = np.append(direction, 0)

        curve_path = np.array(curve_path)
        return curve_path, direction

    def __calculate_x_axis_line_segment(self) -> tuple[list, list]:  
        """ Calculates the line points on the x axis before the clothoid curve.
        """
        x_axis_line = []
        direction_x_line = np.array([])

        # The last point of the line segment should not lay on top of the 
        # curve's first point so we offset it accordingly.
        # offset = euler_distance(self.curve_start, self.control_point)
        for x in np.linspace(0, self.curve_start[0], self.points_on_straights)[:-1]:

            x_axis_line.append([x, 0])
            direction_x_line = np.append(direction_x_line, 0)

        x_axis_line = np.array(x_axis_line) 
        return x_axis_line, direction_x_line

    def __calculate_y_axis_line_segment(self, curve_path) -> tuple[list, list]:
        """ Calculates the line points on the y axis before the clothoid curve.
        """
        y_axis_points = []
        direction_y_line = np.array([])
        s2_line = LinearEquation(self.intersection, self.finish)

        
        # clothoid_curve_last_point = curve_path[-1]
        # mid_y = (self.finish[1] - (self.finish[1] - clothoid_curve_last_point[1]) / 2) 
        # offset = euler_distance(self.curve_start, self.control_point)
        curve_path = sorted(curve_path ,key=lambda l:l[1])
        for y in np.linspace(curve_path[-1][1], self.finish[1], self.points_on_straights)[1:]:
            
            if s2_line.b != 0:
                x = - (s2_line.b / s2_line.a) * y - (s2_line.c / s2_line.a)
            else:
                # Vertical line parallel to the y-axis
                x = - (s2_line.c / s2_line.a)

            y_axis_points.append([x, y])
            direction_y_line = np.append(direction_y_line, self.angle)
        return np.array(y_axis_points), direction_y_line

    def __mirror_direction(self, direction) -> list:
        theta_c = (np.pi - self.angle)/2
        x_axis = LinearEquation([len(direction), theta_c], [len(direction)+1,  theta_c])
        mirror_x = []
        for idx, item in enumerate(direction):
                mirror_x.append(x_axis.find_symmetrical_point_of([idx, item]))
        mirror_x = np.array(mirror_x)[:,1][::-1]

        last_point = direction[-1]
        diff = theta_c - last_point

        mirror_x = mirror_x + diff
        # TODO: this is a patch, is theta_c really (pi - a)/2 ?
        if mirror_x[-1] > self.angle:

            diff = mirror_x[-1] - self.angle
            for idx, item in enumerate(mirror_x):
                temp = item - diff
                if temp >= theta_c: 
                    mirror_x[idx] = item - diff
                else:
                    mirror_x[idx] = (self.angle + theta_c) / 2
        mirrored_direction = np.append(theta_c, mirror_x)
        return mirrored_direction

    def __calculate_symmetric_curve_segment(self, path) -> tuple[list, list]:  
        bisector = LinearEquation(self.intersection, self.control_point)
        symmetrical_path = []
        for path_point in reversed(path):
            symmetrical_point = bisector.find_symmetrical_point_of(path_point)
            symmetrical_path.append([symmetrical_point[0], \
                symmetrical_point[1]])

        symmetrical_path = np.vstack([self.control_point, symmetrical_path])
        return symmetrical_path

    def __warnings(self, curve_path, complete_path):
        """ Tries to warn the user about simple calculation errors that may occur from a given 
        input. This module gives the user the freedom to experiment with different values in order 
        to be tested in the simulation. For example, some max_dev values may give a correct clothoid
        trajectory but may be impossible for the robot to follow with the current simulation 
        settings. Please review the trajectories visually before executing.
        """
        complete_path = sorted(complete_path ,key=lambda l:l[1])
        last_curve_point = curve_path[-1]
        if last_curve_point[1] > self.finish[1]:
            print('Warning: clothoid curve finish is past desired trajectory finish position.' + \
                ' Removing extra points. Try decreasing max_dev.')
            while complete_path[-1][1] > self.finish[1]:
                complete_path.pop()
        elif self.curve_start[0] < 0:
            print('Warning: clothoid curve starts before starting position.' + \
                ' Try decreasing max_dev.')
        elif complete_path[-1][1] > self.finish[1]:
            print('Warning: path exceeded finish. Removing extra points')
            while complete_path[-1][1] > self.finish[1]:
                complete_path.pop()

        return complete_path

    def __calculate_clothoid(self) -> tuple[list, list]:
        """ Calculates the path and the direction of a trajectory that includes 
        a clothoid curve. 
        """
        left_curve_path, left_curve_direction = self.__calculate_curve_to_point_c()
        x_line, x_line_direction = self.__calculate_x_axis_line_segment()  
        right_curve_half = self.__calculate_symmetric_curve_segment(left_curve_path)
        curve_path = np.vstack([left_curve_path, right_curve_half])        
        right_direction_half =  self.__mirror_direction(left_curve_direction)
        curve_direction = np.concatenate([left_curve_direction, right_direction_half])

        y_line, y_line_direction = self.__calculate_y_axis_line_segment(curve_path)  
        path_wo_y_line = np.vstack([x_line, curve_path])
        complete_path = np.vstack([path_wo_y_line, y_line]) 
        #complete_path = curve_path
        direction_wo_y_line = np.append(x_line_direction, curve_direction)
        complete_direction = np.append(direction_wo_y_line, y_line_direction)

        complete_path = self.__warnings(curve_path, complete_path)
        return complete_path, complete_direction

    def __str__(self) -> str:
        return 'Clothoid config:\nStart {}, {}\nInter {}, {}\nFinsh {}, {}'.\
            format(self.start[0], self.start[1], self.intersection[0], \
                self.intersection[1], self.finish[0], self.finish[1]) + \
            '\nMaxdev: {}\n#points: {}\n'.format(self.max_dev, \
            self.num_of_points) 

    def plot(self) -> None:
        plt.style.use('seaborn-whitegrid')
        plt.title('Clothoid Path')
        plt.xlabel("x-axis (m)")
        plt.ylabel("y-axis (m)")

        plt.rcParams['savefig.dpi'] = 500
        plt.gcf().set_size_inches(8, 8)

        # Visual limitations 
        plt.xlim([-0.11, 0.11])
        plt.ylim([-0.11, 0.11])
        
        view_boundaries = True
        if view_boundaries:
            robot_boundaries = plt.Circle((0 , 0), 0.025 , alpha=0.3, color='orange')
            camera_view_boundaries = plt.Rectangle((-0.05, -0.1), 0.1, 0.2, alpha=0.1, color='dimgray')
            plt.gca().add_artist(robot_boundaries)
            plt.gca().add_artist(camera_view_boundaries)
        
        ax = plt.gca()
        ax.set_aspect('equal', adjustable='box')

        for item in self.path:
            plt.scatter(item[0], item[1], c='black')
        
        offset = 0.001
        bisector = LinearEquation(self.intersection, self.control_point)
        curve_finish = bisector.find_symmetrical_point_of(self.curve_start)
        plt.annotate("start", (self.start[0] - offset, self.start[1] + offset))
        plt.annotate("x0", (self.curve_start[0] - offset, self.curve_start[1] + offset))
        plt.annotate("x1", (curve_finish[0] + offset/4, curve_finish[1] + offset/4))
        plt.annotate("P", (self.intersection[0] + offset/2, self.intersection[1] + offset))
        plt.annotate("finish", (self.path[-1][0] - offset, self.path[-1][1] + offset))
        plt.annotate("C", (self.control_point[0] + offset, self.control_point[1] - offset))

        plt.scatter(self.control_point[0], self.control_point[1], c='m', s=100, alpha=0.6)
        plt.scatter(self.intersection[0], self.intersection[1], c='blue', s=100, alpha=0.6)
        plt.scatter(self.start[0], self.start[1], c='red', s=100, alpha=0.6)
        plt.scatter(self.finish[0], self.finish[1], c='green', s=100, alpha=0.6)

        plt.tight_layout()
        plt.show()

def test_main():
    # s1 = np.array([0., 0., 1])
    # p = np.array([0.0, 0.05, 1.])
    # s2 = np.array([0.05, 0.05, 1.])

    # p_0 = np.array([0.05, 0.0, 1.])
    # s2_0 = np.array([0.05, 0.05, 1.])
    # calculate_trajectory(p_0, s2_0)

    # p_1 = np.array([0.0, 0.05, 1.])
    # s2_1 = np.array([-0.05, 0.05, 1.])
    # calculate_trajectory(p_1, s2_1)

    # p_1 = np.array([-0.05, 0.0, 1.])
    # s2_1 = np.array([-0.05, -0.05, 1.])
    # calculate_trajectory(p_1, s2_1)

    # p_2 = np.array([0.0, -0.05, 1.])
    # s2_2 = np.array([0.05, -0.05, 1.])
    # calculate_trajectory(p_2, s2_2)

    ####################################

    # p_0_t = np.array([0.05, 0.0, 1.])
    # s2_0_t = np.array([0.05, -0.05, 1.])
    # calculate_trajectory(p_0_t, s2_0_t)

    # p_1_t = np.array([0.0, 0.05, 1.])
    # s2_1_t = np.array([0.05, 0.05, 1.])
    # calculate_trajectory(p_1_t, s2_1_t)

    # p_1_t = np.array([-0.05, 0.0, 1.])
    # s2_1_t = np.array([-0.05, 0.05, 1.])
    # calculate_trajectory(p_1_t, s2_1_t)

    # p_2_t = np.array([0.0, -0.05, 1.])
    # s2_2_t = np.array([-0.05, -0.05, 1.])
    # calculate_trajectory(p_2_t, s2_2_t)

    #######################################

    p_2_t = np.array([0.06, -0.03, 1.])
    s2_2_t = np.array([0.06, 0.07, 1.])
    _ = calculate_trajectory(p_2_t, s2_2_t, 0.01, 9, 4)
   
if __name__ == '__main__':
    test_main()