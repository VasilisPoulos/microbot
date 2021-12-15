#!/usr/bin/env python

from re import X
from typing import Tuple as tuple
import numpy as np
import scipy.special as sc
import matplotlib.pyplot as plt
from clothoids_math import *

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

    def __init__(self, start, intersection, finish, max_dev=0.1, \
        wheel_axis=0.1, num_of_points=10, reduce_step=0.40) -> None:
        self.start = start
        self.intersection = intersection
        self.finish = finish
        self.max_dev = max_dev
        self.wheel_axis = wheel_axis
        self.num_of_points = num_of_points
        self.reduce_step = 1 - reduce_step 
        self.vector_length = self.__calculate_vector_length()
        self.angle = self.__calculate_angle()
        self.characterizing_shape = self.__calculate_characterizing_shape()
        self.curve_start = self.__calculate_clothoid_starting_point()
        self.control_point = self.__calculate_control_point()
        self.path, self.direction = self.__calculate_clothoid()

    def __calculate_vector_length(self) -> float:
        vector_a = self.start - self.intersection
        vector_b = self.finish - self.intersection

        vector_a_length = np.linalg.norm(vector_a)
        vector_b_length = np.linalg.norm(vector_b)
        if vector_a_length != vector_b_length:
            print('Unequal vectors')     
        
        return vector_a_length

    def __calculate_angle(self) -> float:     
        # more clothoid related computations here #
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
            print('[Warning self.max_dev] set to {}'.format(self.max_dev))
            self.characterizing_shape = self.__calculate_characterizing_shape()

        return [x0, 0]
        
    def __calculate_control_point(self) -> list:
        theta_c = (np.pi - self.angle) / 2
        SF, CF = sc.fresnel(np.sqrt((2 * theta_c) / np.pi))
        x_c = np.sqrt(np.pi / self.characterizing_shape) * CF + self.curve_start[0]
        y_c = np.sqrt(np.pi / self.characterizing_shape) * SF
        return np.array([x_c , y_c]) 

    def __calculate_curve_to_point_c(self) -> tuple[list, list]:
    # TODO: solve equation to find the right t.
        curve_path = []
        direction = np.array([])
        for t in np.linspace(0, euler_distance(self.start[0], self.control_point[0]), self.num_of_points/2):
            SF, CF  = sc.fresnel(np.sqrt((2 * self.angle) / (self.wheel_axis * \
                np.pi)) * t) 
            x = np.sqrt(np.pi/self.characterizing_shape) * CF
            y = np.sqrt(np.pi/self.characterizing_shape) * SF
            theta = (self.angle / self.wheel_axis) * (t**2)

            if(x > self.control_point[0] or y > self.control_point[1]): break

            curve_path.append([self.curve_start[0] + x, self.curve_start[1] + y])
            direction = np.append(direction, theta)
        curve_path = np.array(curve_path)
        return curve_path, direction

    def __calculate_line_step_size(self, curve_path) -> float:
        """ Calculates the step size for the line segment before the clothoid 
        curve.

        """
        if len(curve_path) > 2:
            step = euler_distance(self.start[:-1], self.curve_start) / \
                euler_distance(self.curve_start[0], curve_path[1][0])
        else:
            step = euler_distance(self.start[:-1], self.curve_start) / \
                euler_distance(self.curve_start[0], self.intersection[0])

        step = step * self.reduce_step
        if step < 1:
            step = 1
        
        return step

    def __calculate_x_axis_line_segment(self, curve_path) -> tuple[list, list]:  
        """ Calculates the line points on the x axis before the clothoid curve.
        """

        x_axis_line = []
        direction_x_line = np.array([])
        step = self.__calculate_line_step_size(curve_path)

        # The last point of the line segment should not lay on top of the 
        # curve's first point so we offset it accordingly.
        if len(curve_path) > 1:
            end_of_line = [self.curve_start[0] - \
                euler_distance(curve_path[0][0], curve_path[1][0]), \
                    self.curve_start[1]]
        else:
            end_of_line = [self.curve_start[0] - self.max_dev, \
                self.curve_start[1]]

        for x in np.linspace(0, euler_distance(self.start[:-1], end_of_line), np.ceil(step)):
            x_axis_line.append([x, 0])
            direction_x_line = np.append(direction_x_line, 0)

        x_axis_line = np.array(x_axis_line) 
        return x_axis_line, direction_x_line

    def __calculate_y_axis_line_segment(self, curve_path) -> tuple[list, list]:
        """ Calculates the line points on the y axis before the clothoid curve.
        """

        y_axis_line = []
        direction_y_line = np.array([])
        step = self.__calculate_line_step_size(curve_path)
        bisector = LinearEquation(self.intersection, self.finish)
    
        # The first point of the line segment should not lay on top of the 
        # curve's last point so we offset it accordingly.
        if self.angle < np.pi/2: 
            sgn = -1.
        elif self.angle == np.pi/2:
            sgn = 0.
        else:
            sgn = +1.

        if len(curve_path) > 2:
            offset = sgn*euler_distance(curve_path[0][0], curve_path[1][0])
            start_of_line = \
                 [curve_path[-1][0] + offset, curve_path[-1][1] + offset]
            print(curve_path[-1])
            print(offset)
            print(start_of_line)
        else:
            offset = sgn*self.max_dev
            start_of_line = \
                [curve_path[-1][0] + offset, curve_path[-1][1] + offset]    

        if self.angle == np.pi/2:
            for y in np.linspace(start_of_line[1], self.finish[1], np.ceil(step)):
                y_axis_line.append([start_of_line[0], y])
                direction_y_line = np.append(direction_y_line, self.angle)
        else: 
            for x in np.linspace(start_of_line[0], self.finish[0], np.ceil(step)):
                y = - (bisector.a / bisector.b) * x - (bisector.c / bisector.b)
                y_axis_line.append([x, y])
                direction_y_line = np.append(direction_y_line, self.angle)
            
        return np.array(y_axis_line), direction_y_line

    def __mirror_direction(self, direction) -> list:
        #theta_c = self.angle/2
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

    def __calculate_clothoid(self) -> tuple[list, list]:
        """ Calculates the path and the direction of a trajectory that includes 
        a clothoid curve. 
        """
        left_curve_path, left_curve_direction = self.__calculate_curve_to_point_c()
        x_line, x_line_direction = self.__calculate_x_axis_line_segment(left_curve_path)  
        right_curve_half = self.__calculate_symmetric_curve_segment(left_curve_path)
        curve_path = np.vstack([left_curve_path, right_curve_half])
        
        right_direction_half =  self.__mirror_direction(left_curve_direction)
        curve_direction = np.concatenate([left_curve_direction, right_direction_half])

        y_line, y_line_direction = self.__calculate_y_axis_line_segment(curve_path)  
    
        path_wo_y_line = np.vstack([x_line, curve_path])
        complete_path = np.vstack([path_wo_y_line, y_line]) 

        direction_wo_y_line = np.append(x_line_direction, curve_direction)
        complete_direction = np.append(direction_wo_y_line, y_line_direction)

        # print('path \n{}'.format(complete_path))
        # # print('direction \n{}'.format(complete_direction))
        # print('path {}'.format(len(complete_path)))
        # print('direction {}'.format(len(complete_direction)))
        return complete_path, complete_direction

    def __str__(self) -> str:
        return 'Clothoid config:\nStart {}, {}\nInter {}, {}\nFinsh {}, {}'.\
            format(self.start[0], self.start[1], self.intersection[0], \
                self.intersection[1], self.finish[0], self.finish[1]) + \
            '\nMaxdev: {}\n#points: {}\n'.format(self.max_dev, \
            self.num_of_points) 

    def plot(self) -> None:
        plt.style.use('seaborn-whitegrid')
        plt.subplot(1, 2, 1)
        plt.title('Clothoid Curve ')
        plt.xlabel("x-axis (m)")
        plt.ylabel("y-axis (m)")
        for item in self.path:
            plt.scatter(item[0], item[1], c='black')
        
        plt.annotate("start", (self.start[0], self.start[1]))

        plt.annotate("x0", (self.curve_start[0], self.curve_start[1]))
        
        plt.scatter(self.control_point[0], self.control_point[1], c='m')
        plt.scatter(self.intersection[0], self.intersection[1], c='blue')
        plt.axis('scaled')

        plt.subplot(1, 2, 2)
        plt.plot(range(0, len(self.direction)), self.direction, \
             linestyle='dashed', marker='o', c='lightcoral')
        plt.title('Direction')
        plt.xlabel("Number of clothoid point")
        plt.ylabel("Angle (deg)")
        plt.show()

def test_main():
    s1 = np.array([0., 0., 1.])
    p = np.array([0.5, 0., 1.])
    s2 = np.array([0.5, 0.5, 1.])
    # TODO: catch 180 deg case in a function that handles the user input.
    c1 = Clothoid(s1, p, s2, max_dev=0.01, num_of_points=10)
    c1.plot()
    #print(c1.angle)
if __name__ == '__main__':
    test_main()