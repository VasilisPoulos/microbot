#!/usr/bin/env python

from typing import Tuple as tuple
import numpy as np
import scipy.special as sc
import matplotlib.pyplot as plt
from clothoids_math import *

class Clothoid:
    """ Calculates a path and direction for a trajectory that contains a 
    clothoid curve.
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
        self.curve, self.direction = self.__calculate_clothoid()

    def __calculate_vector_length(self) -> float:
    # TODO: Remake properly, both vectors should have equal length.
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
    # TODO: how can i make this function more readable?
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

    def __calculate_line_segment(self, curve_path) -> tuple[list, list]:  
        """ Calculates the line points before the clothoid curve.
        """

        line = []
        direction = []
        direction_on_line = np.array([])
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

        for x in np.linspace(0, euler_distance(self.start[:-1], end_of_line), step):
            line.append([x, 0])
            direction_on_line = np.append(direction_on_line, 0)

        direction = np.append(direction_on_line, direction)
        line = np.array(line) 
        return line, direction

    def __calculate_symmetric_segment(self, path, direction) -> tuple[list, list]:
        theta_c = (np.pi - self.angle)/2
        x_axis = LinearEquation([len(direction), theta_c], [len(direction)+1,  theta_c])
        mirror_x = []
        for idx, item in enumerate(direction):
                mirror_x.append(x_axis.find_symmetrical_point_of([idx, item]))
        mirror_x = np.array(mirror_x)[:,1][::-1]

        last_point = direction[-1]
        diff = theta_c - last_point

        mirror_x = mirror_x + diff
        if mirror_x[-1] > self.angle:
            diff = mirror_x[-1] - self.angle
            mirror_x = mirror_x - diff
        mirror_x = np.append(theta_c, mirror_x)
        bisector = LinearEquation(self.intersection, self.control_point)
        symmetrical_path = []
        for path_point in reversed(path):
            symmetrical_point = bisector.find_symmetrical_point_of(path_point)
            symmetrical_path.append([symmetrical_point[0], \
                symmetrical_point[1]])

        symmetrical_path = np.vstack([self.control_point, symmetrical_path])
        return symmetrical_path, mirror_x

    def __calculate_clothoid(self) -> tuple[list, list]:
        """ Calculates the path and the direction of a trajectory that includes 
        a clothoid curve. 
        """
        curve_path, curve_direction = self.__calculate_curve_to_point_c()
        line, line_direction = self.__calculate_line_segment(curve_path)
        path = np.vstack([line, curve_path]) 
        direction = np.append(line_direction, curve_direction)
        symmetrical_path, mirror_x = self.__calculate_symmetric_segment(path, direction)
        complete_path = np.vstack([path, symmetrical_path])
        complete_direction = np.append(direction, mirror_x)

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
        for item in self.curve:
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
    p = np.array([1.2, 0., 1.])
    s2 = np.array([1.3, 1., 1.])
    # TODO: catch 180 deg case in a function that handles the user input.
    c1 = Clothoid(s1, p, s2, max_dev=0.01, num_of_points=20)
    print(c1.curve)
    print(c1.direction)
    c1.plot()

    
if __name__ == '__main__':
    test_main()