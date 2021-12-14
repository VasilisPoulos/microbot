#!/usr/bin/env python

import numpy as np
import scipy.special as sc
import matplotlib.pyplot as plt
from clothoids_math import *

class Clothoid:
    '''
    Usually a clothoid consists of:
        - <S1> -> start
        - <S2> -> finish
        - <P> -> intersection 
        - <e> -> vector_length
        - <a> -> angle
        - <list> of (x, y) points that follow the cloth.

    '''
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

    def __calculate_vector_length(self):
    # TODO: Remake properly, both vectors should have equal length.
        vector_a = self.start - self.intersection
        vector_b = self.finish - self.intersection

        vector_a_length = np.linalg.norm(vector_a)
        vector_b_length = np.linalg.norm(vector_b)
        if vector_a_length != vector_b_length:
            print('Unequal vectors')     
        return vector_a_length

    def __calculate_angle(self):     
        # more clothoid related computations here #
        return angle_between_lines(self.start, self.intersection, self.finish)

    def __calculate_characterizing_shape(self):
    # TODO: how can i make this function more readable?
        theta_c = (np.pi - self.angle) / 2
        SF, _ = sc.fresnel(np.sqrt((2 * theta_c) / np.pi))
        characterizing_shape = (np.pi / (self.max_dev**2)) * SF**2
        return characterizing_shape

    def __calculate_clothoid_starting_point(self):
        
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
        
    def __calculate_control_point(self):
    # TODO: how can i make this function more readable?
        theta_c = (np.pi - self.angle) / 2
        SF, CF = sc.fresnel(np.sqrt((2 * theta_c) / np.pi))
        temp_x = np.sqrt(np.pi / self.characterizing_shape) * CF + self.curve_start[0]
        temp_y = np.sqrt(np.pi / self.characterizing_shape) * SF
        control_point = np.array([temp_x , temp_y])
        return control_point 

    def __calculate_curve_to_point_c(self):
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

    def __calculate_line_step(self, curve_path):
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

    def __calculate_line_segment(self, curve_path):  
        line = []
        step = self.__calculate_line_step(curve_path)
        direction_on_line = np.array([])
        direction = []
        end_of_line = [self.curve_start[0] - euler_distance(self.start[:-1], self.curve_start), self.curve_start[1]]
        for x in np.linspace(0, euler_distance(self.start[:-1], end_of_line), step):
            line.append([x, 0])
            direction_on_line = np.append(direction_on_line, 0)

        direction = np.append(direction_on_line, direction)
        line = np.array(line) 
        return line, direction

    def __calculate_symmetric_segment(self, path, direction):
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

    def __calculate_clothoid(self):
        """ Calculates a clothoid curve and the direction the robot should 
        follow in this curve. 
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

    def plot(self):
        plt.style.use('seaborn-whitegrid')
        plt.subplot(1, 2, 1)
        plt.title('Clothoid Curve ')
        plt.xlabel("x-axis (m)")
        plt.ylabel("y-axis (m)")
        for item in self.curve:
            plt.scatter(item[0], item[1], c='black')
        
        x_c = self.vector_length - self.max_dev*(1 / np.tan(self.angle/2))
        y_c = self.max_dev
        plt.scatter(y_c, x_c, c='pink')
        #plt.scatter(self.start[0], self.start[1], c='green')
        plt.annotate("start", (self.start[0], self.start[1]))

        #plt.scatter(self.curve_start[0], self.curve_start[1], c='green')
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
    c1 = Clothoid(s1, p, s2, max_dev=0.05, num_of_points=15)
    print(c1)
    c1.plot()
    
if __name__ == '__main__':
    test_main()