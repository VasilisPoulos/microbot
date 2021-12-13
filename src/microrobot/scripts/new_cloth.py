#!/usr/bin/env python

import numpy as np
import scipy.special as sc
import matplotlib.pyplot as plt
from clothoids_math import LinearEquation

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
    def __init__(self, start, intersection, finish, max_dev, \
        wheel_axis, num_of_points) -> None:
        self.start = start
        self.intersection = intersection
        self.finish = finish
        self.max_dev = max_dev
        self.wheel_axis = wheel_axis
        self.num_of_points = num_of_points
        self.vector_length = self.__calculate_vector_length()
        self.angle = self.__calculate_angle()
        self.characterizing_shape = self.__calculate_characterizing_shape()
        self.curve_start = self.__calculate_clothoid_starting_point()
        self.control_point = self.__calculate_control_point()
        self.curve = self.__calculate_clothoid_curve() # self.direction

    # TODO: Remake properly, both vectors should have equal length.
    def __calculate_vector_length(self):
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

    # TODO: how can i make this function more readable?
    def __calculate_characterizing_shape(self):
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
            print(x0)
            self.characterizing_shape = self.__calculate_characterizing_shape()

        return [x0, 0]
        
    # TODO: how can i make this function more readable?
    def __calculate_control_point(self):
        theta_c = (np.pi - self.angle) / 2
        SF, CF = sc.fresnel(np.sqrt((2 * theta_c) / np.pi))
        temp_x = np.sqrt(np.pi / self.characterizing_shape) * CF + self.curve_start[0]
        temp_y = np.sqrt(np.pi / self.characterizing_shape) * SF
        control_point = np.array([temp_x , temp_y])
        return control_point 


    
    def __calculate_clothoid_curve(self):
        """ Calculates a clothoid curve and the direction the robot should 
        follow in this curve. 
        """
        path = []
        for t in np.linspace(0, np.linalg.norm(self.start[0] - self.control_point[0]), self.num_of_points/2):
            SF, CF  = sc.fresnel(np.sqrt((2 * self.angle) / (self.wheel_axis * \
                np.pi)) * t) 
            x = np.sqrt(np.pi/self.characterizing_shape) * CF
            y = np.sqrt(np.pi/self.characterizing_shape) * SF
            
            # TODO: solve equation to find the right t.
            if(x > self.control_point[0] or y > self.control_point[1]): break

            path.append([self.curve_start[0] + x, self.curve_start[1] + y])
        
        # Calculating the rest of the curve using symmetry.
        bisector = LinearEquation(self.intersection, self.control_point)
        symmetrical_path = []
        for path_point in reversed(path):
            symmetrical_point = bisector.find_symmetrical_point_of(path_point)
            symmetrical_path.append([symmetrical_point[0], \
                symmetrical_point[1]])

        return path + symmetrical_path


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
        plt.scatter(self.start[0], self.start[1], c='green')
        plt.annotate("start", (self.start[0], self.start[1]))

        plt.scatter(self.curve_start[0], self.curve_start[1], c='green')
        plt.annotate("x0", (self.curve_start[0], self.curve_start[1]))
        
        plt.scatter(self.control_point[0], self.control_point[1], c='m')
        plt.scatter(self.intersection[0], self.intersection[1], c='blue')
        plt.scatter(self.finish[0], self.finish[1], c='red')
        plt.axis('scaled')

        plt.subplot(1, 2, 2)
        # plt.plot(range(0, len(self.direction)), self.direction, \
        #      linestyle='dashed', marker='o', c='lightcoral')
        plt.title('Direction')
        plt.xlabel("Number of clothoid point")
        plt.ylabel("Angle (deg)")
        plt.show()

def test_main():
    s1 = np.array([0., 0., 1.])
    p = np.array([0.5, 0., 1.])
    s2 = np.array([0.5, 0.5, 1.])
    c1 = Clothoid(s1, p, s2, 0.01, 0.178, 10)
    print(c1)
    c1.plot()
    
if __name__ == '__main__':
    test_main()