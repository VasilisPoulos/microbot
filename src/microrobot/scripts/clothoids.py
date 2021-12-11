from matplotlib import markers
import numpy as np
from numpy.lib.function_base import append
import scipy.special as sc
from scipy.spatial import distance
from scipy.spatial.transform import Rotation as R, rotation
import matplotlib.pyplot as plt
import math

global _MAX_DEV, _LINE_INTERVAL, _WHEELS_AXIS, _CLOTHOID_INTERVAL, _BYPASS_INTERNAL_INTERVALS, \
    _STEP_MULTIPLIER, _FINAL_DISPLAY

_MAX_DEV = 0.1 # Max trajectory deviation from control point.
_BYPASS_INTERNAL_INTERVALS = False # False for automatic step calculation for linear parts.
_LINE_INTERVAL = 10 # Number of point in linear parts. 
                    # Bypassed if _BYPASS_INTERNAL_INTERVALS is True.
_CLOTHOID_INTERVAL = 10 # Number of points in a clothoid arc.
_WHEELS_AXIS = 0.178
_STEP_MULDULATION = 10. # Used to modulate the number of points in the linear parts of the trajectory, 
                       # affects performance. High number means less points.
_FINAL_DISPLAY = 1
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
        passes through the control point [C] and the intesection point of S1 and S2 
        [P]."""
        temp = -2 * (self.a * point_x[0] + self.b * point_x[1] + self.c) \
                / (self.a * self.a + self.b * self.b)
        x = temp * self.a + point_x[0]
        y = temp * self.b + point_x[1]
        return np.array([x, y]) 


class Clothoid:
    def __init__(self, trajectory_start, intersection_point, trajectory_finish):
        
        self.trajectory_start = trajectory_start
        self.intersection_point = intersection_point
        self.trajectory_finish = trajectory_finish
        self.rotated_trajectory = False
        self.offseted_trajectory = False

        if (self.trajectory_start[0] or self.trajectory_start[1]) != 0:
            self.trajectory_offset = self.move_trajectory_to_x_axis()
            self.offseted_trajectory = True
        else:
            self.trajectory_offset = None

        if (self.intersection_point[1] != 0):
            self.rotation_offset = self.rotate_trajectory()
            self.rotated_trajectory = True
        else:
            self.rotation_offset = None
        self.start_vector = self.trajectory_start - self.intersection_point 
        self.finish_vector = self.trajectory_finish - self.intersection_point
        self.normalized_vector_length = self.calculate_vector_length()

        self.turn_over_pi_rads = False
        self.turn_angle = self.calculate_turn_angle()       
        if self.turn_angle != np.pi:
            self.clothoid_start = self.calculate_clothoid_starting_point()
            self.clothoid_finish = None # Gets calculated with trajectory calculation.
            self.characterizing_shape = self.calculate_characterizing_shape()
            self.control_point = self.calculate_control_point()
            self.clothoid_curve, self.clothoid_direction = self.calculate_clothoid_curve()
            self.clothoid_trajectory = self.calculate_clothoid_trajectory()
            self.direction = []
        else:
            self.clothoid_start = [None, None]
            self.clothoid_finish = [None, None]
            self.characterizing_shape = None
            self.control_point = [None, None]
            self.clothoid_curve = None
            self.clothoid_direction = [None]
            self.direction = [None]

            trajectory = []
            for x in np.linspace(self.trajectory_start[0], self.trajectory_finish[0], _LINE_INTERVAL*2):
                trajectory.append([x, 0])
            self.clothoid_trajectory = trajectory
        
        if self.offseted_trajectory or self.rotated_trajectory:
            self.trajectory = self.retransform_trajectory()
            self.direction = self.transform_direction()
        else:
            self.trajectory = self.clothoid_trajectory
            self.direction = self.clothoid_direction 


    def move_trajectory_to_x_axis(self):
        """ Moves the start, intersection and finsih points of a trajectory to the 
        
        Move starting point to [0, 0] then rotate the trajectory until the intersection point is
        also on x-axis."""
        offset = np.array([0, 0, 0]) - self.trajectory_start
        self.trajectory_start = self.trajectory_start + offset
        self.trajectory_finish = self.trajectory_finish + offset
        self.intersection_point = self.intersection_point + offset

        return offset


    def rotate_trajectory(self):
        self.start_vector = self.trajectory_start - self.intersection_point 

        # rotation_angle = np.degrees(math.atan2(self.start_vector[1], self.start_vector[0]))
        # TODO: atan2 cases
        rotation_angle = np.degrees(get_angle_between_points(self.start_vector[0], self.start_vector[1], 0., 0.))
        rot = R.from_euler('z', -rotation_angle, degrees=True)
        
        self.trajectory_start = rot.apply(self.trajectory_start)
        self.trajectory_finish = rot.apply(self.trajectory_finish)
        self.intersection_point = rot.apply(self.intersection_point)
        return rotation_angle


    def retransform_trajectory(self):
        rot = R.from_euler('z', self.rotation_offset, degrees=True)
        transformed_trajectory = []
        for trajectory_point in self.clothoid_trajectory:
            trajectory_point = np.append(trajectory_point, 1.)
            if self.rotated_trajectory:
                trajectory_point = rot.apply(trajectory_point)
            
            if self.offseted_trajectory: 
                trajectory_point = trajectory_point - self.trajectory_offset
            transformed_trajectory.append(trajectory_point)
        
        return transformed_trajectory


    def transform_direction(self):
        x0 = self.trajectory[0][0]
        x1 = self.trajectory[1][0]
        y0 = self.trajectory[0][1]
        y1 = self.trajectory[1][1]
        rotation_angle = np.degrees(math.atan2((y1 - y0), (x1 - x0)))

        temp_direction = []
        if self.turn_angle > np.pi:
            for item in self.clothoid_direction:
                temp_direction.append(-item)
        else:
            temp_direction = self.clothoid_direction

        return [rotation_angle + angle for angle in temp_direction]
 

    def calculate_turn_angle(self):
        cosine_of_angle = np.dot(self.start_vector, self.finish_vector) \
            / (np.linalg.norm(self.start_vector) * np.linalg.norm(self.finish_vector))
    
        turn_angle = np.arccos(cosine_of_angle)
    
        if self.trajectory_start[1] > self.trajectory_finish[1]:
            # Always need the clockwise angle.
            turn_angle = 2*np.pi - turn_angle

        if turn_angle > np.pi:
            # If we need to make a right hand side turn, the program should 
            # compute the equivalent left hand side turn and then mirror the 
            # result on the x axis.

            # Returning clothoid attributes to the 1st and 2nd quadrants.
            turn_angle = 2*np.pi - turn_angle
            self.trajectory_finish[1] = - self.trajectory_finish[1]
            self.turn_over_pi_rads = True

        return turn_angle


    def calculate_characterizing_shape(self):
        theta_c = (np.pi - self.turn_angle) / 2
        SF, _ = sc.fresnel(np.sqrt((2 * theta_c) / np.pi))
        characterizing_shape = (np.pi / (_MAX_DEV**2)) * SF**2
        return characterizing_shape


    def calculate_control_point(self):
        theta_c = (np.pi - self.turn_angle) / 2
        SF, CF = sc.fresnel(np.sqrt((2 * theta_c) / np.pi))
        temp_x = np.sqrt(np.pi / self.characterizing_shape) \
            * CF + self.clothoid_start[0]
        temp_y = np.sqrt(np.pi / self.characterizing_shape) * SF
        control_point = np.array([temp_x, temp_y])
        return control_point 


    def calculate_vector_length(self):
        start_vector_length = np.sqrt(self.start_vector[0]**2 \
            + self.start_vector[1]**2)
        finish_vector_length = np.sqrt(self.finish_vector[0]**2 \
            + self.finish_vector[1]**2)  
        # TODO: Remake properly  
        return start_vector_length


    def calculate_clothoid_starting_point(self):
        global _MAX_DEV
        while True:
            SF, CF = sc.fresnel(np.sqrt((np.pi - self.turn_angle) / np.pi))
            temp_x = self.normalized_vector_length - _MAX_DEV \
                * ((1 / np.tan(self.turn_angle / 2)) + (CF / SF))    
            x0 = np.array([temp_x, 0])
            
            if x0[0] < self.trajectory_start[0]:
                _MAX_DEV -= 0.01
                print("[WARNING!] Point x0 exceeded s1 length. _MAX_DEV set to {}"\
                    .format(_MAX_DEV))
            else:
                break
        return x0 


    def calculate_clothoid_curve(self):
        """ Calculates a clotoid curve and the direction the robot should follow in this curve. """
        global _LINE_INTERVAL, _WHEELS_AXIS
        clothoid_internal_interval = 30

        path = []
        direction = []
        while(len(path) <= _CLOTHOID_INTERVAL/2):
            path = []
            direction = []
            for t in np.linspace(0, (self.control_point[0]**2 + self.control_point[1]**2), \
                clothoid_internal_interval):
                SF, CF  = sc.fresnel(np.sqrt((2 * self.turn_angle) \
                    / (_WHEELS_AXIS * np.pi)) * t) 
                
                x = np.sqrt(np.pi/self.characterizing_shape) * CF
                y = np.sqrt(np.pi/self.characterizing_shape) * SF
                theta = (self.turn_angle / _WHEELS_AXIS) * (t ** 2)
                if (self.clothoid_start[0] + x > self.control_point[0] \
                    or self.clothoid_start[1] + y > self.control_point[1]):
                    break

                direction.append(np.degrees(theta))
                path.append([x + self.clothoid_start[0], y + self.clothoid_start[1]])
            clothoid_internal_interval += 10

        # Calculating the rest of the curve using symmetry.
        bisector = LinearEquation(self.intersection_point, self.control_point)
        symmetrical_path = []
        for path_point in reversed(path):
            symmetrical_point = bisector.find_symmetrical_point_of(path_point)
            symmetrical_path.append([symmetrical_point[0], \
                symmetrical_point[1]])

        # Calculcating the direction to the corresponding symmetrical points.
        symmetrical_directions = []
        for direction_item in np.linspace(direction[-1], np.degrees(np.pi - self.turn_angle), \
            math.floor(len(direction))):
                symmetrical_directions.append(direction_item)

        return path + symmetrical_path, direction + symmetrical_directions


    def calculate_straight_lines(self):
        
        if len(self.clothoid_curve) >= 2 and not _BYPASS_INTERNAL_INTERVALS:
            step = distance.euclidean(self.clothoid_curve[0], \
                self.clothoid_curve[1]) * _STEP_MULDULATION
        else:
            step = _LINE_INTERVAL

        s1_line = []
        for offset in np.linspace(self.trajectory_start[0], \
            self.clothoid_start[0], math.ceil(np.linalg.norm(self.start_vector) / step)):
            s1_line.append([offset, 0])
        
        s2_line = []
        bisector = LinearEquation(self.clothoid_curve[-1], self.trajectory_finish)

        if round(bisector.b, 3) != 0: # TODO: problem with last clothoid point. Change line 102.
            for x in np.linspace(self.clothoid_curve[-1][0], \
                self.trajectory_finish[0], math.ceil(np.linalg.norm(self.finish_vector) / step)):
                y = - (bisector.a / bisector.b) * x - (bisector.c / bisector.b)
                s2_line.append([x, y])
        else:
            for y in np.linspace(self.clothoid_curve[-1][1], \
                self.trajectory_finish[1], math.ceil(np.linalg.norm(self.finish_vector) / step)):
                s2_line.append([- (bisector.c / bisector.a), y])

        return s1_line, s2_line


    def calculate_clothoid_trajectory(self):
        s1_line, s2_line = self.calculate_straight_lines()

        trajectory = s1_line + self.clothoid_curve + s2_line
        bisector = LinearEquation(self.intersection_point, self.control_point)
        self.clothoid_finish = bisector.find_symmetrical_point_of(self.clothoid_start)

        # Add linear parts to direction 
        s1_direction = [self.clothoid_direction[0] for i in range(len(s1_line))]
        s2_direction = [self.clothoid_direction[-1] for i in range(len(s1_line))]
        self.clothoid_direction = s1_direction + self.clothoid_direction + s2_direction

        if self.turn_over_pi_rads:
            mirrored_trajectory = []
            x_axis = LinearEquation(self.trajectory_start, self.intersection_point)
            for trajectory_point in trajectory:
                symmetrical_point = x_axis.find_symmetrical_point_of(trajectory_point)
                mirrored_trajectory.append([symmetrical_point[0], symmetrical_point[1]])
            trajectory = mirrored_trajectory
            self.clothoid_finish = x_axis.find_symmetrical_point_of(self.clothoid_finish)
            self.control_point = x_axis.find_symmetrical_point_of(self.control_point)
            # Reverting attributes to 3rd and 4th quadrants.
            self.trajectory_finish[1] = - self.trajectory_finish[1]
            self.turn_angle = np.pi*2 - self.turn_angle

        return trajectory


    def __str__(self) -> str:
        return 'Clothoid {}\n' \
            'Trajectory start: {}\n' \
            'Trajectory finish: {}\n' \
            'Clothoid start: {}\n' \
            'Clothoid finish: {}\n' \
            'Control point: {}\n' \
            'Intersection point: {}\n' \
            'Angle: {:.2f}\n' \
            .format(id(self), self.trajectory_start, self.trajectory_finish, 
                self.clothoid_start, self.clothoid_finish, self.control_point, 
                self.intersection_point, np.degrees(self.turn_angle))


    def plot(self):
        plt.style.use('seaborn-whitegrid')
        plt.subplot(2, 2, 1)
        plt.title('Final Trajectory ')
        plt.xlabel("x-axis in cm")
        plt.ylabel("y-axis in cm")
        for item in self.trajectory:
            plt.scatter(item[0], item[1], c='black')

        plt.axis('scaled')


        plt.subplot(2, 2, 2)
        plt.plot(range(0, len(self.direction)), self.direction, \
             linestyle='dashed', marker='o')
        plt.title('Final Robot direction')
        plt.xlabel("Number of clothoid point")
        plt.ylabel("Angle in Deg")


        plt.subplot(2, 2, 3)
        plt.title('Calculated Clothoid')
        plt.xlabel("x-axis in cm")
        plt.ylabel("y-axis in cm")
        plt.plot([self.trajectory_start[0], self.intersection_point[0]], \
            [self.trajectory_start[1], self.intersection_point[1]])
        plt.plot([self.trajectory_finish[0], self.intersection_point[0]], \
            [self.trajectory_finish[1], self.intersection_point[1]])
        
        for item in self.clothoid_trajectory:
            plt.scatter(item[0], item[1], c='black')

        plt.scatter(self.trajectory_start[0], self.trajectory_start[1], c='black')
        plt.scatter(self.trajectory_finish[0], self.trajectory_finish[1], c='black')
        
        plt.annotate("s1", (self.trajectory_start[0], self.trajectory_start[1]))
        plt.annotate("s2", (self.trajectory_finish[0], self.trajectory_finish[1]))
        plt.annotate("P", (self.intersection_point[0], self.intersection_point[1]))

        if self.clothoid_start[0] != None: # In case None, clothoid arc was not calculated.
            plt.scatter(self.control_point[0], self.control_point[1], c='red')
            plt.scatter(self.intersection_point[0], self.intersection_point[1], c='blue')
            plt.scatter(self.clothoid_start[0], self.clothoid_start[1], c='green')
            plt.scatter(self.clothoid_finish[0], self.clothoid_finish[1], c='green') 

            plt.annotate("C", (self.control_point[0], self.control_point[1]), c='red')
            
        plt.axis('scaled')

        
        plt.subplot(2, 2, 4)
        plt.title('Calculated Robot direction')
        plt.plot(range(0, len(self.clothoid_direction)), self.clothoid_direction, \
             linestyle='dashed', marker='o')
        plt.xlabel("Number of clothoid point")
        plt.ylabel("Angle in Deg")

        plt.suptitle('Trajectory id {}'.format(id(self)), fontsize=14, weight='bold')
        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        
        plt.show()


def clothoid_trajectory(points_list):
    clothoid_list = []
    for index in range(len(points_list)//3 + 1):
        clothoid_list.append(Clothoid(points_list[0 + index], points_list[1 + index], \
            points_list[2 + index]))

    final_trajectory = []
    final_direction = []
    for clothoid in clothoid_list:
        final_trajectory.extend(clothoid.trajectory)
        final_direction += clothoid.direction
        # clothoid.plot()


    if _FINAL_DISPLAY:
        plt.style.use('seaborn-whitegrid')
        plt.subplot(2, 2, (1,2))
        plt.title('Final Trajectory')
        plt.xlabel("x-axis in cm")
        plt.ylabel("y-axis in cm")
        for point in final_trajectory:
            plt.scatter(point[0], point[1], c='black')
        plt.axis('scaled')

        plt.subplot(2, 2, (3,4))
        plt.title('Robot\'s Direction')
        plt.xlabel("Point number")
        plt.ylabel("Angle in Deg")
        plt.plot(range(0, len(final_direction)), final_direction, \
                linestyle='dashed', marker='o')
    
        plt.suptitle('Calculated Trajectory', fontsize=14, weight='bold')
        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        plt.show()

    return final_trajectory, final_direction
    

def main():
    trajecory_points = [np.array([0., 0., 0.]), 
                        np.array([-5., -6., 1.]),
                        np.array([-7., -15., 1.])]

    _, _ = clothoid_trajectory(trajecory_points) # TODO: break into two functions (calculate - plot)

    c1 = Clothoid(trajecory_points[1], trajecory_points[2], trajecory_points[3])
    # print(c1)
    c1.plot()
    # c2 = Clothoid(trajecory_points[2], trajecory_points[3], trajecory_points[4])
    # print(c2)
    # c2.plot()
    # c3 = Clothoid(trajecory_points[4], trajecory_points[5], trajecory_points[6])
    # print(c3)
    # c3.plot()

    # trajectory = c1.trajectory + c2.trajectory + c3.trajectory

    for point in c1.trajectory:
        plt.scatter(point[0], point[1], c='black')
    plt.axis('scaled')
    plt.show()

if __name__ == '__main__':
    main()