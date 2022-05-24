# Activity log

## Questions

- Why would i need /tf_static for always true data if thats not the case in the real world?

- Theta_c not true sometimes

- cases where user input is impossible to execute.
  - (Case 1) CLothoid exceeds trajectory finish.
    Trigger by:

    ```python
    s1 = np.array([0., 0., 1.])
    p = np.array([0.5, 0., 1.])
    s2 = np.array([0.5, 0.2, 1.])

    c1 = Clothoid(s1, p, s2, max_dev=0.1, num_of_points=10)
    ```

## Notes

Turtlebot setup link <https://automaticaddison.com/how-to-launch-the-turtlebot3-simulation-with-ros/>

- **Dec 10:** Completing cotroller: I think i can reduce spikes if i find the right values for K's. 
  I removed the round-to-stop if, now robot tries to correct it's position after a while. Maybe i 
  need to re-implement the condition.
    ![image](../microrobot/images/speed_acceleration_spikes.png)

  **Final**

    ![image](../microrobot/images/speed_final.png)

### ROS commands

Launch the virtual Turtlebot3 using RViz.

```bash
roslaunch turtlebot3_fake turtlebot3_fake.launch
```

Gazebo launch

```bash
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
```

Ploting gui

```bash
rqt_plot
```

Topics gui

```bash
rqt
```

xacro to urdf, and check urdf validity

```shell
xacro microbot.urdf.xacro -o test.urdf

check_urdf test.urdf
```

launch empty gazebo world

```bash
roslaunch gazebo_ros empty_world.launch
```

spawn model in gazebo

```shell
rosrun gazebo_ros spawn_model -file test.urdf -urdf -model test

```

### Turtlebot3 Spec

Its characteristics and specificities
<https://www.robot-advance.com/EN/actualite-turtlebot3-burger-by-robotis-149.htm>

- Maximum travel speed: 0.22 m / s.
- Maximum rotational speed: 2.84 rad / s. (167.72 deg / s.
- Maximum payload: 15 kg.
- Size (L x W x H): 138 mm x 178 mm x 192 mm.
- Maximum threshold height: 10 mm.
- Total weight (including SBC, battery and sensors): 1 kg.
- Charging time: 2h30.

## Completed

### May 14 2022

working xacro.

### March 14 2022

Began remaking urdf

![image](../microrobot/images/simple_platform_with_eccentric.png)

![image](../microrobot/images/base_geometry.png)

![image](../microrobot/images/mass_table.png)

from [ETCAE15.pdf](http://nereus.mech.ntua.gr/Documents/pdf_ps/ETCAE15.pdf).

Robot's [site](https://csl-ep.mech.ntua.gr/research-areas/microrobotics/)

![image](../microrobot/images/motor_and_needle.png)
### March 13 2022

Probably i have to use ROS2, i read many things about ros and started to understand how my ros projects should be. I tested my urdf, definitely need a remake. Read most of the useful info from the robot's papers (anything design - related).

### March 12 2022

microrobot in gazebo

![image](../microrobot/images/microrobot%20in%20gazeebo.png)

```bash
$source ../devel/setup.bash
$roslaunch microrobot gazebo.launch 
```

### March 11 2022

inertia work
![image](../microrobot/images/inertia_tensor_cylinder.png)

### March 9 2022

inertia work, eccentrers
![image](../microrobot/images/inertia.png)

### March 8 2022

![image](../microrobot/images/urdf_progress.png)

![image](../microrobot/images/geometry.png)

![image](../microrobot/images/functions.png)

### Dec 14

Kept refacotoring clothoid module, thought about test cases.

### Dec 12

Started re-building clothoids module.

### Dec 11

- [x] Fix workspace. Add setup.py and __init__.py

### Dec 10

- [x] Implement a speed limiter, controller.
- [X] Find right K's.
- [x] Plot speed acceleration etc.
- [x] Whats /tf - /tf_static difference.
  - **Answer:** \
    *source* :<http://wiki.ros.org/turtlebot3_example>
    - tf (tf2_msgs/TFMessage)
      Contains the coordinate transformation such as base_footprint and odom.
    - tf_static (tf2_msgs/TFMessage)
      Topic has same format as "/tf", however it is expected that any transform on this topic can be considered true for all time.  
    **Result:** \
    /tf is noisy, /tf_static has nothing to post.

    ![image](../microrobot/images/tf_noise_1.png)

    ![image](../microrobot/images/tf_noise.png)

### Dec 8

- [x] Convert trajectory node to a service.
