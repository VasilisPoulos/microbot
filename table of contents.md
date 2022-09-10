# Table of contents

1. Introduction
   - motivation (?)
     - General information about the value of testing robots in a simulated
     - environment, rapid testing in realistic scenarios
   - Previous work (Focusing on specific papers on this robot)
     - Goals (To make an accurate simulation and possibly compare results
       with the real robot)

2. Clothoids
    - Use of clothoids
    - implementation

3. Robot Model, ROS and Gazebo setup
    - Fusion 360, inertias, collision and visual models
    - Gazebo libraries, packages, launch files
    - ROS conventions, tf, naming.
    - ROS1 setup
    (in the two last sections i could explain, with
    flow diagrams, how some of the packages work as it may be
    important for one to understand what happens in the background
    eg. ros_control <https://classic.gazebosim.org/tutorials?tut=ros_control&ver=1.9>+)

    packages i used:
        ros_control
        robot_localization
        ar_track_alvar

4. Controllers
    - Explanations for each one i implement

5. Results - Future work
    - Use case, working example
    - Experiments (?)
    - No access to the real robot so we can tune the simulation.
