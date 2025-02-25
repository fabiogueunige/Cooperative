COOPERATIVE ROBOTICS ASSIGNMENT
===============================

-In this folder we have the bimanual manipulation exercise, inside the 'matlab_scripts_bimanual' folder you have the functions TO EDIT for the assignment, while the file 'main.m'is the main script in which implement the kinematic control of the robot. 
-The 'panda.mat' file contain the model of the robot - DO NOT EDIT
-In the 'matlab_scripts_bimanual/simulation_scripts' folder you have various useful functions already implemented, it is recommended NOT TO EDIT.

## Run the project

- Launch the pybullet visualizer:
```
pybullet_simulation/franka_panda_simulation.m
```
- Run the matlab main script.

NB: the left robot is the one with the base frame corresponding to the world frame wTb1 = eye(3)
NB: the right robot it the one with the traslated base w.r.t. the world frame. Keep in mind while defining the tool frame. 
