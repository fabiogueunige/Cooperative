COOPERATIVE ROBOTICS ASSIGNMENT
===============================

-In this folder we have a task priority algorithm to solve a bimanual manipulation task. We have implemented a LOS algorithm to make the robots follow a path whitout exceeding joint limits or going down the minimum safety altitude. 
-Inside the 'matlab_scripts_bimanual' folder there is the code. 
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
