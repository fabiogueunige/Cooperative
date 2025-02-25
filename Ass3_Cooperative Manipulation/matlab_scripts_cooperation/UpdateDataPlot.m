function [ plt ] = UpdateDataPlot( plt, pandaArm1, pandaArm2, t, loop, mission )


% this function samples the variables contained in the structure pandaArm
% and saves them in arrays inside the struct plt
% this allows to have the time history of the datauvms for later plots

% you can add whatever sampling you need to do additional plots
% plots are done in the PrintPlot.m script


plt.t(:, loop) = t;
plt.q(:, loop) = pandaArm1.q;
plt.q_dot(:, loop) = pandaArm1.q_dot;
plt.q2(:, loop) = pandaArm2.q;
plt.q_dot2(:, loop) = pandaArm2.q_dot;

% activation functions

% times at which we change phases
plt.t2 = pandaArm1.t2;
plt.t3 = pandaArm1.t3;

% Plot: desired object velocity
plt.xdot.desired(:,loop) = pandaArm1.xdot.desired; % desired object cartesian velocity

%End effector velocities (Left Arm)
plt.xdot.actual(:,loop) = pandaArm1.xdot.actual; % non-cooperative cartesian velocity

%End effector velocities (Right Arm)
plt.xdot.actual2(:,loop) = pandaArm2.xdot.actual; % non-cooperative velocity 

plt.xdot.fc(:,loop) = pandaArm1.xdot.fc.tool;

% distance between tool of arm1 and tool of arm2
plt.t1Dt2(:,loop) = pandaArm1.t1Dt2;

% Plot: manipulability task activation function

end