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


% Plot: desired object velocity

%End effector velocities (Left Arm)


%End effector velocities (Right Arm)


% Plot: manipulability task activation function

end