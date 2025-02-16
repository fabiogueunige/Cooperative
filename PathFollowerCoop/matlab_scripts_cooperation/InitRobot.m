function [pandaArm, goal] = InitRobot(model,wTb)

%% DO NOT CHANGE FROM HERE ...
% Init two field of the main structure pandaArm containing the two robot
% model
pandaArm = model;
% Init robot basic informations (q_init, transformation matrices ...)
pandaArm.q = [0.0167305,-0.762614,-0.0207622,-2.34352,-0.0305686,1.53975,0.753872]';%check rigid body tree DOCUMENTATION
pandaArm.q_dot = [0 0 0 0 0 0 0]';
pandaArm.alt = 0.20;

pandaArm.bTe = getTransform(pandaArm.franka,[pandaArm.q',0,0],'panda_link7');
pandaArm.wTb = wTb;
pandaArm.wTe = pandaArm.wTb * pandaArm.bTe;

% joint limits corresponding to the actual Panda by Franka arm configuration
pandaArm.jlmin = [-2.8973; -1.7628; -2.8973; -3.0718; -2.8973; -0.0175; -2.8973];
pandaArm.jlmax = [2.8973; 1.7628; 2.8973; -0.0698; 2.8973; 3.7525; 2.8973];

% Init relevance Jacobians
pandaArm.bJe = eye(6,7);
pandaArm.Jjl = [];
pandaArm.wJo = zeros(6,7);
pandaArm.J.ma = zeros(6, 7);

%% ... TO HERE
% Init Task Reference vectors
pandaArm.xdot.tool = zeros(6,1);
pandaArm.xdot.alt = [];
pandaArm.xdot.jl = [];
% Init Activation function for activate or deactivate tasks
pandaArm.A.tool = zeros(6);
pandaArm.A.ma = zeros(6,6);
pandaArm.A.jl = zeros(7);

% Init variables for plotting
pandaArm.xdot.actual = 0;
pandaArm.xdot.desired = 0;
pandaArm.t2 = NaN; % time phase 2
pandaArm.t3 = NaN; % time phase 3
pandaArm.xdot.fc.tool = 0;

% Define goals
goal.wTog(1:4, 1:4, 1) = eye(4);
goal.wTog(1:4, 1:4, 2) = eye(4);
goal.wTog(1:4, 1:4, 3) = eye(4);
goal.wTog(1:4, 1:4, 4) = eye(4);

goal.wTog(1:3, 4, 1) = [0.70, -0.35, 0.50]';
goal.wTog(1:3, 4, 2) = [0.5, -0.35, 0.50]';
goal.wTog(1:3, 4, 3) = [0.50, -0.35, 0.30]';
goal.wTog(1:3, 4, 4) = [0.70, -0.35, 0.30]';
goal.n_goal = 4;

end

