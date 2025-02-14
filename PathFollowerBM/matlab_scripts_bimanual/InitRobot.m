function [pandaArm, goal] = InitRobot(model,wTb_left,wTb_right)
   
    %% DO NOT CHANGE FROM HERE ...
    % Init two field of the main structure pandaArm containing the two robot
    % model
    pandaArm.ArmL = model;
    pandaArm.ArmR = model;
    % Init robot basic informations (q_init, transformation matrices ...)
    pandaArm.ArmL.q = [0.0167305,-0.762614,-0.0207622,-2.34352,-0.0305686,1.53975,0.753872]';%check rigid body tree DOCUMENTATION
    pandaArm.ArmR.q = pandaArm.ArmL.q;
    pandaArm.ArmL.q_dot = [0 0 0 0 0 0 0]';
    pandaArm.ArmR.q_dot = [0 0 0 0 0 0 0]';
    pandaArm.ArmL.bTe = getTransform(pandaArm.ArmL.franka,[pandaArm.ArmL.q',0,0],'panda_link7');
    pandaArm.ArmR.bTe = getTransform(pandaArm.ArmR.franka,[pandaArm.ArmR.q',0,0],'panda_link7');
    pandaArm.ArmL.wTb = wTb_left;
    pandaArm.ArmR.wTb = wTb_right;
    pandaArm.ArmL.wTe = pandaArm.ArmL.wTb*pandaArm.ArmL.bTe;
    pandaArm.ArmR.wTe = pandaArm.ArmR.wTb*pandaArm.ArmR.bTe;
    
    % joint limits corresponding to the actual Panda by Franka arm configuration
    pandaArm.jlmin = [-2.8973;-1.7628;-2.8973;-3.0718;-2.8973;-0.0175;-2.8973];
    pandaArm.jlmax = [2.8973;1.7628;2.8973;-0.0698;2.8973;3.7525;2.8973];
    

    % Init relevance Jacobians

    pandaArm.ArmL.bJe = eye(6,7);
    pandaArm.ArmR.bJe = eye(6,7);

    pandaArm.ArmL.wJt = eye(6,7);
    pandaArm.ArmR.wJt = eye(6,7);

    pandaArm.ArmL.wJo = eye(6,7);
    pandaArm.ArmR.wJo = eye(6,7);

    pandaArm.Jrc = zeros(6,14);

    pandaArm.ArmL.Jma = zeros(6,14);
    pandaArm.ArmR.Jma = zeros(6,14);

    pandaArm.ArmL.Jjl = zeros(14);
    pandaArm.ArmR.Jjl = zeros(14);

    % Init activation functions

    % rigid constraint 
    pandaArm.A.rc = zeros(6);
    % joints limit
    pandaArm.ArmL.A.jl = zeros(14);
    pandaArm.ArmR.A.jl = zeros(14);
    % minimum altitude
    pandaArm.ArmL.A.ma = zeros(6);
    pandaArm.ArmR.A.ma = zeros(6);
    % grasping task
    pandaArm.ArmL.A.tool = eye(6);
    pandaArm.ArmR.A.tool = eye(6);
    % move_object
    pandaArm.ArmL.A.target = zeros(6);
    pandaArm.ArmR.A.target = zeros(6);
    % stop 
    pandaArm.ArmL.A.stop = zeros(6);
    pandaArm.ArmR.A.stop = zeros(6);    

    % Init xdot 
    pandaArm.xdot.rc = zeros(6,1);

    pandaArm.ArmL.xdot.alt = [];
    pandaArm.ArmR.xdot.alt = [];

    pandaArm.ArmL.xdot.jl = zeros(14,1);
    pandaArm.ArmR.xdot.jl = zeros(14,1);

    pandaArm.ArmL.xdot.tool = [];
    pandaArm.ArmR.xdot.tool = [];
    
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

