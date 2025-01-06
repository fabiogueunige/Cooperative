function [pandaArm] = InitRobot(model,wTb_left,wTb_right)
   
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
    pandaArm.Jjl = [];

end

