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
    
    % Define goals (mantenuto per compatibilità, ma ora usiamo trajectory)
    goal.wTog(1:4, 1:4, 1) = eye(4);
    goal.wTog(1:4, 1:4, 2) = eye(4);
    goal.wTog(1:4, 1:4, 3) = eye(4);
    goal.wTog(1:4, 1:4, 4) = eye(4);

    goal.wTog(1:3, 4, 1) = [0.70, -0.35, 0.50]';
    goal.wTog(1:3, 4, 2) = [0.5, -0.35, 0.50]';
    goal.wTog(1:3, 4, 3) = [0.50, -0.35, 0.15]';
    goal.wTog(1:3, 4, 4) = [0.70, -0.35, 0.30]';
    goal.n_goal = 4;

    % TRAJECTORY FOLLOWER: Define trajectory with timing
    % Each waypoint has a specific time associated
    goal.trajectory.n_waypoints = 4;
    goal.trajectory.poses = zeros(4, 4, goal.trajectory.n_waypoints);
    goal.trajectory.times = zeros(1, goal.trajectory.n_waypoints);
    
    % Define waypoint poses (same as goals)
    for i = 1:goal.trajectory.n_waypoints
        goal.trajectory.poses(:, :, i) = goal.wTog(:, :, i);
    end
    
    % Define timing for each waypoint (absolute times in seconds)
    % Times are chosen to give smooth motion between waypoints
    % Adjust these values based on desired speed
    goal.trajectory.times(1) = 0.0;   % Start immediately after phase 1
    goal.trajectory.times(2) = 15.0;  % Reach second waypoint after 15s
    goal.trajectory.times(3) = 30.0;  % Reach third waypoint after 30s
    goal.trajectory.times(4) = 45.0;  % Reach fourth waypoint after 45s
    
    % Total trajectory duration
    goal.trajectory.duration = goal.trajectory.times(end);
    
    % Current trajectory state
    goal.trajectory.current_segment = 1;
    goal.trajectory.completed = false;

end

