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
        
    %% TRAJECTORY SELECTION: Choose trajectory type
    trajectory_option = 'B';  % 'A' or 'B'
    
    if strcmp(trajectory_option, 'A')
        %% OPTION A: CIRCULAR TRAJECTORY
        % Create waypoints along a circle in the horizontal plane
        
        n_points = 8;  % 8 waypoint lungo il cerchio
        goal.trajectory.n_waypoints = n_points;
        goal.trajectory.poses = zeros(4, 4, n_points);
        goal.trajectory.times = zeros(1, n_points);
        
        % Parametri del cerchio
        % Centro del cerchio nello spazio di lavoro
        center = [0.5, -0.2, 0.4]';  % Centro a (x=0.5, y=-0.2, z=0.4)
        radius = 0.15;  % Raggio di 15 cm
        total_time = 40.0;  % 40 secondi per completare il cerchio
        
        % Crea i waypoint lungo il cerchio
        for i = 1:n_points
            % Angolo per questo waypoint (distribuiti uniformemente sul cerchio)
            theta = 2*pi*(i-1)/(n_points-1);
            
            % Posizione sul cerchio (cerchio nel piano XY)
            x = center(1) + radius*cos(theta);
            y = center(2) + radius*sin(theta);
            z = center(3);  % Altezza costante
            
            % Creare la posa (identità per l'orientamento)
            goal.trajectory.poses(:, :, i) = eye(4);
            goal.trajectory.poses(1:3, 4, i) = [x; y; z];
            
            % Tempo uniforme lungo il cerchio
            goal.trajectory.times(i) = total_time * (i-1)/(n_points-1);
        end
        
        disp('=== TRAJECTORY A: Circular trajectory selected ===');
        
    elseif strcmp(trajectory_option, 'B')
        %% OPTION B: LINEAR TRAJECTORY (stessa path following)
        % 4 waypoints forming a square path with varying heights
        
        goal.trajectory.n_waypoints = 4;
        goal.trajectory.poses = zeros(4, 4, goal.trajectory.n_waypoints);
        goal.trajectory.times = zeros(1, goal.trajectory.n_waypoints);
        
        % Define waypoint poses (4 corners of a path)
        % Waypoint 1: Starting position
        goal.trajectory.poses(:, :, 1) = eye(4);
        goal.trajectory.poses(1:3, 4, 1) = [0.70, -0.35, 0.50]';
        
        % Waypoint 2: Move to the side
        goal.trajectory.poses(:, :, 2) = eye(4);
        goal.trajectory.poses(1:3, 4, 2) = [0.5, -0.35, 0.50]';
        
        % Waypoint 3: Lower down
        goal.trajectory.poses(:, :, 3) = eye(4);
        goal.trajectory.poses(1:3, 4, 3) = [0.50, -0.35, 0.15]';
        
        % Waypoint 4: Move forward and up
        goal.trajectory.poses(:, :, 4) = eye(4);
        goal.trajectory.poses(1:3, 4, 4) = [0.70, -0.35, 0.30]';
        
        % Define timing for each waypoint (absolute times in seconds)
        goal.trajectory.times(1) = 0.0;   
        goal.trajectory.times(2) = 20.0;  
        goal.trajectory.times(3) = 30.0;  
        goal.trajectory.times(4) = 45.0;   
        
        disp('=== TRAJECTORY B: Linear trajectory selected ===');
        
    else
        error('Invalid trajectory_option! Use ''A'' or ''B''');
    end
    
    % Total trajectory duration
    goal.trajectory.duration = goal.trajectory.times(end);
    
    % Current trajectory state
    goal.trajectory.current_segment = 1;
    goal.trajectory.completed = false;

end

