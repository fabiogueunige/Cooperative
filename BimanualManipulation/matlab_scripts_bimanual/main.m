%% Template Exercises Manipulation - Cooperative Robotics a.y. 24-25
addpath('./simulation_scripts');
clc;
clear;
close all
real_robot = false;

%% Initialization - DON'T CHANGE ANYTHING from HERE ... 
% Simulation variables (integration and final time)
dt = 0.005;
Tf = 15; %simulation time
loop = 1;
maxloops = ceil(Tf/dt);
mission.phase = 1;
mission.phase_time = 0;
model = load("panda.mat");

% UDP Connection with Franka Interface
if real_robot == true
    hudprLeft = dsp.UDPReceiver('LocalIPPort',1501,'MaximumMessageLength',255);
    hudprRight = dsp.UDPReceiver('LocalIPPort',1503,'MaximumMessageLength',255);
    hudpsLeft = dsp.UDPSender('RemoteIPPort',1500);
    hudpsLeft.RemoteIPAddress = '127.0.0.1';
    hudpsRight = dsp.UDPSender('RemoteIPPort',1502);
    hudpsRight.RemoteIPAddress = '127.0.0.1';
else
    hudps = dsp.UDPSender('RemoteIPPort',1500);
    hudps.RemoteIPAddress = '127.0.0.1';
end
%% ... to HERE.
% Init robot model
wTb_left = eye(4); % fixed transformation word -> base1 (coincides with left arm)
% fixed transformation word -> base2
wTb_right = eye(4); 
wTb_right (1:3,1:3) = rotation(0, 0, pi);
wTb_right (1:3,4) = [1.06; -0.01; 0];

plt = InitDataPlot(maxloops);
pandaArm = InitRobot(model, wTb_left, wTb_right);
% Init object and tools frames
obj_length = 0.12;
w_obj_pos = [0.5 0 0.59]';
w_obj_ori = rotation(0,0,0);
pandaArm.ArmL.wTo = [w_obj_ori w_obj_pos;
                       0 0 0 1];
pandaArm.ArmR.wTo = pandaArm.ArmL.wTo;

theta = -44.9949;% FIXED ANGLE BETWEEN EE AND TOOL 
tool_length = 0.2104;% FIXED DISTANCE BETWEEN EE AND TOOL
% Define trasnformation matrix from ee to tool.
pandaArm.ArmL.eTt = eye(4);
pandaArm.ArmL.eTt(1:3, 1:3) = rotation(0, 0, deg2rad(theta));
pandaArm.ArmL.eTt(1:3, 4) = [0; 0; tool_length];

pandaArm.ArmR.eTt = pandaArm.ArmL.eTt;

% Transformation matrix from <t> to <w>
pandaArm.ArmL.wTt = pandaArm.ArmL.wTe * pandaArm.ArmL.eTt;
pandaArm.ArmR.wTt = pandaArm.ArmR.wTe * pandaArm.ArmR.eTt;

%% Defines the goal position for the end-effector/tool position task
% First goal reach the grasping points.
w_obj_g_left = [0.44 0 0.59]';
w_obj_g_right = [0.56 0 0.59]';
% the tool should rotate of 30 deg on y axis to reach goal orientation
pandaArm.ArmL.wTg = eye(4);
pandaArm.ArmL.wTg(1:3, 1:3) = pandaArm.ArmL.wTt(1:3, 1:3) * rotation(0, pi/6, 0);
pandaArm.ArmL.wTg (1:3, 4) = w_obj_g_left;

pandaArm.ArmR.wTg = eye(4);
pandaArm.ArmR.wTg(1:3, 1:3) = pandaArm.ArmR.wTt(1:3, 1:3) * rotation(0, pi/6, 0);
pandaArm.ArmR.wTg (1:3, 4) = w_obj_g_right;

%% prova
pandaArm.ArmL.tTo = inv(pandaArm.ArmL.wTt) * pandaArm.ArmL.wTo; 
pandaArm.ArmR.tTo = inv(pandaArm.ArmR.wTt) * pandaArm.ArmR.wTo; 

%% Second goal move the object
pandaArm.wTog = eye(4);
% probably check possible error, chek in case wtg intread wto, depending
% the rotation of the tool
pandaArm.wTog(1:3, 1:3) = pandaArm.ArmR.wTo(1:3, 1:3);
pandaArm.wTog(1:3, 4) = [0.65, -0.35, 0.28]';

%% Mission configuration

mission.prev_action = "go_to";
mission.current_action = "go_to";

mission.phase = 1;
mission.phase_time = 0;
% Define the active tasks for each phase of the mission
% Suggested Name for the task
% T = move tool task
% JL = joint limits task
% MA = minimum altitude task
% RC = rigid constraint task
mission.actions.go_to.tasks = ["JL", "MA", "T"];
mission.actions.coop_manip.tasks = ["JL", "MA", "RC", "TC"];
mission.actions.end_motion.tasks = ["JL", "MA", "RC"];
mission.actions.previous = 'NULL';

%% CONTROL LOOP
disp('STARTED THE SIMULATION');
for t = 0:dt:Tf
    % Receive UDP packets - DO NOT EDIT
    if real_robot == true
        dataLeft = step(hudprLeft);
        dataRight = step(hudprRight);
        % wait for data (to decide)
         if t == 0
             while(isempty(dataLeft))
                 dataLeft = step(hudprLeft);
                 pause(dt);
             end
             while(isempty(dataRight))
                 dataRight = step(hudprRight);
                 pause(dt);
             end
         end
        qL = typecast(dataLeft, 'double');
        qR = typecast(dataRight, 'double');
        pandaArm.ArmL.q = qL;
        pandaArm.ArmR.q = qR;
    end
    
    % update all the involved variables
    pandaArm = UpdateTransforms(pandaArm, mission);
    pandaArm = ComputeJacobians(pandaArm, mission);
    pandaArm = ComputeActivationFunctions(pandaArm,mission);
    pandaArm = ComputeTaskReferences(pandaArm,mission);

    % main kinematic algorithm initialization
    % ydotbar order is [qdot_1, qdot_2, ..., qdot_7, xdot, ydot, zdot, omega_x, omega_y, omega_z]
    % the vector of the vehicle linear and angular velocities are assumed
    % projected on <w>
    
    ydotbar = zeros(14, 1);
    Qp = eye(14);

    % Used by the Move-To task
    tool_jacobian_L = zeros(6, 7);
    tool_jacobian_R = zeros(6, 7);
    if (mission.phase == 1)
        % In this phase the tool frame coincide with the center of the
        % gripper
        tool_jacobian_L = pandaArm.ArmL.wJt;
        tool_jacobian_R = pandaArm.ArmR.wJt;
    elseif(mission.phase == 2)
        % In this phase the tool frame coincide with the object frame
        tool_jacobian_L = pandaArm.ArmL.wJo;
        tool_jacobian_R = pandaArm.ArmR.wJo;
    end

    % ADD minimum distance from table
    % add all the other tasks here!
    % the sequence of iCAT_task calls defines the priority
 
    %% EXAMPLE 
    %[Qp, ydotbar] = iCAT_task(pandaArm.ArmL.A.ma, Jma,    ...
    %                           Qp, ydotbar, zeros(14,1),  ...
    %                          0.0001,   0.01, 10);
    %iCAT_task(A, J, Qold, rhoold, xdot, lambda, threshold, weight)

    %% INitialization adn Parameters
    Qold = Qp;   
    rhoold = ydotbar;
    lambda = 0.0001;
    threshold = 0.01;
    weight = 10;

    %% MINIMUM ALTITUDE
    % we have two task of dimension 6, we consider here the all robot dof. so A = 12 x 12
    A = zeros(6);
    A (6, 6) = pandaArm.ArmL.A.ma;
    J = zeros(6,14);
    J(6,1:7) = pandaArm.ArmL.Jma;
    % 
    [Qold, ydotbar] = iCAT_task(A, J, Qold, ydotbar, pandaArm.ArmL.xdot.alt, lambda, threshold, weight);


    A (6,6) = pandaArm.ArmR.A.ma; 
    J = zeros(6, 14);
    J(6,8:14) = pandaArm.ArmR.Jma;
    % values poassed by default
    % minimum altitude left
    [Qold, ydotbar] = iCAT_task(A, J, Qold, ydotbar, pandaArm.ArmR.xdot.alt, lambda, threshold, weight);

    if A ~= 0
        mission.actions.previous = 'MA';
    end


    %Qold % dim = 14 x 14
    %rhoold % dim 12 x 14

    %pandaArm.ArmL.wTt(1:3, 4) % print the distance of tool w.r.t. world

    %% JOINT LIMIT
    % we have 14 task, of one dimension each. Because we act directly on
    % the single joint velocity
    % in this case I am in space joint yet, so there isn't mapping beween
    % the real space and joint space, so the J is an Identity matrix
    A = zeros(14,14);
    A(1:7,1:7) = pandaArm.ArmL.A.jl;
    A(8:14,8:14) = pandaArm.ArmR.A.jl;
    % in this case m = 14, same shape of A, because the the speed reference, x_ref = 14, so we are
    % mapping directly the speed of the joint, not all the 6dof of each manipulator
    J = eye(14,14);
    xdot = zeros(14);
    [Qold, rhoold] = iCAT_task(A, J, Qold, rhoold, xdot, lambda, threshold, weight);

    if A ~= zeros(14,14)    
        mission.actions.previous = 'JL';
    end

    %% GRASPING TASK
    if mission.phase == 1
        A = eye(6) * pandaArm.A.tool;
        
        % Left Arm
        %12 row, 6 ang vel, 6 lin vel 
        J = [pandaArm.ArmL.wJt, zeros(6,7)];
        [Qold, ydotbar] = iCAT_task(A, J, Qold, ydotbar, pandaArm.ArmL.xdot.tool, lambda, threshold, weight);
        %ydotbar
    
        % Right Arm
        J = [zeros(6, 7), pandaArm.ArmR.wJt];
        [Qold, ydotbar] = iCAT_task(A, J, Qold, ydotbar, pandaArm.ArmR.xdot.tool, lambda, threshold, weight);
        %ydotbar
        
        mission.actions.previous = 'T';
    end
    
    if mission.phase == 2
        %% TARGET 
        A = eye(6) * pandaArm.A.target;

        % Left Arm
        J = [pandaArm.ArmL.wJo, zeros(6,7)];
        [Qold, ydotbar] = iCAT_task(A, J, Qold, ydotbar, pandaArm.ArmL.xdot.obj, lambda, threshold, weight);
        
        % Right Arm
        J = [zeros(6,7), pandaArm.ArmR.wJo];
        [Qold, ydotbar] = iCAT_task(A, J, Qold, ydotbar, pandaArm.ArmR.xdot.obj, lambda, threshold, weight);

        mission.actions.previous = 'T';

        %% RIGID CONSTRAINT TASK
        A = eye(6) * pandaArm.A.rc;
        
        % Left arm
        J = [pandaArm.ArmL.wJo - pandaArm.ArmR.wJo, zeros(6,7)];
        [Qold, ydotbar] = iCAT_task(A, J, Qold, ydotbar, pandaArm.xdot.rc, lambda, threshold, weight);

        % Right arm
        J = [zeros(6,7), pandaArm.ArmL.wJo - pandaArm.ArmR.wJo];
        [Qold, ydotbar] = iCAT_task(A, J, Qold, ydotbar, pandaArm.xdot.rc, lambda, threshold, weight);

        mission.actions.previous = 'RC';
        
    end



    %% LAST TASK
    [Qold, ydotbar] = iCAT_task(eye(14),     eye(14),    ...
         Qold, ydotbar, zeros(14,1),  ...
         0.0001,   0.01, 10);    % this task should be the last one

    % get the two variables for integration
    pandaArm.ArmL.q_dot = ydotbar(1:7);
    pandaArm.ArmR.q_dot = ydotbar(8:14);

    pandaArm.ArmL.x = tool_jacobian_L * pandaArm.ArmL.q_dot;
    pandaArm.ArmR.x = tool_jacobian_R * pandaArm.ArmR.q_dot;
    % Integration
	pandaArm.ArmL.q = pandaArm.ArmL.q(1:7) + pandaArm.ArmL.q_dot*dt;    
    pandaArm.ArmR.q = pandaArm.ArmR.q(1:7) + pandaArm.ArmR.q_dot*dt;
    %Send udp packets [q_dot1, ..., q_dot7] DO NOT CHANGE
    if real_robot == false
        pandaArm.ArmL.q = pandaArm.ArmL.q(1:7) + pandaArm.ArmL.q_dot*dt; 
        pandaArm.ArmR.q = pandaArm.ArmR.q(1:7) + pandaArm.ArmR.q_dot*dt; 
    end
    %Send udp packets [q_dot1, ..., q_dot7]
    if real_robot == true
        step(hudpsLeft,[t;pandaArm.ArmL.q_dot]);
        step(hudpsRight,[t;pandaArm.ArmR.q_dot]);
    else 
        step(hudps,[pandaArm.ArmL.q',pandaArm.ArmR.q']);
        % step(hudps,[[0, 0, 0, 0, 0, 0, 0],[0, 0, 0, 0, 0, 0, 0]])
    end
    % check if the mission phase should be changed
    mission.phase_time = mission.phase_time + dt;
    [pandaArm,mission] = UpdateMissionPhase(pandaArm, mission);
   
    % Update data plot
    plt = UpdateDataPlot(plt,pandaArm,t,loop, mission);
    loop = loop + 1;
    % add debug prints here
    if (mod(t,0.1) == 0)
        t
        phase = mission.phase;
        time = mission.phase_time;
        if (mission.phase == 1)
            %add debug prints phase 1 here
        elseif (mission.phase == 2)
            %disp(pandaArm.A.tool);
            disp(pandaArm.A.target);
            %add debug prints phase 2 here
            
        end
    end
    
    % enable this to have the simulation approximately evolving like real
    % time. Remove to go as fast as possible
    % WARNING: MUST BE ENABLED IF CONTROLLING REAL ROBOT !
    SlowdownToRealtime(dt);
    
end

PrintPlot(plt, pandaArm);
