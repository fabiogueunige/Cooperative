addpath('./simulation_scripts');
clc;
clear;
close all
real_robot = false;
%% Initialization - DON'T CHANGE ANYTHING from HERE ... 
% Simulation variables (integration and final time)
deltat = 0.005;
end_time = 15;
loop = 1;
maxloops = ceil(end_time/deltat);
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
    hudps = dsp.UDPSender('RemoteIPPort',1505);
    hudps.RemoteIPAddress = '127.0.0.1';
end
%% TO HERE

% Init robot model
wTb_left = ...; %fixed transformation word -> base left
wTb_right = ...; %fixed transformation word -> base right
pandaArm1 = InitRobot(model,wTb_left);
pandaArm2 = InitRobot(model,wTb_right);

% Preallocation
plt = InitDataPlot(maxloops);

% Init object frame
obj_length = ...;
w_obj_pos = ...;
w_obj_ori = ...;
pandaArm1.wTo = ...;
pandaArm2.wTo = ...;

theta = -44.9949;% FIXED ANGLE BETWEEN EE AND TOOL 
tool_length = 0.2124;% FIXED DISTANCE BETWEEN EE AND TOOL
% Define trasnformation matrix from ee to tool.
pandaArm1.eTt = ...;
pandaArm2.eTt = ...;

% Transformation matrix from <t> to <w>
pandaArm1.wTt = ...;
pandaArm2.wTt = ...;

%% Defines the goal position for the end-effector/tool position task
% First goal reach the grasping points.
pandaArm1.wTg = ...;
pandaArm2.wTg = ...;
% Second goal move the object
pandaArm1.wTog = ...;
pandaArm2.wTog = ...;

%% Mission configuration

mission.prev_action = "go_to";
mission.current_action = "go_to";

mission.phase = 1;
mission.phase_time = 0;
% Define the active tasks for each phase of the mission
% T = move tool task
% JL = joint limits task
% MA = minimum altitude task
% RC = rigid constraint task
mission.actions.go_to.tasks = [...];
mission.actions.coop_manip.tasks = [...];
mission.actions.end_motion.tasks = [...];

%% CONTROL LOOP
for t = 0:deltat:end_time

    % Receive UDP packets - DO NOT EDIT
    if real_robot == true
        dataLeft = step(hudprLeft);
        dataRight = step(hudprRight);
        % wait for data (to decide)
         if t == 0
             while(isempty(dataLeft))
                 dataLeft = step(hudprLeft);
                 pause(deltat);
             end
             while(isempty(dataRight))
                 dataRight = step(hudprRight);
                 pause(deltat);
             end
         end
        qL = typecast(dataLeft, 'double');
        qR = typecast(dataRight, 'double');
        pandaArms.ArmL.q = qL;
        pandaArms.ArmR.q = qR;
    end

    % update all the involved variables
    [pandaArm1] = UpdateTransforms(pandaArm1,mission);
    [pandaArm2] = UpdateTransforms(pandaArm2,mission);
    [pandaArm1] = ComputeJacobians(pandaArm1,mission);
    [pandaArm2] = ComputeJacobians(pandaArm2,mission);
    [pandaArm1] = ComputeActivationFunctions(pandaArm1,mission);
    [pandaArm2] = ComputeActivationFunctions(pandaArm2,mission);
    [pandaArm1] = ComputeTaskReferences(pandaArm1,mission);
    [pandaArm2] = ComputeTaskReferences(pandaArm2,mission);


    % main kinematic algorithm initialization
    % ydotbar order is [qdot_1, qdot_2, ..., qdot_7, xdot, ydot, zdot, omega_x, omega_y, omega_z]
    % the vector of the vehicle linear and angular velocities are assumed
    % projected on <v>
    
    ydotbar = zeros(7,1);
    Qp = eye(7);
    ydotbar2 = zeros(7,1);
    Qp2 = eye(7);

    % Used by the Move-To task
    tool_jacobian_L = zeros(6, 7);
    tool_jacobian_R = zeros(6, 7);
    if (mission.phase == 1)
        % In this phase the tool frame coincide with the center of the
        % gripper
        tool_jacobian_L = pandaArm1.wJt;
        tool_jacobian_R = pandaArm2.wJt;
    elseif(mission.phase == 2)
        % In this phase the tool frame coincide with the object frame
        tool_jacobian_L = pandaArm1.wJo;
        tool_jacobian_R = pandaArm2.wJo;
    end
    
    
    % ADD minimum distance from table
    % add all the other tasks here!
    % the sequence of iCAT_task calls defines the priority

    % First Manipulator TPIK (left)
    % Task: Tool Move-To
    [Qp, ydotbar] = iCAT_task(eye(7),...
        eye(7),...
        Qp2,...
        ydotbar, zeros(7,1),...
        0.0001,   0.01, 10);    % this task should be the last one

    % Second manipulator TPIK (right)
    % Task: Tool Move-To
    [Qp2, ydotbar2] = iCAT_task(eye(7),...
        eye(7),...
        Qp2,...
        ydotbar2, zeros(7,1),...
        0.0001,   0.01, 10);    % this task should be the last one
    
    % COOPERATION hierarchy
    % SAVE THE NON COOPERATIVE VELOCITIES COMPUTED

    % Task: Left Arm Cooperation
    % ...

    
    % this task should be the last one
    [Qp, ydotbar] = iCAT_task(eye(7),...
        eye(7),...
        Qp, ydotbar,...
        zeros(7,1),...
        0.0001,   0.01, 10);    
    % Task: Right Arm Cooperation
    % ...

    % this task should be the last one
    [Qp2, ydotbar2] = iCAT_task(eye(7),...
        eye(7),....
        Qp2, ydotbar2,...
        zeros(7,1),...
        0.0001,   0.01, 10);    
    % get the two variables for integration
    pandaArm1.q_dot = ydotbar(1:7);
    pandaArm2.q_dot = ydotbar2(1:7);

    pandaArm1.x = tool_jacobian_L * pandaArm1.q_dot;
    pandaArm2.x = tool_jacobian_R * pandaArm2.q_dot;

    % Integration
	pandaArm1.q = pandaArm1.q(1:7) + pandaArm1.q_dot*deltat;    
    pandaArm2.q = pandaArm2.q(1:7) + pandaArm2.q_dot*deltat;  

    %Send udp packets [q_dot1, ..., q_dot7] DO NOT CHANGE
    if real_robot == false
        pandaArm1.q = pandaArm1.q(1:7) + pandaArm1.q_dot*deltat; 
        pandaArm2.q = pandaArm2.q(1:7) + pandaArm2.q_dot*deltat; 
    end
    %Send udp packets [q_dot1, ..., q_dot7]
    if real_robot == true
        step(hudpsLeft,[t;pandaArm1.q_dot]);
        step(hudpsRight,[t;pandaArm2.q_dot]);
    else 
        step(hudps,[pandaArm1.q',pandaArm2.q'])
    end

    % check if the mission phase should be changed
    mission.phase_time = mission.phase_time + deltat;
    [pandaArm1,pandaArm2,mission] = UpdateMissionPhase(pandaArm1,pandaArm2,mission);

    % Compute distance between tools for plotting
    pandaArm1.dist_tools = norm(pandaArm1.wTt(1:3, 4) - pandaArm2.wTt(1:3, 4));

    % Update data for plots
    plt = UpdateDataPlot(plt,pandaArm1,pandaArm2,t,loop, mission);

    loop = loop + 1;
    % add debug prints here
    if (mod(t,0.1) == 0)
        t 
        mission.phase
    end
    
    % enable this to have the simulation approximately evolving like real
    % time. Remove to go as fast as possible
    % WARNING: MUST BE ENABLED IF CONTROLLING REAL ROBOT !
    SlowdownToRealtime(deltat);
    
end
PrintPlot(plt);

end