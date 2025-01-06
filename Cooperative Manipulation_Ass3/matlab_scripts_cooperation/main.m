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
    hudps = dsp.UDPSender('RemoteIPPort',1500);
    hudps.RemoteIPAddress = '127.0.0.1';
end
%% TO HERE

% Init robot model
wTb_left = eye(4); %fixed transformation word -> base left
wTb_right = eye(4);
wTb_right(1:3,1:3) = rotation(0,0,pi); %fixed transformation word -> base right
wTb_right(1:3,4) = [1.06; -0.01; 0];
pandaArm1 = InitRobot(model,wTb_left);
pandaArm2 = InitRobot(model,wTb_right);

% Preallocation
plt = InitDataPlot(maxloops);

% Init object frame
obj_length = 0.06;
w_obj_pos = [0.50; 0; 0.30];
w_obj_ori = rotation(0,0,0); %% !!!!!
pandaArm1.wTo = [w_obj_ori w_obj_pos; 0 0 0 1];
pandaArm2.wTo = pandaArm1.wTo; %% !!!!!

theta = -44.9949;% FIXED ANGLE BETWEEN EE AND TOOL 
tool_length = 0.2104;% FIXED DISTANCE BETWEEN EE AND TOOL
% Define trasnformation matrix from ee to tool.
pandaArm1.eTt = eye(4);
pandaArm1.eTt(1:3,1:3) = rotation(0, 0, deg2rad(theta));
pandaArm1.eTt(1:3,4) = [0; 0; tool_length];
pandaArm2.eTt = pandaArm1.eTt;

% Transformation matrix from <t> to <w>
pandaArm1.wTt = pandaArm1.wTe * pandaArm1.eTt;
pandaArm2.wTt = pandaArm2.wTe * pandaArm2.eTt;

% Transformation from tool to object
pandaArm1.tTo = inv(pandaArm1.wTt)  * (pandaArm1.wTo);
pandaArm2.tTo = inv(pandaArm2.wTt)  * (pandaArm2.wTo);


%% Defines the goal position for the end-effector/tool position task

% First goal reach the grasping points.
pandaArm1.wTg = eye(4);
pandaArm1.wTg(1:3,1:3) = pandaArm1.wTt(1:3,1:3) * rotation(0, deg2rad(20), 0); % !!!!
pandaArm1.wTg(1:3,4) = [w_obj_pos(1) - (obj_length/2); w_obj_pos(2); w_obj_pos(3)];
pandaArm2.wTg = eye(4);
pandaArm2.wTg(1:3,1:3) = pandaArm2.wTt(1:3,1:3) * rotation(0, deg2rad(20), 0);
pandaArm2.wTg(1:3,4) = [w_obj_pos(1) + (obj_length/2); w_obj_pos(2); w_obj_pos(3)];

% Second goal move the object
w_obj_g = [0.60; 0.40; 0.48]'; % object goal position
pandaArm1.wTog = eye(4);
pandaArm1.wTog(1:3,4) = w_obj_g;
pandaArm2.wTog = eye(4);
pandaArm2.wTog(1:3,4) = w_obj_g;

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
% TC = tool constraint
mission.actions.go_to.tasks = ["JL","MA","T"];
mission.actions.coop_manip.tasks = ["JL", "MA", "RC", "TC"];
mission.actions.end_motion.tasks = ["MA"];

% debug code
mission.error.lin = [];
mission.error.ang = [];

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
     

    %% call all necessary functions

    % Joints limit
    % arm1 (left)
    [Qp, ydotbar] = iCAT_task(pandaArm1.A.jl, pandaArm1.J.jl, Qp, ydotbar, ...
                              pandaArm1.xdot.jl, 0.0001,   0.01, 10);  
    % arm2 (right)
    [Qp2, ydotbar2] = iCAT_task(pandaArm2.A.jl, pandaArm2.J.jl, Qp2, ydotbar2, ...
                              pandaArm2.xdot.jl, 0.0001,   0.01, 10);   

    % Minimum distance from table
    % arm1 (left)
    [Qp, ydotbar] = iCAT_task(pandaArm1.A.ma, pandaArm1.J.ma, Qp, ydotbar, ...
                              pandaArm1.xdot.alt, 0.0001,   0.01, 10);  

    % arm2 (right)
    [Qp2, ydotbar2] = iCAT_task(pandaArm2.A.ma, pandaArm2.J.ma, Qp2, ydotbar2, ...
                              pandaArm2.xdot.alt, 0.0001,   0.01, 10);    

    % the sequence of iCAT_task calls defines the priority
    if mission.phase == 1
        % First Manipulator TPIK (left)
        % Task: Tool Move-To
        [Qp, ydotbar] = iCAT_task(pandaArm1.A.tool, pandaArm1.wJt, Qp,...
                                    ydotbar, pandaArm1.xdot.tool,...
                                    0.0001,   0.01, 10);    
    
        % Second manipulator TPIK (right)
        % Task: Tool Move-To
        [Qp2, ydotbar2] = iCAT_task(pandaArm2.A.tool, pandaArm2.wJt, Qp2,...
                                    ydotbar2, pandaArm2.xdot.tool,...
                                    0.0001,   0.01, 10);  
    end
    
   
    % COOPERATION hierarchy
    % SAVE THE NON COOPERATIVE VELOCITIES COMPUTED
    
    % Task: Left Arm Cooperation
    % ...
    if(mission.phase ==2)
        % 
        [Qp, ydotbar] = iCAT_task(pandaArm1.A.target,...
        pandaArm1.wJo,...
        Qp, ydotbar,...
        pandaArm1.xdot.tool,...
        0.0001,   0.01, 10);

        [Qp2, ydotbar2] = iCAT_task(pandaArm2.A.target,...
        pandaArm2.wJo,....
        Qp2, ydotbar2,...
        pandaArm2.xdot.tool,...
        0.0001,   0.01, 10);  
    end

    
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

%end