%% Template Exercises Manipulation - Cooperative Robotics a.y. 24-25
addpath('./simulation_scripts');
clc;
clear;
close all
real_robot = false;

%% Initialization - DON'T CHANGE ANYTHING from HERE ... 
% Simulation variables (integration and final time)
dt = 0.005;
Tf = 180; %simulation time
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
[pandaArm, goal] = InitRobot(model, wTb_left, wTb_right);

%Init object and tools frames
obj_length = 0.12;
w_obj_pos = [0.5 0 0.50]';
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
w_obj_g_left = [(w_obj_pos(1) - obj_length/2), w_obj_pos(2), w_obj_pos(3)]';
w_obj_g_right = [(w_obj_pos(1) + obj_length/2), w_obj_pos(2), w_obj_pos(3)]';
% the tool should rotate of 30 deg on y axis to reach goal orientation
pandaArm.ArmL.wTg = eye(4);
pandaArm.ArmL.wTg(1:3, 1:3) = pandaArm.ArmL.wTt(1:3, 1:3) * rotation(0, pi/6, 0);
pandaArm.ArmL.wTg (1:3, 4) = w_obj_g_left;

pandaArm.ArmR.wTg = eye(4);
pandaArm.ArmR.wTg(1:3, 1:3) = pandaArm.ArmR.wTt(1:3, 1:3) * rotation(0, pi/6, 0);
pandaArm.ArmR.wTg (1:3, 4) = w_obj_g_right;

%% Mission configuration

% Define the active tasks for each phase of the mission
% Suggested Name for the task
% T = move tool task
% JL = joint limits task
% MA = minimum altitude task
% RC = rigid constraint task
% CM = cooperative manipulation
% S = stop motion

mission.actions.go_to.tasks = ["JL", "MA", "T"];
mission.actions.coop_manip.tasks = ["JL", "MA", "RC", "T"];
mission.actions.end_motion.tasks = ["MA", "T"];

mission.phase = 1;
mission.phase_time = 0;
mission.prev_action = mission.actions.go_to.tasks; % variable updated in UpdateMissionPhase.m
mission.current_action = mission.actions.go_to.tasks;

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
    
    % No longer needed - trajectory follower doesn't use point A and B
    % if mission.phase == 1
    %     A = w_obj_pos;
    %     B = w_obj_pos + [0.3 0 0]';
    % end
    
    %%
    % Path follower logic removed - now using trajectory follower
    % line passing through A and B: r = a + t*v
    %v = (B-A); % vector between A and B
    %T = ( ((v(1)*wTog(1,4)) + (v(2)*wTog(2,4)) + (v(3)*wTog(3,4))) - ((v(1)*A(1)) + (v(2)*A(2)) + (v(3)*A(3))) ) / (v(1)^2 + v(2)^2 + v(3)^2);
    %H = A + v * t; % intersection between plane and line
    %d = norm(wTog(1:3,4),H); % distance from object to line between A,B
    
    %%
    % update all the involved variables
    pandaArm = UpdateTransforms(pandaArm, mission);
    pandaArm = ComputeJacobians(pandaArm, mission);
    pandaArm = ComputeActivationFunctions(pandaArm,mission);
    pandaArm = ComputeTaskReferences(pandaArm,mission,goal);

    % main kinematic algorithm initialization
    % ydotbar order is [qdot_1, qdot_2, ..., qdot_7, xdot, ydot, zdot, omega_x, omega_y, omega_z]
    % the vector of the vehicle linear and angular velocities are assumed
    % projected on <w>
    
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

    % add all the other tasks here!
    % the sequence of iCAT_task calls defines the priority
    %% Initialization and Parameters
    ydotbar = zeros(14, 1);
    Qp = eye(14);
    lambda = 0.0001;
    threshold = 0.01;
    weight = 10;
        
    % RIGID CONSTRAINT TASK
    [Qp, ydotbar] = iCAT_task(pandaArm.A.rc, pandaArm.Jrc, Qp, ydotbar, pandaArm.xdot.rc, lambda, threshold, weight);
  
    % JOINT LIMIT TASK
    % we have 14 task, of one dimension each. Because we act directly on
    % the single joint velocity
    % in this case I am in space joint yet, so there isn't mapping beween
    % the real space and joint space, so the J is an Identity matrix
    % left arm
    %[Qp, ydotbar] = iCAT_task(pandaArm.ArmL.A.jl, pandaArm.ArmL.Jjl, Qp, ydotbar, pandaArm.ArmL.xdot.jl, lambda, threshold, weight);
    % right arm
    %[Qp, ydotbar] = iCAT_task(pandaArm.ArmR.A.jl, pandaArm.ArmR.Jjl, Qp, ydotbar, pandaArm.ArmR.xdot.jl, lambda, threshold, weight);
   
    % MINIMUM ALTITUDE TASK
    % we have two task of dimension 6, we consider here the all robot dof.
    % left arm
    %[Qp, ydotbar] = iCAT_task(pandaArm.ArmL.A.ma, pandaArm.ArmL.Jma, Qp, ydotbar, pandaArm.ArmL.xdot.alt, lambda, threshold, weight);
    % right arm
    %[Qp, ydotbar] = iCAT_task(pandaArm.ArmR.A.ma, pandaArm.ArmR.Jma, Qp, ydotbar, pandaArm.ArmR.xdot.alt, lambda, threshold, weight);

    % MOVE TASK
    % Left Arm 
    [Qp, ydotbar] = iCAT_task(pandaArm.ArmL.A.tool, pandaArm.ArmL.J, Qp, ydotbar, pandaArm.ArmL.xdot.tool, lambda, threshold, weight);
    
    % Right Arm
    [Qp, ydotbar] = iCAT_task(pandaArm.ArmR.A.tool, pandaArm.ArmR.J, Qp, ydotbar, pandaArm.ArmR.xdot.tool, lambda, threshold, weight);
    
    % LAST TASK
    [Qp, ydotbar] = iCAT_task(eye(14), eye(14), Qp, ydotbar, zeros(14,1),  ...
                                0.0001,   0.01, 10);
   
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
        % step(hudps,[[0, 0, 0, 0, 0, -200, 0],[0, 0, 20, 0, 0, 0, 0]]);
    end
    % check if the mission phase should be changed
    mission.phase_time = mission.phase_time + dt;
    [pandaArm,mission,goal] = UpdateMissionPhase(pandaArm,mission,goal);
   
    % Update data plot
    plt = UpdateDataPlot(plt,pandaArm,t,loop, mission);
    % loop = loop + 1;

    % add debug prints here
    if (mod(t,0.1) == 0)
        t;
        phase = mission.phase
        time = mission.phase_time;
       
       if (mission.phase == 1)
            %add debug prints phase 1 here

            %disp(pandaArm.A.rc); %ok

            %disp(pandaArm.ArmL.A.jl);
            %disp(pandaArm.ArmR.A.jl);
        
            % disp (pandaArm.ArmL.A.ma);
            % disp(pandaArm.ArmR.A.ma);

            % disp(pandaArm.ArmL.A.tool);
            % disp(pandaArm.ArmR.A.tool);
            
        elseif (mission.phase == 2)
            %add debug prints phase 2 here

            % disp(pandaArm.A.rc); %ok

            % disp("ArmL jl");
            % disp(pandaArm.ArmL.A.jl);
            % disp("ArmR jl");
            % disp(pandaArm.ArmR.A.jl);

            % disp("ArmL ma");
            % disp (pandaArm.ArmL.A.ma);
            % disp("ArmR ma");
            % disp(pandaArm.ArmR.A.ma);

            % disp(pandaArm.ArmL.A.tool);
            % disp(pandaArm.ArmR.A.tool);
            
            loop = loop + 1;

            % TRAJECTORY FOLLOWER DEBUG
            disp("Trajectory time: " + goal.trajectory.time_in_phase);
            disp("Current segment: " + goal.trajectory.current_segment);
            
            distance_x(loop) = pandaArm.ArmL.wTt(1, 4) - pandaArm.ArmR.wTt(1, 4);
            traiettoria_L(:, loop) = pandaArm.ArmL.wTo(1:3, 4);
            traiettoria_R(:, loop) = pandaArm.ArmR.wTo(1:3,4);
            
            % Store desired trajectory point instead of LOS
            if isfield(goal.trajectory, 'pose_desired')
                desired_point(:, loop) = goal.trajectory.pose_desired(1:3, 4);
            end
            
            % Compute tracking error
            if isfield(goal.trajectory, 'pose_desired')
                [lin, ang] = CartError(goal.trajectory.pose_desired, pandaArm.ArmL.wTo);
                lineare(:, loop) = lin;
                angolare(:, loop) = ang;
            end
        elseif (mission.phase == 3)
            %add debug prints phase 2 here

            % disp(pandaArm.A.rc); %ok
            
            % disp(pandaArm.ArmL.A.jl);
            % disp(pandaArm.ArmR.A.jl);
            
            % disp(pandaArm.ArmL.A.tool);
            % disp(pandaArm.ArmR.A.tool);
           
        end
    end
    
    % enable this to have the simulation approximately evolving like real
    % time. Remove to go as fast as possible
    % WARNING: MUST BE ENABLED IF CONTROLLING REAL ROBOT !
    SlowdownToRealtime(dt);
end  
PrintPlot(plt, pandaArm);
