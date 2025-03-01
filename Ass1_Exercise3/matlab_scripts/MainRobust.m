clc;
clear;
close all

addpath('./simulation_scripts');
% Simulation variables (integration and final time)
deltat = 0.005;
end_time = 25;
loop = 1;
maxloops = ceil(end_time/deltat);

% this struct can be used to1 evolve what the UVMS has to do
mission.phase = 1;
mission.phase_time = 0;

% Rotation matrix to convert coordinates between Unity and the <w> frame
% do not change
wuRw = rotation(0,-pi/2,pi/2);
vRvu = rotation(-pi/2,0,-pi/2);

% pipe parameters
u_pipe_center = [-10.66 31.47 -1.94]'; % in unity coordinates
pipe_center = wuRw'*u_pipe_center;     % in world frame coordinates
pipe_radius = 0.3;

% rock position 
%uvms.rock_center = [12.2025   37.3748  -39.8860]'; % in world frame coordinates

% UDP Connection with Unity viewer v2
uArm = udp('127.0.0.1',15000,'OutputDatagramPacketSize',28);
uVehicle = udp('127.0.0.1',15001,'OutputDatagramPacketSize',24);
fopen(uVehicle);
fopen(uArm);
uAltitude = dsp.UDPReceiver('LocalIPPort',15003,'MessageDataType','single');
uAltitude.setup();

% Preallocation
plt = InitDataPlot(maxloops);

% initialize uvms structure
uvms = InitUVMS('Robust');
% uvms.q 
% Initial joint positions. You can change these values to initialize the simulation with a 
% different starting position for the arm
uvms.q = [-0.0031, 0, 0.0128, -1.2460, 0.0137, 0.0853-pi/2, 0.0137]'; 
% uvms.p
% initial position of the vehicle
% the vector contains the values in the following order
% [x y z r(rot_x) p(rot_y) y(rot_z)]
% RPY angles are applied in the following sequence
% R(rot_x, rot_y, rot_z) = Rz (rot_z) * Ry(rot_y) * Rx(rot_x)
uvms.p = [8.5 38.5 -36 0 -0.06 3.5]'; % initial pose

% defines the goal position for the end-effector/tool position task
uvms.goalPosition = [12.2025   37.3748  -39.8860]'; % rock position
uvms.wRg = rotation(0, 0, 0);
uvms.wTg = [uvms.wRg uvms.goalPosition; 0 0 0 1];

% defines the goal position for the vehicle position task
uvms.vehicleGoalPosition = [10.5 37.5 -38]';
uvms.wRgv = rotation(0.6, 0.6, 0.9);
uvms.wTgv = [uvms.wRgv uvms.vehicleGoalPosition; 0 0 0 1];

% defines the tool control point
uvms.eTt = eye(4);

uvms = ReceiveUdpPackets(uvms, uAltitude);
w_kw = [0 0 1]';
v_kw = uvms.vTw(1:3,1:3) * w_kw;
uvms.altitude = v_kw' * [0 0 uvms.sensorDistance]';

% definition of action
% MA = Minimum Altitude (z SAFETY TASK)
% HA = Horizontal Attitude ( roll, pitch SAFETY TASK)
% JL = Joints Limit ( SAFETY TASK)
% VH = Vehicle Heading (yaw CONTROL TASK)
% VP = Vehicle Position (x, y, z CONTROL TASK)
% VA = Veichle Aligning (CONTROL TASK)
% AC = Attitude Control ( roll, pitch CONTROL TASK)
% RN = reaching nodule task (CONTROL TASK)
uvms.actions.safe_navigation.tasks = ["MA", "HA", "VH","VP", "AC"];
uvms.actions.align.tasks = ["MA", "HA", "VA"];
uvms.actions.landing.tasks = [ "HA","VP"];
uvms.actions.fixed_base_manipulation.tasks = ["HA","JL", "RN"];

uvms.prev_action = uvms.actions.safe_navigation.tasks;
uvms.act_action = uvms.prev_action;

tic
for t = 0:deltat:end_time
    % update all the involved variables
    uvms = UpdateTransforms(uvms);
    uvms = ComputeJacobians(uvms, mission);
    uvms = ComputeTaskReferences(uvms, mission);
    uvms = ComputeActivationFunctions(uvms, mission);
    
    % receive altitude information from unity
    uvms = ReceiveUdpPackets(uvms, uAltitude);
    w_kw = [0 0 1]';
    v_kw = uvms.vTw(1:3,1:3) * w_kw;
    uvms.altitude = v_kw' * [0 0 uvms.sensorDistance]';
    
    % main kinematic algorithm initialization
    % ydotbar order is [qdot_1, qdot_2, ..., qdot_7, xdot, ydot, zdot, omega_x, omega_y, omega_z]
    % the vector of the vehicle linear and angular velocities are assumed
    % projected on <v>
    
    ydotbar = zeros(13,1);
    Qp = eye(13); 
    % add all the other taassasks here!
    % the sequence of iCAT_task calls defines the priority
    % robot task sequence changes basing on the mission phase
   
    % MA (minimum altitude: z; SAFETY TASK)
    [Qp, ydotbar] = iCAT_task(uvms.A.ma, uvms.J.ma, Qp, ydotbar, uvms.xdot.ma, 0.0001, 0.01, 10);

    % HA (horizontal attitude: roll, pitch; SAFETY TASK)
    [Qp, ydotbar] = iCAT_task(uvms.A.ha, uvms.J.ha, Qp, ydotbar, uvms.xdot.ha, 0.0001, 0.01, 10);
        
    % JL (joint limits: q1, q2, q3, q4, q5, q6, q7; SAFETY TASK)
    [Qp, ydotbar] = iCAT_task(uvms.A.jl, uvms.J.jl, Qp, ydotbar, uvms.xdot.jl, 0.0001, 0.01, 10);
    
    % VH (vehicle heading control: yaw; CONTROL TASK)
    [Qp, ydotbar] = iCAT_task(uvms.A.vh, uvms.J.vh, Qp, ydotbar, uvms.xdot.vh, 0.0001, 0.01, 10);
    
    % VP (vehicle position control: x, y, z; CONTROL TASK)
    [Qp, ydotbar] = iCAT_task(uvms.A.vp, uvms.J.vp, Qp, ydotbar, uvms.xdot.vp, 0.0001, 0.01, 10);
      
    % AC (vehicle attitude control: roll, pitch; CONTROL TASK)
    [Qp, ydotbar] = iCAT_task(uvms.A.ac, uvms.J.ac, Qp, ydotbar, uvms.xdot.ac, 0.0001, 0.01, 10);

    % VA (vehicle aligning CONTROL TASK)
    [Qp, ydotbar] = iCAT_task(uvms.A.va, uvms.J.va, Qp, ydotbar, uvms.xdot.va, 0.0001, 0.01, 10);
    
    % RN (reaching nodule: q1, q2, q3, q4, q5, q6, q7; CONTROL TASK)
    [Qp, ydotbar] = iCAT_task(uvms.A.t, [uvms.J.t_a zeros(6)], Qp, ydotbar, uvms.xdot.rn, 0.0001, 0.01, 10);
   
    % last task  
    [Qp, ydotbar] = iCAT_task(eye(13), eye(13), Qp, ydotbar, zeros(13,1), 0.0001, 0.01, 10);
    
    mission.phase_time = mission.phase_time + deltat;

    % get the two variables for integration
    uvms.q_dot = ydotbar(1:7);
    uvms.p_dot = ydotbar(8:13);
    
    % Integration
	uvms.q = uvms.q + uvms.q_dot*deltat;
    % beware: p_dot should be projected on <v>
    uvms.p = integrate_vehicle(uvms.p, uvms.p_dot, deltat);
    
    % check if the mission phase should be changed
    [uvms, mission] = UpdateMissionPhase(uvms, mission);
    
    % send packets to Unity viewer
    SendUdpPackets(uvms,wuRw,vRvu,uArm,uVehicle);
        
    % collect data for plots
    plt = UpdateDataPlot(plt,uvms,t,loop);
    loop = loop + 1;
   
    % add debug prints here
    if (mod(t,0.1) == 0)
        phase = mission.phase
        if mission.phase == 1
            % mission.phase_time  
            % disp(uvms.A.gv2)
            % disp(uvms.A.t)
        elseif mission.phase == 2
            
    
        elseif mission.phase == 3

        end
    end


    % enable this to have the simulation approximately evolving like real
    % time. Remove to go as fast as possible
    SlowdownToRealtime(deltat);
end

fclose(uVehicle);
fclose(uArm);

PrintPlot(plt);

