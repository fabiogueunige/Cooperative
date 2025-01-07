function MainRobust
addpath('./simulation_scripts');
clc;
clear;
close all

% Simulation variables (integration and final time)
deltat = 0.005;
end_time = 25;
loop = 1;
maxloops = ceil(end_time/deltat);

% this struct can be used to evolve what the UVMS has to do
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
rock_center = [12.2025   37.3748  -39.8860]'; % in world frame coordinates

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
uvms.q = [-0.0031 0 0.0128 -1.2460 0.0137 0.0853-pi/2 0.0137]'; 
% uvms.p
% initial position of the vehicle
% the vector contains the values in the following order
% [x y z r(rot_x) p(rot_y) y(rot_z)]
% RPY angles are applied in the following sequence
% R(rot_x, rot_y, rot_z) = Rz (rot_z) * Ry(rot_y) * Rx(rot_x)
uvms.p = [48.5 11.5 -33 0 0 pi/3]'; % initial xyz [48.5 11.5 -33]

% defines the goal position for the end-effector/tool position task
uvms.goalPosition = [50 -12.5 -33]';
uvms.wRg = rotation(0, pi, pi/2);
uvms.wTg = [uvms.wRg uvms.goalPosition; 0 0 0 1];

% defines the goal position for the vehicle position task
uvms.vehicleGoalPosition = [50 -12.5 -33]';
uvms.wRgv = rotation(0, 0, 0);
uvms.wTgv = [uvms.wRgv uvms.vehicleGoalPosition; 0 0 0 1];

% defines the tool control point
uvms.eTt = eye(4);

uvms = ReceiveUdpPackets(uvms, uAltitude);
w_kw = [0 0 1]';
v_kw = uvms.vTw(1:3,1:3) * w_kw;
uvms.altitude = v_kw' * [0 0 uvms.sensorDistance]';

% definition of action
% MA = Minimum Altitude
% HA = Horizontal Attitude
% VP = Vehicle Position
% AC = Attitude Control
% AC0 = Altitude control to 0 (equality task)
uvms.actions.safe_navigation.tasks = ["MA", "HA", "VP", "AC"];
uvms.actions.landing.tasks = ["AC0", "HA", "VP"];


tic
for t = 0:deltat:end_time
    % update all the involved variables
    uvms = UpdateTransforms(uvms);
    uvms = ComputeJacobians(uvms);
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
    %[Qp, ydotbar] = iCAT_task(uvms.A.t,    uvms.Jt,    Qp, ydotbar, uvms.xdot.t,  0.0001,   0.01, 10);
    % robot task sequence changes basing on the mission phase
    if(mission.phase == 1)
        % SAFETY NAVIGATION
        
        % MA
        [Qp, ydotbar] = iCAT_task(uvms.A.ma,    uvms.Jma,    Qp, ydotbar, uvms.xdot.ma,  0.0001,   0.01, 10);
    
        % HA
        [Qp, ydotbar] = iCAT_task(uvms.A.ha,    uvms.Jha,    Qp, ydotbar, uvms.xdot.ha,  0.0001,   0.01, 10);

        % VP
        [Qp, ydotbar] = iCAT_task(uvms.A.gv,    uvms.Jgv,    Qp, ydotbar, uvms.xdot.gv,  0.0001,   0.01, 10);
        
        % AC
        % da sistemare perché funziona male
        %[Qp, ydotbar] = iCAT_task(uvms.A.ac,    uvms.Jac,    Qp, ydotbar, uvms.xdot.ac,  0.0001,   0.01, 10);
        

    elseif(mission.phase == 2)
        % LANDING

        % Altitude Control to 0
        % non funzia (dio bono)
        [Qp, ydotbar] = iCAT_task(uvms.A.ac0, -uvms.Jma  ,    Qp, ydotbar, uvms.xdot.ac0,  0.0001,   0.01, 10);
        
        
        % HA
        [Qp, ydotbar] = iCAT_task(uvms.A.ha,    uvms.Jha,    Qp, ydotbar, uvms.xdot.ha,  0.0001,   0.01, 10);
        
        % VP (only x and y)
        [Qp, ydotbar] = iCAT_task(uvms.A.gv(1:2, 1:2),    uvms.Jgv(1:2, :),    Qp, ydotbar, uvms.xdot.gv(1:2),  0.0001,   0.01, 10);
        

    end

    
    %[Qp, ydotbar] = iCAT_task(eye(13),     eye(13),    Qp, ydotbar, zeros(13,1),  0.0001,   0.01, 10);    % this task should be the last one
    

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
        %phase = mission.phase;
        t = mission.phase_time
        %uvms.sensorDistance
        uvms.p(4:5)
        uvms.xdot.ha
        uvms.A.ha
        %uvms.altitude
        %mission.phase
        
        
    end

    % enable this to have the simulation approximately evolving like real
    % time. Remove to go as fast as possible
    SlowdownToRealtime(deltat);
end

fclose(uVehicle);
fclose(uArm);

%PrintPlot(plt);

end
