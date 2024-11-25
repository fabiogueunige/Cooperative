function MainRobust
% Main script for simulating the control of an underwater vehicle-manipulator system (UVMS).
% The code uses MATLAB for simulation and communicates with Unity for visualization.

addpath('./simulation_scripts'); % Add the path to the folder with required scripts
clc; clear; close all; % Clear console, variables, and figures

%% Simulation Variables
deltat = 0.005; % Simulation time step (integration interval)
end_time = 25; % Total simulation duration (seconds)
loop = 1; % Loop counter for tracking iterations
maxloops = ceil(end_time / deltat); % Total number of loops required

% Struct to manage the phases of the mission
mission.phase = 1; % Initial mission phase
mission.phase_time = 0; % Timer for the current mission phase

%% Rotation Matrices
% Predefined rotation matrices to align coordinate frames:
% - Unity to world frame (<w>)
wuRw = rotation(0, -pi/2, pi/2); % Rotation matrix from Unity to <w> frame
vRvu = rotation(-pi/2, 0, -pi/2); % Rotation matrix from vehicle (<v>) to Unity

%% Pipe and Rock Parameters
% Pipe position in Unity coordinates
u_pipe_center = [-10.66, 31.47, -1.94]'; 
pipe_center = wuRw' * u_pipe_center; % Convert to world frame coordinates
pipe_radius = 0.3; % Pipe radius

% Rock position in the world frame
rock_center = [12.2025, 37.3748, -39.8860]';

%% UDP Communication
% Initialize UDP communication with Unity viewer for visualization
uArm = udp('127.0.0.1', 15000, 'OutputDatagramPacketSize', 28); % Arm data
uVehicle = udp('127.0.0.1', 15001, 'OutputDatagramPacketSize', 24); % Vehicle data
fopen(uVehicle); % Open UDP connection for vehicle
fopen(uArm); % Open UDP connection for arm
uAltitude = dsp.UDPReceiver('LocalIPPort', 15003, 'MessageDataType', 'single'); % Altitude data
uAltitude.setup();

%% Preallocation
% Initialize data structure for storing plot data
plt = InitDataPlot(maxloops);

%% UVMS Initialization
% Initialize the UVMS structure and its state variables
uvms = InitUVMS('Robust');
uvms.q = [-0.0031, 0, 0.0128, -1.2460, 0.0137, 0.0853 - pi/2, 0.0137]'; % Initial joint positions
uvms.p = [8.5, 38.5, -38, 0, -0.06, 0.5]'; % Initial vehicle position and orientation (x, y, z, roll, pitch, yaw)

% Goal position for the end-effector (tool) in the world frame
uvms.goalPosition = [12.2025, 37.3748, -39.8860]'; 
uvms.wRg = rotation(0, pi, pi/2); % Desired orientation of the end-effector
uvms.wTg = [uvms.wRg, uvms.goalPosition; 0, 0, 0, 1]; % Homogeneous transformation matrix

% Tool control point (identity matrix, assuming no offset)
uvms.eTt = eye(4);

%% Simulation Loop
tic; % Start simulation timer
for t = 0:deltat:end_time
    %% Update UVMS Variables
    uvms = UpdateTransforms(uvms); % Update transformations
    uvms = ComputeJacobians(uvms); % Compute Jacobians
    uvms = ComputeTaskReferences(uvms, mission); % Compute task references
    uvms = ComputeActivationFunctions(uvms, mission); % Compute activation functions
    
    %% Receive Data from Unity
    uvms = ReceiveUdpPackets(uvms, uAltitude); % Receive altitude data
    
    %% Kinematic Algorithm
    % Initialize joint and vehicle velocity commands
    ydotbar = zeros(13, 1); % [qdot (7), linear velocity (3), angular velocity (3)]
    Qp = eye(13); % Null-space projector (identity matrix)
    
    % Insert tasks here using iCAT_task (e.g., prioritize tasks in sequence)
    % Example: [Qp, ydotbar] = iCAT_task(A, J, Qp, ydotbar, xdot, lambda, threshold, weight);
    [...]
    [Qp, ydotbar] = iCAT_task(eye(13), eye(13), Qp, ydotbar, zeros(13, 1), 0.0001, 0.01, 10); % Default task

    %% Integrate UVMS Dynamics
    uvms.q_dot = ydotbar(1:7); % Joint velocities
    uvms.p_dot = ydotbar(8:13); % Vehicle velocities
    uvms.q = uvms.q + uvms.q_dot * deltat; % Update joint positions
    uvms.p = integrate_vehicle(uvms.p, uvms.p_dot, deltat); % Update vehicle position

    %% Mission Phase Update
    [uvms, mission] = UpdateMissionPhase(uvms, mission); % Check and update mission phase
    
    %% Send Data to Unity Viewer
    SendUdpPackets(uvms, wuRw, vRvu, uArm, uVehicle); % Send updated state to Unity
    
    %% Data Logging
    plt = UpdateDataPlot(plt, uvms, t, loop); % Store data for plotting
    loop = loop + 1;
    
    %% Debugging
    if mod(t, 0.1) == 0 % Print debug info every 0.1 seconds
        disp(['Time: ', num2str(t)]);
        disp(['Sensor Distance: ', num2str(uvms.sensorDistance)]);
    end

    %% Real-Time Slowdown
    SlowdownToRealtime(deltat); % Optional: match simulation time with real time
end

%% Close UDP Connections and Plot Results
fclose(uVehicle);
fclose(uArm);
PrintPlot(plt); % Generate plots for simulation results

end
