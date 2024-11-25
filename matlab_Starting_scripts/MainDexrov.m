% MainDexrov
% Main function to initialize the simulation and manage the control loop
function MainDexrov
    addpath('./simulation_scripts'); % Add path to simulation scripts
    clc; clear; close all; % Clear workspace, close figures, and clear command window
    
    % Simulation variables (time step and total duration)
    deltat = 0.005; % Time step in seconds
    end_time = 15; % Total simulation time in seconds
    loop = 1; % Loop counter
    maxloops = ceil(end_time/deltat); % Maximum number of iterations
    
    % Mission structure to track phases and phase time
    mission.phase = 1; % Initial mission phase
    mission.phase_time = 0; % Time elapsed in the current phase
    
    % Rotation matrices for converting Unity to world frame and vice versa
    wuRw = rotation(0, -pi/2, pi/2); % Unity-to-world rotation
    vRvu = rotation(-pi/2, 0, -pi/2); % Vehicle-to-Unity rotation
    
    % Pipe parameters (position and radius in Unity and world coordinates)
    u_pipe_center = [-10.66 31.47 -1.94]'; % Pipe center in Unity coordinates
    pipe_center = wuRw' * u_pipe_center; % Pipe center in world frame
    pipe_radius = 0.3; % Pipe radius in meters
    
    % UDP connection setup for communication with Unity viewer
    uArm = udp('127.0.0.1', 15000, 'OutputDatagramPacketSize', 28); % Arm UDP
    uVehicle = udp('127.0.0.1', 15001, 'OutputDatagramPacketSize', 24); % Vehicle UDP
    fopen(uVehicle); % Open vehicle UDP connection
    fopen(uArm); % Open arm UDP connection
    
    % Initialize data plot structure
    plt = InitDataPlot(maxloops);
    
    % Initialize UVMS structure
    uvms = InitUVMS('DexROV'); % Select robot name (DexROV)
    uvms.q = [-0.0031 1.2586 0.0128 -1.2460 0.0137 0.0853-pi/2 0.0137]'; % Initial joint positions
    uvms.p = [-1.9379 10.4813-6.1 -29.7242-0.1 0 0 0]'; % Initial vehicle position
    
    % Define goal position for the tool
    distanceGoalWrtPipe = 0.3; % Distance of goal from the pipe surface
    uvms.goalPosition = pipe_center + (pipe_radius + distanceGoalWrtPipe) * [0 0 1]'; % Goal position
    uvms.wRg = rotation(pi, 0, 0); % Rotation of goal in world frame
    uvms.wTg = [uvms.wRg uvms.goalPosition; 0 0 0 1]; % Transformation matrix for goal
    
    % Define tool control point
    uvms.eTt = eye(4); % Tool frame relative to end-effector
    
    tic % Start timing
    for t = 0:deltat:end_time
        % Update transformations, Jacobians, task references, and activations
        uvms = UpdateTransforms(uvms); % Update transformation matrices
        uvms = ComputeJacobians(uvms); % Compute task Jacobians
        uvms = ComputeTaskReferences(uvms, mission); % Compute task references
        uvms = ComputeActivationFunctions(uvms, mission); % Compute activation functions
       
        % Kinematic control initialization
        rhop = zeros(13, 1); % Control vector (joint + vehicle velocities)
        Qp = eye(13); % Initial null space projection matrix
    
        % Call iCAT tasks in order of priority
        [Qp, rhop] = iCAT_task(uvms.A.mu, uvms.Jmu, Qp, rhop, uvms.xdot.mu, 0.000001, 0.0001, 10); % Minimum altitude
        [Qp, rhop] = iCAT_task(uvms.A.ha, uvms.Jha, Qp, rhop, uvms.xdot.ha, 0.0001, 0.01, 10); % Horizontal attitude
        [Qp, rhop] = iCAT_task(uvms.A.t, uvms.Jt, Qp, rhop, uvms.xdot.t, 0.0001, 0.01, 10); % Tool position control
        [Qp, rhop] = iCAT_task(eye(13), eye(13), Qp, rhop, zeros(13, 1), 0.0001, 0.01, 10); % Null space task
    
        % Extract joint and vehicle velocities
        uvms.q_dot = rhop(1:7); % Joint velocities
        uvms.p_dot = rhop(8:13); % Vehicle velocities
        
        % Integrate to compute new joint and vehicle positions
        uvms.q = uvms.q + uvms.q_dot * deltat; % Update joint positions
        uvms.p = integrate_vehicle(uvms.p, uvms.p_dot, deltat); % Update vehicle position
        
        % Check for mission phase updates
        [uvms, mission] = UpdateMissionPhase(uvms, mission);
        
        % Send packets to Unity for visualization
        SendUdpPackets(uvms, wuRw, vRvu, uArm, uVehicle);
            
        % Store data for plotting
        plt = UpdateDataPlot(plt, uvms, t, loop);
        loop = loop + 1; % Increment loop counter
       
        % Debug print every 0.1 seconds
        if (mod(t, 0.1) == 0)
            disp(t); % Display current time
            disp(uvms.p'); % Display vehicle position
        end
        
        % Slow down simulation to match real-time
        SlowdownToRealtime(deltat);
    end
    
    % Close UDP connections
    fclose(uVehicle);
    fclose(uArm);
    
    % Generate plots
    PrintPlot(plt);
end
