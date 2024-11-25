% InitUVMS
% Initializes the UVMS structure with default parameters based on the robot type
function [uvms] = InitUVMS(robotname)
    % Transformation matrix between the arm base and vehicle frame
    % Defines how the arm base is attached to the vehicle
    if (strcmp(robotname, 'DexROV'))
        uvms.vTb = [rotation(pi, 0, pi) [0.167 0 -0.43]'; 0 0 0 1]; % DexROV configuration
    elseif (strcmp(robotname, 'Robust'))
        uvms.vTb = [rotation(0, 0, pi) [0.85 0 -0.42]'; 0 0 0 1]; % Robust configuration
    end

    % Initial joint and vehicle velocities (zero at start)
    uvms.q_dot = [0 0 0 0 0 0 0]'; % Joint velocities
    uvms.p_dot = [0 0 0 0 0 0]'; % Vehicle velocities

    % Joint limits for MARIS arm configuration
    uvms.jlmin = [-2.9; -1.6; -2.9; -2.95; -2.9; -1.65; -2.8]; % Lower bounds
    uvms.jlmax = [2.9; 1.65; 2.9; 0.01; 2.9; 1.25; 2.8]; % Upper bounds

    % Initialize transformation matrices and Jacobians (identity or zero)
    uvms.wTv = eye(4, 4); % World-to-vehicle transform
    uvms.wTt = eye(4, 4); % World-to-tool transform
    uvms.vTw = eye(4, 4); % Vehicle-to-world transform
    uvms.vTe = eye(4, 4); % Vehicle-to-end-effector transform
    uvms.vTt = eye(4, 4); % Vehicle-to-tool transform
    uvms.vTg = eye(4, 4); % Vehicle-to-goal transform
    uvms.Ste = eye(6, 6); % Rigid body transform for end-effector
    uvms.bTe = eye(4, 4); % Base-to-end-effector transform
    uvms.bJe = eye(6, 7); % Jacobian (6x7) for arm end-effector
    uvms.djdq = zeros(6, 7, 7); % Derivatives of Jacobian w.r.t joints
    uvms.mu = 0; % Minimum altitude
    uvms.phi = zeros(3, 1); % Horizontal attitude
    uvms.sensorDistance = 0; % Distance from sensor to objects

    % Task Jacobians (to be computed at runtime)
    uvms.Jjl = []; % Joint limit Jacobian
    uvms.Jmu = []; % Minimum altitude Jacobian
    uvms.Jha = []; % Horizontal attitude Jacobian
    uvms.Jt_a = []; % Tool-frame Jacobian (arm)
    uvms.Jt_v = []; % Tool-frame Jacobian (vehicle)
    uvms.Jt = []; % Combined tool-frame Jacobian

    % Task references (to be computed at runtime)
    uvms.xdot.jl = []; % Joint limit reference
    uvms.xdot.mu = []; % Minimum altitude reference
    uvms.xdot.ha = []; % Horizontal attitude reference
    uvms.xdot.t = []; % Tool-frame reference

    % Task activation matrices (diagonal)
    uvms.A.jl = zeros(7, 7); % Joint limit activation
    uvms.A.mu = 0; % Minimum altitude activation
    uvms.A.ha = zeros(1, 1); % Horizontal attitude activation
    uvms.A.t = zeros(6, 6); % Tool-frame activation
end
