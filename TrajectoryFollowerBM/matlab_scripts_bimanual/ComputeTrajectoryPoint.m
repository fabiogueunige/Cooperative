function [pose, velocity, acceleration] = ComputeTrajectoryPoint(t_current, t_start, t_end, pose_start, pose_end)
% ComputeTrajectoryPoint - Computes desired pose, velocity and acceleration 
% using quintic polynomial interpolation
%
% Inputs:
%   t_current: current time
%   t_start: start time of this trajectory segment
%   t_end: end time of this trajectory segment
%   pose_start: 4x4 homogeneous transformation at start (position + orientation)
%   pose_end: 4x4 homogeneous transformation at end (position + orientation)
%
% Outputs:
%   pose: 4x4 homogeneous transformation at current time
%   velocity: 6x1 vector [angular_vel; linear_vel] in world frame
%   acceleration: 6x1 vector [angular_acc; linear_acc] in world frame

    % Extract positions and orientations
    p_start = pose_start(1:3, 4);
    p_end = pose_end(1:3, 4);
    R_start = pose_start(1:3, 1:3);
    R_end = pose_end(1:3, 1:3);
    
    % Compute trajectory duration
    T = t_end - t_start;
    
    % Clamp time to trajectory interval
    t = max(0, min(t_current - t_start, T));
    
    % Normalized time [0, 1]
    if T > 0
        s = t / T;
    else
        s = 1; % If duration is zero, jump to end
    end
    
    % Quintic polynomial coefficients (boundary conditions: zero velocity and acceleration at endpoints)
    % s(τ) = 10τ³ - 15τ⁴ + 6τ⁵
    % ds/dτ = 30τ² - 60τ³ + 30τ⁴
    % d²s/dτ² = 60τ - 180τ² + 120τ³
    
    s_pos = 10*s^3 - 15*s^4 + 6*s^5;
    s_vel = (30*s^2 - 60*s^3 + 30*s^4) / T;
    s_acc = (60*s - 180*s^2 + 120*s^3) / (T^2);
    
    % Linear interpolation for position
    p = p_start + s_pos * (p_end - p_start);
    p_dot = s_vel * (p_end - p_start);
    p_ddot = s_acc * (p_end - p_start);
    
    % SLERP (Spherical Linear Interpolation) for orientation
    % Find rotation from start to end
    R_delta = R_start' * R_end;
    
    % Convert to axis-angle
    trace_R = trace(R_delta);
    if trace_R >= 3 - 1e-6
        % No rotation needed
        omega_axis = [0; 0; 0];
        theta = 0;
    elseif trace_R <= -1 + 1e-6
        % 180 degree rotation - special case
        theta = pi;
        % Find the axis (largest diagonal element)
        [~, idx] = max(diag(R_delta));
        omega_axis = zeros(3,1);
        omega_axis(idx) = 1;
    else
        theta = acos((trace_R - 1) / 2);
        if abs(theta) < 1e-6
            omega_axis = [0; 0; 0];
        else
            omega_axis = (1 / (2*sin(theta))) * [R_delta(3,2) - R_delta(2,3);
                                                   R_delta(1,3) - R_delta(3,1);
                                                   R_delta(2,1) - R_delta(1,2)];
        end
    end
    
    % Interpolate rotation
    theta_interp = s_pos * theta;
    theta_dot = s_vel * theta;
    theta_ddot = s_acc * theta;
    
    if abs(theta_interp) < 1e-6
        R = R_start;
        omega = zeros(3,1);
        omega_dot = zeros(3,1);
    else
        % Rodrigues rotation formula
        K = skew(omega_axis);
        R_interp = eye(3) + sin(theta_interp)*K + (1-cos(theta_interp))*K^2;
        R = R_start * R_interp;
        
        % Angular velocity in world frame
        omega = R_start * (theta_dot * omega_axis);
        
        % Angular acceleration in world frame
        omega_dot = R_start * (theta_ddot * omega_axis);
    end
    
    % Construct output pose
    pose = [R, p; 0, 0, 0, 1];
    
    % Construct velocity vector [angular; linear]
    velocity = [omega; p_dot];
    
    % Construct acceleration vector [angular; linear]
    acceleration = [omega_dot; p_ddot];
    
end
