function [uvms] = ComputeTaskReferences(uvms, mission)
% compute the task references here

% Extract rover and rock positions in the world frame
p_rover = uvms.wTv(1:2, 4); % (x, y) coordinates of the rover in the world frame
p_rock  = uvms.rock_center(1:2); % (x, y) coordinates of the rock in the world frame
% Calculate the direction vector from the rover to the rock
d = p_rock - p_rover; % Direction vector
nd = d / norm(d); % Normalized to avoid numerical issues
% Extract the direction of the rover's X-axis in the world frame 
x_axis_rover = uvms.wTv(1:2,1);   % First column of the rotation matrix
% Calculate the angle between the rover's X-axis and the direction to the rock
uvms.angle = atan2(det([x_axis_rover, nd]), dot(x_axis_rover, nd));

% Calculate the reference velocity to align with the rock
xdot_ref = 0.3 * (uvms.angle);


% reference for tool-frame position control task
[ang, lin] = CartError(uvms.vTg , uvms.vTt);
uvms.xdot.t = 0.2 * [ang; lin];

% limit the requested velocities...
uvms.xdot.t(1:3) = Saturate(uvms.xdot.t(1:3), 0.2);
uvms.xdot.t(4:6) = Saturate(uvms.xdot.t(4:6), 0.2);

% MA task reference for minimum altituJvhde
uvms.xdot.ma = 0.2 * (2 - uvms.altitude);

% HA task reference for horizontal attitude
uvms.xdot.ha = -0.4 * (uvms.p(4:5) - [deg2rad(0); deg2rad(0)]);

% JL task reference for joint limits
uvms.xdot.jl = 0.5 * (((uvms.jlmax - uvms.jlmin)/2) - uvms.q);

% computing angular and linear error between vehicle goal and vehicle
[ang, lin] = CartError(uvms.wTgv, uvms.wTv);

% VH vehicle heading control task reference 
if mission.phase == 1 
    uvms.xdot.vh = 0.6 * ang(3);
else % mission.phase == 2 || mission.phase == 3 || mission.phase == 4
    % in this case the rover mus be aligned with the rock
    % compute the theta_dot reference (angular error)
   uvms.xdot.vh = xdot_ref;
end
uvms.xdot.vh = Saturate(uvms.xdot.vh, 0.8); % saturation, computed in every task

% VP vehicle position task reference
uvms.xdot.vp = 1 * lin;
if mission.phase == 2
     uvms.xdot.vp = [0; 0; 0];
elseif mission.phase == 3 % landing task
    uvms.xdot.vp = [0; 0; 0.6 * (0 - uvms.altitude)];
elseif mission.phase == 4
     uvms.xdot.vp = [0; 0; 0]; % No motor motions when landed
end
uvms.xdot.vp = Saturate(uvms.xdot.vp, 0.8);

% AC attitude control task
uvms.xdot.ac = 0.6 * ang(1:2);
uvms.xdot.ac = Saturate(uvms.xdot.ac, 0.8);

% VA Veichle Aligning
% in this case the rover mus be aligned with the rock
uvms.xdot.va = xdot_ref;
uvms.xdot.va = Saturate(uvms.xdot.va, 0.8);

% RN grasping task reference
[ang, lin] = CartError(uvms.wTg, uvms.wTt); % error between goal and tool
uvms.xdot.rn = 0.6 * [zeros(3,1); lin];


