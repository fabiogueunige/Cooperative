function [uvms] = ComputeTaskReferences(uvms, mission)
% compute the task references here

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
    d = uvms.rock_center(1:2) - uvms.wTv(1:2, 4); % distance vector between rock and rover
    nd = d / norm(d); % normalized distance  
    uvms.angle = atan2(nd(2), nd(1)) + atan2(uvms.wTv(2, 1), uvms.wTv(1, 1));
    uvms.xdot.vh = 0.6 * (0 - atan2(nd(2), nd(1)) - atan2(uvms.wTv(2, 1), uvms.wTv(1, 1)));
end
uvms.xdot.vh = Saturate(uvms.xdot.vh, 0.8); % saturation, computed in every task

% VP vehicle position task reference
uvms.xdot.vp = 1 * lin;
if mission.phase == 3 % landing task
    uvms.xdot.vp = [0; 0; 0.6 * (0 - uvms.altitude)];
elseif mission.phase == 4
     uvms.xdot.vp = [0; 0; 0]; % No motor motions when landed
end
uvms.xdot.vp = Saturate(uvms.xdot.vp, 0.8);

% VA Veichle Aligning
% in this case the rover mus be aligned with the rock
% compute the theta_dot reference (angular error)
d = uvms.rock_center(1:2) - uvms.wTv(1:2, 4); % distance vector between rock and rover
nd = d / norm(d); % normalized distance  
uvms.angle = atan2(nd(2), nd(1)) + atan2(uvms.wTv(2, 1), uvms.wTv(1, 1));
uvms.xdot.va = 0.6 * (0 - atan2(nd(2), nd(1)) - atan2(uvms.wTv(2, 1), uvms.wTv(1, 1)));
uvms.xdot.va = Saturate(uvms.xdot.va, 0.8);

% AC attitude control task
uvms.xdot.ac = 0.6 * ang(1:2);
uvms.xdot.ac = Saturate(uvms.xdot.ac, 0.8);


% RN grasping task reference
[ang, lin] = CartError(uvms.wTg, uvms.wTt); % error between goal and tool
uvms.xdot.rn = 0.6 * [zeros(3,1); lin];


