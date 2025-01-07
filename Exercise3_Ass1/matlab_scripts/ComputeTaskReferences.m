function [uvms] = ComputeTaskReferences(uvms, mission)
% compute the task references here

% reference for tool-frame position control task
[ang, lin] = CartError(uvms.vTg , uvms.vTt);
uvms.xdot.t = 0.2 * [ang; lin];

% limit the requested velocities...
uvms.xdot.t(1:3) = Saturate(uvms.xdot.t(1:3), 0.2);
uvms.xdot.t(4:6) = Saturate(uvms.xdot.t(4:6), 0.2);

% task reference for minimum altitude
uvms.xdot.ma = 0.2 * (2 - uvms.altitude);

% task reference for horizontal attitude
uvms.xdot.ha = -0.4 * (uvms.p(4:5) - [deg2rad(0); deg2rad(0)]);

% computing angular and linear error between vehicle goal and vehicle
[ang, lin] = CartError(uvms.wTgv, uvms.wTv);

% vehicle position task reference
uvms.xdot.gv = 1 * lin;
uvms.xdot.gv = Saturate(uvms.xdot.gv, 0.8);

% attitude control task
uvms.xdot.ac = 0.6 * ang(1:2);

% vehicle heading control task reference
if mission.phase == 1    
    uvms.xdot.vh = 0.6 * ang(3);
    uvms.xdot.vh = Saturate(uvms.xdot.vh, 0.8);
elseif mission.phase == 2
    % new target for veiche heading control
    target_vec = uvms.rock_center(1:2) - uvms.p(1:2);
    unit_vec = target_vec / abs(target_vec);
    angle = atan2(unit_vec(2), unit_vec(1));
    disp(angle)
    uvms.xdot.vh = -0.6 * (0 - angle);
  
end

% grasping task reference
% error between goal and tool
[ang, lin] = CartError(uvms.wTg, uvms.wTt);
uvms.xdot.gr = 0.6 * [zeros(3,1); lin];


